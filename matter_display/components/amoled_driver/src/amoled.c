#include "amoled.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_io_expander_tca9554.h"
#include "esp_lcd_sh8601.h"
#include "esp_lcd_panel_interface.h"

static const char *TAG = "amoled";

#define LCD_HOST  SPI2_HOST
#define LCD_BIT_PER_PIXEL  16

static i2c_master_bus_handle_t   s_i2c_bus      = NULL;
static esp_lcd_panel_handle_t    s_panel        = NULL;
static esp_lcd_panel_io_handle_t s_panel_io     = NULL;
static esp_io_expander_handle_t  s_io_expander  = NULL;
/* The TCA9554 output register is shared: the LVGL task toggles P4/P5 (display + touch power) while
 * the audio task toggles P7 (speaker PA). esp_io_expander_set_level is a non-atomic I2C read-modify-
 * write, so serialize all expander access through this mutex. */
static SemaphoreHandle_t         s_ioexp_mtx    = NULL;

esp_err_t amoled_io_expander_set_dir(uint32_t pin_mask, bool output)
{
    if (!s_io_expander) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ioexp_mtx) {
        xSemaphoreTake(s_ioexp_mtx, portMAX_DELAY);
    }
    esp_err_t err = esp_io_expander_set_dir(s_io_expander, pin_mask,
                                            output ? IO_EXPANDER_OUTPUT : IO_EXPANDER_INPUT);
    if (s_ioexp_mtx) {
        xSemaphoreGive(s_ioexp_mtx);
    }
    return err;
}

esp_err_t amoled_io_expander_set_level(uint32_t pin_mask, uint8_t level)
{
    if (!s_io_expander) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ioexp_mtx) {
        xSemaphoreTake(s_ioexp_mtx, portMAX_DELAY);
    }
    esp_err_t err = esp_io_expander_set_level(s_io_expander, pin_mask, level);
    if (s_ioexp_mtx) {
        xSemaphoreGive(s_ioexp_mtx);
    }
    return err;
}

/* ── AXP2101 registers ───────────────────────────────────── */
#define AXP2101_ADDR            0x34
#define AXP2101_STATUS1         0x00
#define AXP2101_STATUS2         0x01
#define AXP2101_COMMON_CFG      0x10  /* bit 0: software shutdown */
#define AXP2101_PWROFF_EN       0x22  /* bit 1: long press shutdown enable */
#define AXP2101_IRQ_OFF_ON_LVL  0x27  /* bits [1:0]: long press duration */
#define AXP2101_INTEN2          0x41  /* power key interrupt enable */
#define AXP2101_INTSTS2         0x49  /* power key interrupt status */
#define AXP2101_ADC_CH_CTRL     0x30  /* ADC channel enable: bit0=batt, bit1=TS, bit2=vbus, bit3=sys */
#define AXP2101_CHG_V_SET       0x64  /* charge target voltage */
#define AXP2101_CHG_I_SET       0x62  /* charge current setting */
#define AXP2101_BAT_V_H         0x34  /* battery voltage high byte */
#define AXP2101_BAT_V_L         0x35  /* battery voltage low byte */
#define AXP2101_BAT_PERCENT     0xA4  /* fuel gauge percentage */

static i2c_master_dev_handle_t s_axp_dev = NULL;

/* SH8601 init commands from Waveshare official reference */
static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
    {0x11, (uint8_t[]){0x00}, 0, 120},
    {0x44, (uint8_t[]){0x01, 0xD1}, 2, 0},
    {0x35, (uint8_t[]){0x00}, 1, 0},
    {0x53, (uint8_t[]){0x20}, 1, 10},
    {0x2A, (uint8_t[]){0x00, 0x00, 0x01, 0x6F}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xBF}, 4, 0},
    {0x51, (uint8_t[]){0x00}, 1, 10},
    {0x29, (uint8_t[]){0x00}, 0, 10},
    {0x51, (uint8_t[]){0xD0}, 1, 0},
};

/* ── AXP2101 I2C helpers ─────────────────────────────────── */

static esp_err_t axp_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_axp_dev, &reg, 1, val, 1, 100);
}

static esp_err_t axp_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_axp_dev, buf, 2, 100);
}

static esp_err_t axp_set_bits(uint8_t reg, uint8_t mask)
{
    uint8_t val;
    ESP_RETURN_ON_ERROR(axp_read_reg(reg, &val), TAG, "AXP read 0x%02x", reg);
    val |= mask;
    return axp_write_reg(reg, val);
}

static esp_err_t axp_init(void)
{
    /* Add AXP2101 device to I2C bus */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AXP2101_ADDR,
        .scl_speed_hz = AMOLED_I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_axp_dev),
        TAG, "AXP2101 add device");

    /* Verify communication — read status register */
    uint8_t status;
    ESP_RETURN_ON_ERROR(axp_read_reg(AXP2101_STATUS1, &status), TAG, "AXP2101 read status");
    ESP_LOGI(TAG, "AXP2101 status1=0x%02x", status);

    /* CRITICAL: Disable TS pin measurement — board has no battery thermistor.
     * Without this, charging may malfunction (Waveshare reference code does this). */
    uint8_t adc_ctrl;
    axp_read_reg(AXP2101_ADC_CH_CTRL, &adc_ctrl);
    adc_ctrl &= ~0x02;  /* Clear bit 1 = disable TS pin */
    adc_ctrl |= 0x0D;   /* Enable: bit0=batt voltage, bit2=VBUS, bit3=system voltage */
    axp_write_reg(AXP2101_ADC_CH_CTRL, adc_ctrl);
    ESP_LOGI(TAG, "AXP2101 TS pin disabled, battery/VBUS/sys ADC enabled (0x%02x)", adc_ctrl);

    /* Set charge current to 200mA (per Waveshare reference) */
    axp_write_reg(AXP2101_CHG_I_SET, 0x04);  /* 200mA */
    ESP_LOGI(TAG, "AXP2101 charge current: 200mA");

    /* Set charge target voltage to 4.1V (per Waveshare reference — NOT 4.2V) */
    uint8_t chg_v;
    axp_read_reg(AXP2101_CHG_V_SET, &chg_v);
    chg_v = (chg_v & 0xFC) | 0x01;  /* bits[1:0]: 00=4.0V, 01=4.1V, 02=4.15V, 03=4.2V */
    axp_write_reg(AXP2101_CHG_V_SET, chg_v);
    ESP_LOGI(TAG, "AXP2101 charge target: 4.1V");

    /* Enable long press shutdown (2.5s default) */
    ESP_RETURN_ON_ERROR(axp_set_bits(AXP2101_PWROFF_EN, 0x02), TAG, "AXP long press enable");
    ESP_LOGI(TAG, "AXP2101 long-press shutdown enabled (2.5s)");

    /* Enable power key IRQs (short + long press) */
    ESP_RETURN_ON_ERROR(axp_set_bits(AXP2101_INTEN2, 0x0C), TAG, "AXP IRQ enable");
    ESP_LOGI(TAG, "AXP2101 power key IRQ enabled");

    /* Clear any pending IRQ */
    axp_write_reg(AXP2101_INTSTS2, 0xFF);

    return ESP_OK;
}

/* ── Public API: init ────────────────────────────────────── */

esp_err_t amoled_init(void)
{
    /* ── 1. I2C master bus (new driver) ──────────────────────── */
    ESP_LOGI(TAG, "I2C master bus init (SCL=%d, SDA=%d, %dHz)",
             AMOLED_I2C_SCL, AMOLED_I2C_SDA, AMOLED_I2C_FREQ_HZ);
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = AMOLED_I2C_NUM,
        .sda_io_num = AMOLED_I2C_SDA,
        .scl_io_num = AMOLED_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_i2c_bus), TAG, "I2C bus create");
    ESP_LOGI(TAG, "I2C master bus ready");

    /* ── 2. AXP2101 PMIC ────────────────────────────────────── */
    ESP_LOGI(TAG, "AXP2101 PMIC init (addr=0x%02x)", AXP2101_ADDR);
    esp_err_t axp_ret = axp_init();
    if (axp_ret != ESP_OK) {
        ESP_LOGW(TAG, "AXP2101 init failed (%s) — continuing without PMIC",
                 esp_err_to_name(axp_ret));
    }

    /* ── 3. TCA9554 IO Expander ──────────────────────────────── */
    ESP_LOGI(TAG, "TCA9554 init (addr=0x20)");
    ESP_RETURN_ON_ERROR(
        esp_io_expander_new_i2c_tca9554(s_i2c_bus,
            ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000, &s_io_expander),
        TAG, "TCA9554 create");
    s_ioexp_mtx = xSemaphoreCreateMutex();

    /* Power cycle: LOW → 200ms → HIGH */
    esp_io_expander_set_dir(s_io_expander,
        IO_EXPANDER_PIN_NUM_4 | IO_EXPANDER_PIN_NUM_5, IO_EXPANDER_OUTPUT);
    esp_io_expander_set_level(s_io_expander, IO_EXPANDER_PIN_NUM_4, 0);
    esp_io_expander_set_level(s_io_expander, IO_EXPANDER_PIN_NUM_5, 0);
    ESP_LOGI(TAG, "TCA9554 P4/P5 LOW — power cycling display+touch");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_io_expander_set_level(s_io_expander, IO_EXPANDER_PIN_NUM_4, 1);
    esp_io_expander_set_level(s_io_expander, IO_EXPANDER_PIN_NUM_5, 1);
    ESP_LOGI(TAG, "TCA9554 P4/P5 HIGH — display+touch powered");

    /* ── 4. QSPI bus ─────────────────────────────────────────── */
    ESP_LOGI(TAG, "SPI2 QSPI bus init");
    const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(
        AMOLED_QSPI_SCLK,
        AMOLED_QSPI_DATA0, AMOLED_QSPI_DATA1,
        AMOLED_QSPI_DATA2, AMOLED_QSPI_DATA3,
        AMOLED_LCD_H_RES * AMOLED_LCD_V_RES * LCD_BIT_PER_PIXEL / 8);
    esp_err_t spi_ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (spi_ret != ESP_OK && spi_ret != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(spi_ret, TAG, "SPI bus init");
    }
    if (spi_ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI2 bus already initialized — freeing and reconfiguring for QSPI");
        spi_bus_free(LCD_HOST);
        ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO),
                            TAG, "SPI bus reinit");
    }

    /* ── 5. SH8601 panel ─────────────────────────────────────── */
    ESP_LOGI(TAG, "SH8601 panel init (CS=%d, QSPI %d MHz)",
             AMOLED_QSPI_CS, AMOLED_QSPI_PCLK_HZ / 1000000);
    esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(
        AMOLED_QSPI_CS, NULL, NULL);
    io_config.pclk_hz = AMOLED_QSPI_PCLK_HZ;
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &s_panel_io),
        TAG, "Panel IO create");

    sh8601_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags.use_qspi_interface = 1,
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_sh8601(s_panel_io, &panel_config, &s_panel),
                        TAG, "Panel create");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "Panel reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "Panel init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "Panel on");

    ESP_LOGI(TAG, "SH8601 AMOLED ready (%dx%d)", AMOLED_LCD_H_RES, AMOLED_LCD_V_RES);
    return ESP_OK;
}

i2c_master_bus_handle_t amoled_get_i2c_bus(void)
{
    return s_i2c_bus;
}

esp_lcd_panel_handle_t amoled_get_panel(void)
{
    return s_panel;
}

esp_lcd_panel_io_handle_t amoled_get_panel_io(void)
{
    return s_panel_io;
}

esp_io_expander_handle_t amoled_get_io_expander(void)
{
    return s_io_expander;
}

esp_err_t amoled_set_brightness(uint8_t level)
{
    ESP_LOGI(TAG, "Brightness → %d", level);
    const uint8_t data = level;
    /* SH8601 QSPI command format: opcode(0x02) << 24 | cmd << 8 */
    int cmd = (0x02UL << 24) | (0x51 << 8);
    return esp_lcd_panel_io_tx_param(s_panel_io, cmd, &data, 1);
}

esp_err_t amoled_release_spi(void)
{
    ESP_LOGI(TAG, "Releasing SPI2 — deleting panel IO");
    if (s_panel_io) {
        esp_lcd_panel_io_del(s_panel_io);
        s_panel_io = NULL;
    }
    esp_err_t ret = spi_bus_free(LCD_HOST);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "spi_bus_free: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "SPI2 released");
    return ESP_OK;
}

/*
 * The SH8601 panel internally caches the IO handle pointer.
 * After recreating panel IO, we must update that internal reference.
 * The sh8601_panel_t struct layout: { esp_lcd_panel_t base; esp_lcd_panel_io_handle_t io; ... }
 */
typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
} sh8601_panel_io_ref_t;

esp_err_t amoled_reclaim_spi(void)
{
    ESP_LOGI(TAG, "Reclaiming SPI2 for QSPI display");
    const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(
        AMOLED_QSPI_SCLK,
        AMOLED_QSPI_DATA0, AMOLED_QSPI_DATA1,
        AMOLED_QSPI_DATA2, AMOLED_QSPI_DATA3,
        AMOLED_LCD_H_RES * AMOLED_LCD_V_RES * LCD_BIT_PER_PIXEL / 8);

    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO),
                        TAG, "SPI2 reinit");

    esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(
        AMOLED_QSPI_CS, NULL, NULL);
    io_config.pclk_hz = AMOLED_QSPI_PCLK_HZ;
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &s_panel_io),
        TAG, "Panel IO re-create");

    /* Update the panel's internal IO reference (struct hack — verified layout matches) */
    if (s_panel) {
        sh8601_panel_io_ref_t *p = (sh8601_panel_io_ref_t *)s_panel;
        ESP_LOGI(TAG, "Panel IO: old=%p new=%p", p->io, s_panel_io);
        p->io = s_panel_io;
    }

    /* Send display-off to ensure clean state, then re-init will happen via display_on_off */
    int cmd_off = (0x02UL << 24) | (0x28 << 8);
    esp_lcd_panel_io_tx_param(s_panel_io, cmd_off, NULL, 0);

    ESP_LOGI(TAG, "SPI2 QSPI reclaimed — panel IO updated, display in OFF state");
    return ESP_OK;
}

esp_err_t amoled_display_on_off(bool on)
{
    /* 0x29 = Display On, 0x28 = Display Off (low-power, stops AMOLED scanning) */
    uint8_t raw_cmd = on ? 0x29 : 0x28;
    int cmd = (0x02UL << 24) | (raw_cmd << 8);
    ESP_LOGI(TAG, "Display %s (cmd 0x%02X)", on ? "ON" : "OFF", raw_cmd);
    return esp_lcd_panel_io_tx_param(s_panel_io, cmd, NULL, 0);
}

esp_err_t amoled_sleep(void)
{
    if (!s_io_expander) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "Display sleep: panel off + cut AMOLED power (TCA9554 P4 LOW)");
    /* Stop scanning first (best effort), then cut the AMOLED display power rail.
     * P5 (touch power) is left HIGH so a touch can still wake us. */
    if (s_panel) {
        esp_lcd_panel_disp_on_off(s_panel, false);
    }
    return amoled_io_expander_set_level(IO_EXPANDER_PIN_NUM_4, 0);
}

esp_err_t amoled_wake(void)
{
    if (!s_io_expander || !s_panel) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "Display wake: restore AMOLED power (TCA9554 P4 HIGH) + re-init panel");
    ESP_RETURN_ON_ERROR(amoled_io_expander_set_level(IO_EXPANDER_PIN_NUM_4, 1),
                        TAG, "P4 high");
    vTaskDelay(pdMS_TO_TICKS(120));   /* let the AMOLED rail + SH8601 power up */
    /* Power was cut, so the SH8601 lost all state — re-run reset + full init. */
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "wake reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "wake init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "wake on");
    return ESP_OK;
}

/* ── AXP2101 battery + power ────────────────────────────── */

esp_err_t amoled_get_battery_info(amoled_battery_info_t *info)
{
    if (!s_axp_dev || !info) return ESP_ERR_INVALID_STATE;
    memset(info, 0, sizeof(*info));

    /* Status registers */
    uint8_t s1, s2;
    ESP_RETURN_ON_ERROR(axp_read_reg(AXP2101_STATUS1, &s1), TAG, "read status1");
    ESP_RETURN_ON_ERROR(axp_read_reg(AXP2101_STATUS2, &s2), TAG, "read status2");
    info->vbus_present    = (s1 >> 5) & 1;
    info->charging        = ((s2 >> 5) & 0x07) == 1; /* charger state: 1 = charging */
    info->battery_present = (s1 >> 3) & 1;

    /* Battery voltage: 14-bit value split across two registers */
    uint8_t vh, vl;
    ESP_RETURN_ON_ERROR(axp_read_reg(AXP2101_BAT_V_H, &vh), TAG, "read bat_v_h");
    ESP_RETURN_ON_ERROR(axp_read_reg(AXP2101_BAT_V_L, &vl), TAG, "read bat_v_l");
    info->voltage_mv = ((uint16_t)(vh & 0x3F) << 8) | vl;

    /* Battery percentage */
    uint8_t pct;
    ESP_RETURN_ON_ERROR(axp_read_reg(AXP2101_BAT_PERCENT, &pct), TAG, "read bat_pct");
    info->percentage = (pct > 100) ? 100 : pct;

    return ESP_OK;
}

esp_err_t amoled_power_off(void)
{
    if (!s_axp_dev) return ESP_ERR_INVALID_STATE;
    ESP_LOGW(TAG, "Software power off — cutting all power rails");
    /* Set bit 0 of COMMON_CFG (0x10) to trigger shutdown */
    return axp_set_bits(AXP2101_COMMON_CFG, 0x01);
}
