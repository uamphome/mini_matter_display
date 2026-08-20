#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_io_expander.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Pin definitions */
#define AMOLED_QSPI_SCLK    0
#define AMOLED_QSPI_DATA0   1
#define AMOLED_QSPI_DATA1   2
#define AMOLED_QSPI_DATA2   3
#define AMOLED_QSPI_DATA3   4
#define AMOLED_QSPI_CS      5

/* QSPI pixel clock. The esp_lcd_sh8601 macro defaults to 40 MHz; SPI2 on the C6 and the SH8601
 * both go to 80. A full 368x448x16bpp frame is ~330 KB, i.e. ~16.5 ms of DMA at 40 MHz vs ~8.2 ms
 * at 80 — halving the window in which the panel scans out a partially-written frame is the main
 * lever against tearing, and it frees the LVGL task from waiting on the bus.
 *
 * Caveat: none of the board's QSPI pins are SPI2 IOMUX pins (C6 IOMUX wants CLK=6, MOSI=7, CS=16;
 * the board wires CLK=0, D0=1, CS=5), so the bus runs through the GPIO matrix. The matrix delay
 * only compromises *input* sampling, and this is a write-only display on a half-duplex quad bus,
 * so the driver accepts 80 MHz without dummy-cycle compensation. If the panel ever shows garbage
 * or dropped strips, drop this back to 40 MHz — the SPI clock is 80 MHz / N, so 40 is the only
 * step below 80 (there is no usable 60). */
#define AMOLED_QSPI_PCLK_HZ (80 * 1000 * 1000)

#define AMOLED_I2C_SCL       7
#define AMOLED_I2C_SDA       8
#define AMOLED_I2C_NUM       I2C_NUM_0
#define AMOLED_I2C_FREQ_HZ   200000

#define AMOLED_LCD_H_RES     368
#define AMOLED_LCD_V_RES     448

/**
 * Initialize hardware: I2C master bus, TCA9554 IO expander (power on display + touch),
 * AXP2101 PMIC (battery + long-press shutdown), SPI2 QSPI bus, SH8601 AMOLED panel.
 */
esp_err_t amoled_init(void);

/** Get the I2C master bus handle (for touch and other I2C peripherals). */
i2c_master_bus_handle_t amoled_get_i2c_bus(void);

/** Get the initialized panel handle. */
esp_lcd_panel_handle_t amoled_get_panel(void);

/** Get the panel IO handle. */
esp_lcd_panel_io_handle_t amoled_get_panel_io(void);

/** Get the TCA9554 IO expander handle (for P7 speaker amp, etc.). */
esp_io_expander_handle_t amoled_get_io_expander(void);

/** Mutex-guarded TCA9554 access. The expander's output register is shared between the display power
 *  pins (P4/P5, toggled by the LVGL task) and the speaker PA pin (P7, toggled by the audio task);
 *  since set_level is a non-atomic I2C read-modify-write, all callers must go through these so the
 *  updates don't clobber each other. pin_mask uses IO_EXPANDER_PIN_NUM_*. */
esp_err_t amoled_io_expander_set_dir(uint32_t pin_mask, bool output);
esp_err_t amoled_io_expander_set_level(uint32_t pin_mask, uint8_t level);

/** Set AMOLED brightness via SH8601 register 0x51. level: 0-255. */
esp_err_t amoled_set_brightness(uint8_t level);

/** Turn display on/off. Off = low-power mode (0x28), On = resume (0x29). */
esp_err_t amoled_display_on_off(bool on);

/** Deep display sleep: stop the panel and CUT the AMOLED + SH8601 driver power
 *  rail via the TCA9554 (P4 = LOW). The touch controller (P5) stays powered, so
 *  a touch can still wake the device. Pair with amoled_wake(). */
esp_err_t amoled_sleep(void);

/** Wake from amoled_sleep(): restore the AMOLED power rail (P4 = HIGH) and
 *  re-initialize the SH8601 panel. The framebuffer is lost across a power cut,
 *  so the caller must redraw the screen afterwards. */
esp_err_t amoled_wake(void);

/** Release SPI2 bus — deletes panel IO device, frees bus. Call when display is OFF. */
esp_err_t amoled_release_spi(void);

/** Reclaim SPI2 bus — reinits QSPI, recreates panel IO. Call before display ON. */
esp_err_t amoled_reclaim_spi(void);

/** Get touch INT GPIO number for strapping pin reference */
#define AMOLED_TOUCH_INT_GPIO  15

/* ── AXP2101 Power Management ─────────────────────────────── */

/** Battery info structure. */
typedef struct {
    uint16_t voltage_mv;    /* Battery voltage in mV */
    uint8_t  percentage;    /* State of charge 0-100% */
    bool     charging;      /* True if charging */
    bool     vbus_present;  /* True if USB power present */
    bool     battery_present; /* True if battery detected */
} amoled_battery_info_t;

/** Read battery status from AXP2101. */
esp_err_t amoled_get_battery_info(amoled_battery_info_t *info);

/** Trigger software power-off via AXP2101. Cuts all power rails. */
esp_err_t amoled_power_off(void);

#ifdef __cplusplus
}
#endif
