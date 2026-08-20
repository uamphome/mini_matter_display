#include "amoled_touch.h"
#include "amoled.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch_ft5x06.h"

static const char *TAG = "touch";

esp_err_t amoled_touch_init(esp_lcd_touch_handle_t *out_touch)
{
    ESP_RETURN_ON_FALSE(out_touch, ESP_ERR_INVALID_ARG, TAG, "null out_touch");

    /* FT3168 needs time after TCA9554 powers it on (P5=HIGH) */
    ESP_LOGI(TAG, "Waiting 300ms for FT3168 power-up...");
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_LOGI(TAG, "FT3168 init (addr=0x38, INT=%d)", AMOLED_TOUCH_INT_GPIO);

    /* I2C IO for touch — uses the I2C master bus from amoled_init() */
    i2c_master_bus_handle_t i2c_bus = amoled_get_i2c_bus();
    ESP_RETURN_ON_FALSE(i2c_bus, ESP_ERR_INVALID_STATE, TAG, "I2C bus not initialized");

    esp_lcd_panel_io_handle_t tp_io = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_i2c(i2c_bus, &tp_io_config, &tp_io),
        TAG, "Touch IO create");

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = AMOLED_LCD_H_RES,
        .y_max = AMOLED_LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = AMOLED_TOUCH_INT_GPIO,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    ESP_RETURN_ON_ERROR(
        esp_lcd_touch_new_i2c_ft5x06(tp_io, &tp_cfg, out_touch),
        TAG, "FT3168 init");

    ESP_LOGI(TAG, "FT3168 touch ready (max %dx%d)", AMOLED_LCD_H_RES, AMOLED_LCD_V_RES);
    return ESP_OK;
}
