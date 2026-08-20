#pragma once

#include "esp_err.h"
#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AMOLED_TOUCH_INT_GPIO  15

/**
 * Initialize FT3168 touch controller via I2C (I2C bus must already be initialized).
 * @param out_touch  Returned touch handle for LVGL registration
 */
esp_err_t amoled_touch_init(esp_lcd_touch_handle_t *out_touch);

#ifdef __cplusplus
}
#endif
