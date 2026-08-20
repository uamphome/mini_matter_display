#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback invoked during each LVGL flush. Used by snapshot to capture pixels.
 */
typedef void (*amoled_flush_hook_t)(const lv_area_t *area, lv_color_t *color_map);

/**
 * Initialize LVGL: lv_init, draw buffers (DMA), display driver with rounder,
 * tick timer, and touch input device.
 *
 * @param panel  Panel handle from amoled_init()
 * @param touch  Touch handle from amoled_touch_init() (NULL to skip touch)
 */
esp_err_t amoled_lvgl_init(esp_lcd_panel_handle_t panel, esp_lcd_touch_handle_t touch);

/**
 * Set how much DMA-capable heap the draw buffers must leave free, before calling
 * amoled_lvgl_init(). On this SoC all internal SRAM is DMA-capable, so the draw buffers compete
 * with the network stack: a caller that is about to do something memory-hungry (e.g. a Matter
 * commissioning handshake, which runs with BLE still resident) should raise this so the buffers
 * shrink instead of starving it. Has no effect once amoled_lvgl_init() has run.
 */
void amoled_lvgl_set_dma_reserve(size_t bytes);

/**
 * Get the registered LVGL display.
 */
lv_disp_t *amoled_lvgl_get_disp(void);

/**
 * Set a hook that is called during every LVGL flush with the area and pixel data.
 * Set to NULL to disable. Used by snapshot module for flush-callback capture.
 */
void amoled_lvgl_set_flush_hook(amoled_flush_hook_t hook);

/**
 * Enable/disable the touch input device. When disabled, the touch read
 * callback reports RELEASED and skips the I2C read entirely. Useful when
 * blanking the screen for power-saving so stray touches don't trigger UI.
 * Touch is enabled by default after amoled_lvgl_init().
 */
void amoled_lvgl_set_touch_enabled(bool enabled);

#ifdef __cplusplus
}
#endif
