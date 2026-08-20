#include "amoled_lvgl.h"
#include "amoled.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "lvgl.h"

static const char *TAG = "lvgl_drv";

/* LVGL draw-buffer height (lines). Bigger = fewer flush chunks per frame = fewer tear seams during
 * animations/swipes, at the cost of DMA-capable RAM (2 buffers, 368*L*2 bytes each). We can't use a
 * full-frame buffer (no PSRAM on the C6; ~322 KB won't fit alongside Matter+Thread), so this is the
 * anti-tearing lever. 74 ≈ 1/6 of 448 (6 chunks vs 10 at 45), keeping ~55 KB DMA headroom for the
 * boot-time network storm. Waveshare's RAM example uses 112 (1/4); we can't afford that much here. */
#define LVGL_BUF_LINES   74
#define LVGL_TICK_MS      2

/* Heap the draw buffers must leave free. On the C6 all internal SRAM is DMA-capable, so the draw
 * buffers compete directly with the network stack, and sizing is "largest pair that still leaves
 * this much" rather than "largest pair that fits". The default suits an idle device; a caller
 * expecting heavy allocation (Matter commissioning) raises it via
 * amoled_lvgl_set_dma_reserve(). */
#define LVGL_DMA_RESERVE_DEFAULT (40 * 1024)
static size_t s_dma_reserve = LVGL_DMA_RESERVE_DEFAULT;

void amoled_lvgl_set_dma_reserve(size_t bytes)
{
    s_dma_reserve = bytes;
}

static lv_disp_t          *s_disp = NULL;
static lv_disp_drv_t       s_disp_drv;
static lv_disp_draw_buf_t  s_draw_buf;
static amoled_flush_hook_t s_flush_hook = NULL;

/* Cached at touch register time so the read callback doesn't poke into the
 * touch handle's private struct on every poll. -1 means "no INT pin". */
static int  s_touch_int_gpio = -1;
static bool s_touch_enabled  = true;

/* ── ISR: DMA transfer to display complete ────────────────── */
static bool flush_ready_cb(esp_lcd_panel_io_handle_t io,
                           esp_lcd_panel_io_event_data_t *edata,
                           void *user_ctx)
{
    lv_disp_drv_t *drv = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(drv);
    return false;
}

/* ── LVGL flush callback ─────────────────────────────────── */
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area,
                          lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)drv->user_data;
    esp_lcd_panel_draw_bitmap(panel,
        area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);

    /* Snapshot hook — called for every strip during forced refresh */
    if (s_flush_hook) {
        s_flush_hook(area, color_map);
    }
}

/* ── SH8601 rounder: coordinates must be even ────────────── */
static void lvgl_rounder_cb(lv_disp_drv_t *drv, lv_area_t *area)
{
    area->x1 = (area->x1 >> 1) << 1;
    area->y1 = (area->y1 >> 1) << 1;
    area->x2 = ((area->x2 >> 1) << 1) + 1;
    area->y2 = ((area->y2 >> 1) << 1) + 1;
}

/* ── Touch read callback ─────────────────────────────────── */
static void lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)drv->user_data;
    if (!tp || !s_touch_enabled) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    /* FT3168 only has valid data when INT is asserted (active LOW). Reading
     * while INT is HIGH causes periodic I2C NACK bursts when the chip is
     * idle/asleep. */
    if (s_touch_int_gpio >= 0 && gpio_get_level(s_touch_int_gpio) == 1) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    esp_lcd_touch_read_data(tp);

    uint16_t x, y;
    uint8_t cnt = 0;
    bool pressed = esp_lcd_touch_get_coordinates(tp, &x, &y, NULL, &cnt, 1);
    if (pressed && cnt > 0) {
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/* ── LVGL tick ───────────────────────────────────────────── */
static void lvgl_tick_cb(void *arg)
{
    lv_tick_inc(LVGL_TICK_MS);
}

/* ── Public API ──────────────────────────────────────────── */

esp_err_t amoled_lvgl_init(esp_lcd_panel_handle_t panel, esp_lcd_touch_handle_t touch)
{
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "null panel");

    /* Register DMA-done callback on the panel IO */
    esp_lcd_panel_io_handle_t io = amoled_get_panel_io();
    if (io) {
        const esp_lcd_panel_io_callbacks_t cbs = {
            .on_color_trans_done = flush_ready_cb,
        };
        esp_lcd_panel_io_register_event_callbacks(io, &cbs, &s_disp_drv);
    }

    ESP_LOGI(TAG, "lv_init()");
    lv_init();

    /* Allocate the draw buffers in DMA-capable memory, degrading gracefully if RAM is tight. The
     * ideal is two LVGL_BUF_LINES buffers (fewest flush chunks), but DMA RAM is scarcest on a FRESH,
     * uncommissioned boot: CHIP keeps the BLE stack (~40 KB) resident until pairing completes, and
     * the commissioning CASE handshake needs heap of its own. So each candidate size must leave
     * s_dma_reserve free, not merely fit — otherwise the display comes up and pairing then fails on
     * an allocation. Buffers grow back on the next boot, once BLE is freed. */
    size_t free_dma = heap_caps_get_free_size(MALLOC_CAP_DMA);
    ESP_LOGI(TAG, "Free DMA before draw buffers: %lu bytes", (unsigned long)free_dma);
    static const int kBufLineOptions[] = { LVGL_BUF_LINES, 50, 36, 24, 16 };
    lv_color_t *buf1 = NULL, *buf2 = NULL;
    int   chosen_lines = 0;
    bool  double_buffered = true;
    for (size_t i = 0; i < sizeof(kBufLineOptions) / sizeof(kBufLineOptions[0]); i++) {
        size_t by = (size_t)AMOLED_LCD_H_RES * kBufLineOptions[i] * sizeof(lv_color_t);
        if (heap_caps_get_free_size(MALLOC_CAP_DMA) < 2 * by + s_dma_reserve) {
            continue;   /* would starve the Matter/BLE stack — try a smaller pair */
        }
        buf1 = heap_caps_malloc(by, MALLOC_CAP_DMA);
        buf2 = heap_caps_malloc(by, MALLOC_CAP_DMA);
        if (buf1 && buf2) { chosen_lines = kBufLineOptions[i]; break; }
        if (buf1) { heap_caps_free(buf1); buf1 = NULL; }
        if (buf2) { heap_caps_free(buf2); buf2 = NULL; }
    }
    if (!chosen_lines) {
        /* Last resort: a single small buffer (LVGL supports one-buffer mode). */
        size_t by = (size_t)AMOLED_LCD_H_RES * 16 * sizeof(lv_color_t);
        buf1 = heap_caps_malloc(by, MALLOC_CAP_DMA);
        if (!buf1) {
            ESP_LOGE(TAG, "Failed to allocate even a single %d-line draw buffer (%zu bytes)", 16, by);
            return ESP_ERR_NO_MEM;
        }
        buf2 = NULL;
        chosen_lines = 16;
        double_buffered = false;
    }
    size_t buf_pixels = (size_t)AMOLED_LCD_H_RES * chosen_lines;
    size_t free_after = heap_caps_get_free_size(MALLOC_CAP_DMA);
    if (chosen_lines == LVGL_BUF_LINES) {
        ESP_LOGI(TAG, "Draw buffers: 2 x %zu bytes (%d lines) DMA, %lu bytes DMA left",
                 buf_pixels * sizeof(lv_color_t), chosen_lines, (unsigned long)free_after);
    } else {
        /* Logged at W so it stays visible in the default (INFO-stripped) build — it is the first
         * thing to check if commissioning ever fails on an allocation. */
        ESP_LOGW(TAG, "DMA RAM tight (%lu free) — draw buffers %s %d lines, %lu bytes left",
                 (unsigned long)free_dma, double_buffered ? "2 x" : "1 x", chosen_lines,
                 (unsigned long)free_after);
    }
    lv_disp_draw_buf_init(&s_draw_buf, buf1, buf2, buf_pixels);

    /* Display driver */
    lv_disp_drv_init(&s_disp_drv);
    s_disp_drv.hor_res    = AMOLED_LCD_H_RES;
    s_disp_drv.ver_res    = AMOLED_LCD_V_RES;
    s_disp_drv.flush_cb   = lvgl_flush_cb;
    s_disp_drv.rounder_cb = lvgl_rounder_cb;
    s_disp_drv.draw_buf   = &s_draw_buf;
    s_disp_drv.user_data  = panel;
    s_disp = lv_disp_drv_register(&s_disp_drv);
    ESP_LOGI(TAG, "Display driver registered (%dx%d, rounder ON)",
             AMOLED_LCD_H_RES, AMOLED_LCD_V_RES);

    /* Tick timer */
    const esp_timer_create_args_t tick_args = {
        .callback = lvgl_tick_cb,
        .name = "lvgl_tick",
    };
    esp_timer_handle_t tick_timer;
    ESP_RETURN_ON_ERROR(esp_timer_create(&tick_args, &tick_timer), TAG, "tick create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(tick_timer, LVGL_TICK_MS * 1000),
                        TAG, "tick start");
    ESP_LOGI(TAG, "Tick timer: %d ms", LVGL_TICK_MS);

    /* Touch input device */
    if (touch) {
        s_touch_int_gpio = touch->config.int_gpio_num;
        s_touch_enabled  = true;

        static lv_indev_drv_t indev_drv;
        lv_indev_drv_init(&indev_drv);
        indev_drv.type    = LV_INDEV_TYPE_POINTER;
        indev_drv.disp    = s_disp;
        indev_drv.read_cb = lvgl_touch_cb;
        indev_drv.user_data = touch;
        lv_indev_drv_register(&indev_drv);
        ESP_LOGI(TAG, "Touch input registered (INT=GPIO%d)", s_touch_int_gpio);
    } else {
        ESP_LOGW(TAG, "No touch — skipping input device");
    }

    ESP_LOGI(TAG, "LVGL ready (free heap: %lu, free DMA: %lu)",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA));
    return ESP_OK;
}

lv_disp_t *amoled_lvgl_get_disp(void)
{
    return s_disp;
}

void amoled_lvgl_set_flush_hook(amoled_flush_hook_t hook)
{
    s_flush_hook = hook;
}

void amoled_lvgl_set_touch_enabled(bool enabled)
{
    s_touch_enabled = enabled;
}
