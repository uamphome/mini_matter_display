/*
   Matter Display — LVGL user interface.

   Every screen and page on the 368x448 AMOLED: the pairing QR, the swipeable Home / Controls /
   Indoor / Weather / Settings pages, the modal per-device, time-and-date and confirmation
   overlays, the alert banner, and the LVGL task that drives them (including display sleep,
   touch-to-wake and the debounced Matter sends).

   This code is in the Public Domain (or CC0 licensed, at your option.) Unless required by
   applicable law or agreed to in writing, this software is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#include "light_ui.h"

#include "amoled.h"
#include "amoled_touch.h"
#include "amoled_lvgl.h"
#include "weather_icons.h"
#include <app_priv.h>

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_freertos_hooks.h"
#include "esp_system.h"
#include "esp_attr.h"
#include "lvgl.h"

static const char *TAG = "light_ui";

/* LVGL is not thread-safe. Every LVGL call must hold this mutex. It is taken by
 * the LVGL task around lv_timer_handler() and by light_ui_set_state() /
 * light_ui_set_brightness() when the Matter subscription thread updates the UI.
 *
 * Lock ordering: we must NEVER take the CHIP stack lock while holding this
 * mutex (the set_* functions run with the CHIP stack lock already held and then
 * take this mutex). So UI event handlers only update widgets here and set a
 * pending flag; the LVGL task sends the Matter command AFTER releasing this
 * mutex. */
static SemaphoreHandle_t s_lvgl_mtx = NULL;

static bool      s_state = false;          /* assumed light on/off (protected by s_lvgl_mtx) */

/* Brightness page */
static lv_obj_t *s_slider    = NULL;
static lv_obj_t *s_pct_label = NULL;
static int       s_pct       = 100;        /* 1-100 (protected by s_lvgl_mtx) */
static bool      s_dragging  = false;      /* user is touching the slider */

/* Color-temperature page. Range 2000–6000 K in 100 K steps (user spec). */
#define CT_KELVIN_MIN 2000
#define CT_KELVIN_MAX 6000
#define CT_KELVIN_STEP 100
static lv_obj_t *s_ct_slider   = NULL;
static lv_obj_t *s_ct_label    = NULL;
static int       s_ct_kelvin   = 4000;     /* 2000-6000 K (protected by s_lvgl_mtx) */
static bool      s_ct_dragging = false;    /* user is touching the CT slider */

/* ── Colour (RGB) mode — hue slider on the Controls page ─────── */
enum { COLOR_MODE_WARMTH = 0, COLOR_MODE_COLOUR = 1 };
static int       s_color_mode  = COLOR_MODE_WARMTH;  /* which control the toggle shows */
static lv_obj_t *s_seg_warmth  = NULL;
static lv_obj_t *s_seg_colour  = NULL;
static lv_obj_t *s_ct_card     = NULL;     /* Warmth card (shown in WARMTH mode) */
static lv_obj_t *s_hue_card    = NULL;     /* Colour card (shown in COLOUR mode) */
static lv_obj_t *s_hue_slider  = NULL;
static lv_obj_t *s_hue_name    = NULL;     /* colour name label */
static lv_obj_t *s_hue_swatch  = NULL;     /* current-colour chip */
static int       s_hue_deg     = 0;        /* 0-359 hue (protected by s_lvgl_mtx) */
static bool      s_hue_dragging = false;

/* Home page — weather block + light toggle card */
static lv_obj_t *s_light_card  = NULL;     /* the tappable light toggle card */
static lv_obj_t *s_light_state = NULL;     /* "ON" / "OFF" label */
static lv_obj_t *s_light_cap   = NULL;     /* "LIGHTS" caption (recoloured by state) */
static lv_obj_t *s_light_knob  = NULL;     /* the toggle knob dot */
static lv_obj_t *s_light_icon  = NULL;     /* bulb glyph badge */

/* Home page — ambient clock (time pushed by the Matter controller via TimeSync). */
static lv_obj_t *s_clock_time  = NULL;     /* big "HH:MM" */
static lv_obj_t *s_clock_date  = NULL;     /* "SUNDAY\n12 JUL" */
static int32_t   s_utc_offset_s = 0;       /* local offset (s) from the controller's timezone; 0 = UTC */

/* Manual clock override, set from the Time & Date settings page (tap the clock card). With
 * s_time_auto on (default) a controller SetUTCTime overrides any manual time; turn it off to keep
 * the manual time. The manual base + uptime reconstruct that wall time monotonically, so a
 * controller push can be undone while auto-sync is off. Not persisted across reboots. */
static bool      s_time_auto             = true;   /* sync from controller (default on)            */
static bool      s_have_manual           = false;  /* user has set a manual time this boot          */
static time_t    s_manual_utc_base       = 0;      /* the UTC epoch we last settimeofday()'d         */
static int64_t   s_manual_uptime_base_us = 0;      /* esp_timer uptime (us) when we set it           */

/* Per-device "Lights" page — each individually-controllable bound light / smart plug. Keyed by
 * (node, endpoint); populated from OnOff subscription reports. All protected by s_lvgl_mtx. */
#define MAX_UI_DEVICES 10
enum { DEV_KIND_LIGHT = 0, DEV_KIND_PLUG = 1 };
typedef struct { bool valid; bool have_state; uint8_t fabric; uint64_t node; uint16_t ep; int kind; bool on; } ui_device_t;
static ui_device_t s_ui_devices[MAX_UI_DEVICES];
static lv_obj_t   *s_devices_scr  = NULL;   /* modal per-device page (child of the active screen) */
static lv_obj_t   *s_devices_grid = NULL;   /* card grid inside it */

/* Modal "Time & Date" settings page (child of the active screen), opened by tapping the clock card. */
static lv_obj_t   *s_timeset_scr  = NULL;
static lv_obj_t   *s_ro_year = NULL, *s_ro_mon = NULL, *s_ro_day = NULL, *s_ro_hour = NULL, *s_ro_min = NULL;
static lv_obj_t   *s_auto_switch = NULL;   /* "sync from controller" toggle */
/* Pending per-device toggles, sent by the LVGL task outside the lock (like the other commands). */
typedef struct { uint8_t fabric; uint64_t node; uint16_t ep; } dev_toggle_t;
static dev_toggle_t s_dev_toggle_q[8];
static int          s_dev_toggle_n = 0;

/* Climate page — temperature + humidity from a bound Matter sensor. All values
 * below are protected by s_lvgl_mtx. The "have" flags stay false until the
 * first attribute report arrives, so the page shows "--" instead of 0. */
static lv_obj_t *s_temp_label     = NULL;
static lv_obj_t *s_humidity_label = NULL;
static float     s_temp_c         = 0.0f;
static float     s_humidity_pct   = 0.0f;
static bool      s_have_temp      = false;
static bool      s_have_humidity  = false;

/* Air-quality page — overall rating + CO2 + PM2.5 from a bound Matter air-quality monitor.
 * All values below are protected by s_lvgl_mtx; "have" flags gate "--" placeholders. */
static lv_obj_t *s_aq_label   = NULL;
static lv_obj_t *s_aq_card    = NULL;   /* air-quality card (bg recoloured by rating) */
static lv_obj_t *s_co2_label  = NULL;
static lv_obj_t *s_pm25_label = NULL;
static int       s_aq_level   = -1;        /* -1 = none yet; else Matter AirQualityEnum 0..6 */
static float     s_co2_ppm    = 0.0f;
static float     s_pm25_ugm3  = 0.0f;
static bool      s_have_co2   = false;
static bool      s_have_pm25  = false;

/* Weather page — forecast min/max/current temps + text condition + location, from the bound
 * weather aggregator device. All protected by s_lvgl_mtx. */
static lv_obj_t *s_wx_location_label  = NULL;
static lv_obj_t *s_wx_condition_label = NULL;
static lv_obj_t *s_wx_icon            = NULL;   /* weather condition glyph (lv_img) */
static lv_obj_t *s_wx_min_label       = NULL;
static lv_obj_t *s_wx_max_label       = NULL;
static lv_obj_t *s_wx_now_label       = NULL;
static float     s_wx_min_c    = 0.0f;
static float     s_wx_max_c    = 0.0f;
static float     s_wx_now_c    = 0.0f;
static bool      s_have_wx_min = false;
static bool      s_have_wx_max = false;
static bool      s_have_wx_now = false;
static char      s_wx_condition[40] = {0};
static char      s_wx_location[24]  = {0};

/* Alert overlay: a modal banner on the top layer (lv_layer_top) that floats above whichever page
 * or screen is currently showing — used for the contact-open and water-leak alerts. It auto-hides
 * after ALERT_VISIBLE_MS, revealing the page underneath (which is never changed, so there is no
 * "previous page" to restore). All widgets protected by s_lvgl_mtx. */
#define ALERT_VISIBLE_MS 8000
static lv_obj_t   *s_alert_overlay = NULL;   /* black backdrop on the top layer */
static lv_obj_t   *s_alert_card    = NULL;   /* the coloured alert card (recoloured per alert) */
static lv_obj_t   *s_alert_icon    = NULL;
static lv_obj_t   *s_alert_title   = NULL;
static lv_obj_t   *s_alert_sub     = NULL;
static lv_timer_t *s_alert_timer   = NULL;
/* Read by the LVGL task's power state machine to keep the panel awake while an alert shows. */
static volatile bool s_alert_visible = false;

/* Settings page (swipe up from Home) — display brightness + idle-off sliders and the two action
 * buttons. All widgets protected by s_lvgl_mtx. */
static lv_obj_t *s_set_bright_slider  = NULL;
static lv_obj_t *s_set_bright_label   = NULL;
static lv_obj_t *s_set_timeout_label  = NULL;   /* "N S" value inside the −/+ stepper */

/* Reusable confirmation overlay (top layer, above every page) for the Reboot / Factory-reset
 * buttons. s_confirm_action records which action the user is confirming. */
enum { CONFIRM_NONE = 0, CONFIRM_REBOOT, CONFIRM_FACTORY };
static int         s_confirm_action  = CONFIRM_NONE;
static lv_obj_t   *s_confirm_overlay = NULL;
static lv_obj_t   *s_confirm_card    = NULL;
static lv_obj_t   *s_confirm_icon    = NULL;
static lv_obj_t   *s_confirm_title   = NULL;
static lv_obj_t   *s_confirm_msg     = NULL;
static lv_obj_t   *s_confirm_ok_lbl  = NULL;

/* Terminal actions, latched by the LVGL task and performed outside the LVGL lock (they don't
 * return / restart the device). Set from the confirm overlay's OK button. */
static bool s_reboot_pending  = false;
static bool s_factory_pending = false;

/* Deferred Matter commands, sent by the LVGL task outside the lock. These are
 * only touched by the LVGL task (event callbacks run inside lv_timer_handler). */
static bool s_onoff_pending  = false;   /* master toggle tapped: send an explicit On/Off... */
static bool s_onoff_on       = false;   /* ...to this target state (not a blind Toggle) */

/* Slider command debounce. A drag updates the label/preview live but sends only one Matter command
 * once the value has been still for SLIDER_DEBOUNCE_MS, coalescing the drag stream into a single
 * command. Each var holds the esp_timer ms of the last change (0 = nothing pending); the LVGL task
 * sends and clears it once the debounce elapses. */
#define SLIDER_DEBOUNCE_MS 200
static int64_t s_level_dirty_ms = 0;
static int64_t s_ct_dirty_ms    = 0;
static int64_t s_hue_dirty_ms   = 0;

/* Top-level screens (children of the active screen). Exactly one is shown at a
 * time per s_mode. Swiping is only possible in READY mode (the others are not
 * the tileview). */
static lv_obj_t       *s_boot_scr   = NULL;   /* neutral startup "loading" screen */
static lv_obj_t       *s_pair_scr   = NULL;   /* QR code (pairing) */
static lv_obj_t       *s_nobind_scr = NULL;   /* "bind a light" notice */
static lv_obj_t       *s_tileview   = NULL;   /* controls (bulb / brightness / CT) */
static lv_obj_t       *s_home_tile  = NULL;   /* the Home page inside the tileview */
static light_ui_mode_t s_mode       = LIGHT_UI_MODE_BOOTING;

/* Boot screen dismissal. The UI opens on the BOOTING screen so the Matter QR never flashes on a
 * commissioned device, and leaves for the settled screen (requested via light_ui_set_mode) once the
 * startup CASE/Thread storm has passed — detected by the CPU going idle, with a floor, a
 * sustained-idle requirement and a hard cap so a stuck device is never trapped there. */
#define BOOT_MIN_MS         1500   /* earliest we may leave the boot screen */
#define BOOT_MAX_MS        25000   /* hard cap (backstop only), set well above the real storm so the
                                    * CPU-idle detector normally dismisses the screen first. */
#define BOOT_IDLE_SAMPLE_MS  100   /* how often we sample the idle counter */
#define BOOT_QUIET_SAMPLES     4   /* consecutive idle samples (~400 ms) = storm over */
#define BOOT_IDLE_FRAC_PCT    70   /* "idle" = idle rate >= this % of its observed peak */
static volatile light_ui_mode_t s_boot_target       = LIGHT_UI_MODE_READY;
static volatile bool            s_boot_target_valid  = false;
/* Bumped by a FreeRTOS idle hook; its rate (samples/window) is our direct CPU-idle measure. */
static volatile uint32_t        s_idle_ticks         = 0;

static char s_qr_payload[96]  = {0};
static char s_manual_code[32] = {0};

/* Display power management: after this much inactivity the screen sleeps (cuts the AMOLED power
 * rail via amoled_sleep()); a touch wakes it, but that wake-tap is swallowed, so every sleep costs
 * the user an extra tap. This is a mains-powered device, so the default is long — enough to guard
 * against burn-in over real idle, not to save power. Adjustable on the Settings page and loaded
 * from NVS at boot. */
#define LIGHT_UI_IDLE_TIMEOUT_DEFAULT_MS  120000
#define LIGHT_UI_IDLE_TIMEOUT_MIN_S        10
#define LIGHT_UI_IDLE_TIMEOUT_MAX_S       600
static int64_t s_idle_timeout_ms = LIGHT_UI_IDLE_TIMEOUT_DEFAULT_MS;   /* protected by s_lvgl_mtx / LVGL task */

/* Display (AMOLED) backlight brightness as a user-facing percentage (5-100) — separate from the
 * bound light's brightness (s_pct). Set on the Settings page, applied live via
 * amoled_set_brightness(), and persisted in NVS. */
#define DISP_BRIGHT_DEFAULT_PCT  70
#define DISP_BRIGHT_MIN_PCT       5
static int s_disp_bright_pct = DISP_BRIGHT_DEFAULT_PCT;   /* protected by s_lvgl_mtx / LVGL task */

/* Map the display-brightness percentage to the SH8601 0-255 register level, with a floor so the
 * lowest setting still shows a dim (not black) panel. */
static inline uint8_t disp_level_from_pct(int pct)
{
    if (pct < DISP_BRIGHT_MIN_PCT) pct = DISP_BRIGHT_MIN_PCT;
    if (pct > 100) pct = 100;
    int lvl = pct * 255 / 100;
    if (lvl < 13) lvl = 13;   /* ~5% floor: never fully off */
    return (uint8_t)lvl;
}

/* Time is stashed here just before a UI-initiated reboot and restored at boot, so the wall clock
 * survives the restart. RTC_NOINIT memory survives a soft reset (esp_restart) but is garbage on a
 * cold power-on — the magic guards that case, where we fall back to the neutral default clock. */
#define RTC_TIME_MAGIC 0x54494D45u   /* "TIME" */
static RTC_NOINIT_ATTR uint32_t s_rtc_time_magic;
static RTC_NOINIT_ATTR time_t   s_rtc_time_utc;

static bool    s_awake             = true;
static int64_t s_last_active_ms    = 0;
static bool    s_consume_touch     = false;  /* swallow the wake touch until released */
static bool    s_force_wake        = false;  /* request a wake (e.g. to show the QR)  */
static bool    s_need_redraw       = false;  /* GRAM lost on wake; force a full redraw */
static bool    s_power_save_enabled = false; /* gated: only on once fully commissioned */
static int64_t s_last_icd_ms        = 0;     /* last ICD active-mode nudge (ICD build only)  */

#define LVGL_LOCK()   xSemaphoreTakeRecursive(s_lvgl_mtx, portMAX_DELAY)
#define LVGL_UNLOCK() xSemaphoreGiveRecursive(s_lvgl_mtx)

/* Persist the clock's UTC offset in NVS. The Matter controller only pushes the timezone at
 * commissioning (not on every boot), and the SDK doesn't notify us of its reloaded value, so we
 * mirror it here: saved when the controller sends a zone, loaded on boot so the clock is right
 * immediately after a reboot instead of falling back to UTC. */
#define TZ_NVS_NS  "lightui"
#define TZ_NVS_KEY "utc_off"

static void tz_offset_save(int32_t off)
{
    nvs_handle_t h;
    if (nvs_open(TZ_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, TZ_NVS_KEY, off);
        nvs_commit(h);
        nvs_close(h);
    }
}

static bool tz_offset_load(int32_t *out)
{
    nvs_handle_t h;
    bool found = false;
    if (nvs_open(TZ_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        found = (nvs_get_i32(h, TZ_NVS_KEY, out) == ESP_OK);
        nvs_close(h);
    }
    return found;
}

/* Persist the "sync time from controller" preference (the Time & Date page toggle). Written only
 * when the user flips the switch — a rare user action, so no flash-wear concern. Reused NVS ns. */
#define AUTO_NVS_KEY "t_auto"

static void time_auto_save(bool on)
{
    nvs_handle_t h;
    if (nvs_open(TZ_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, AUTO_NVS_KEY, on ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

static bool time_auto_load(bool *out)
{
    nvs_handle_t h;
    bool found = false;
    if (nvs_open(TZ_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        int32_t v = 1;
        if (nvs_get_i32(h, AUTO_NVS_KEY, &v) == ESP_OK) { *out = (v != 0); found = true; }
        nvs_close(h);
    }
    return found;
}

/* Persist the two Settings-page display preferences (backlight brightness %, idle-off seconds).
 * Written only when the user releases the corresponding slider — rare, so no flash-wear concern.
 * Same NVS namespace as the clock prefs. */
#define BRIGHT_NVS_KEY "disp_br"
#define IDLE_NVS_KEY   "idle_s"

static void disp_bright_save(int pct)
{
    nvs_handle_t h;
    if (nvs_open(TZ_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, BRIGHT_NVS_KEY, pct);
        nvs_commit(h);
        nvs_close(h);
    }
}

static bool disp_bright_load(int *out)
{
    nvs_handle_t h;
    bool found = false;
    if (nvs_open(TZ_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        int32_t v = 0;
        if (nvs_get_i32(h, BRIGHT_NVS_KEY, &v) == ESP_OK) { *out = (int)v; found = true; }
        nvs_close(h);
    }
    return found;
}

static void idle_timeout_save(int secs)
{
    nvs_handle_t h;
    if (nvs_open(TZ_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, IDLE_NVS_KEY, secs);
        nvs_commit(h);
        nvs_close(h);
    }
}

static bool idle_timeout_load(int *out)
{
    nvs_handle_t h;
    bool found = false;
    if (nvs_open(TZ_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        int32_t v = 0;
        if (nvs_get_i32(h, IDLE_NVS_KEY, &v) == ESP_OK) { *out = (int)v; found = true; }
        nvs_close(h);
    }
    return found;
}

/* Proleptic-Gregorian date/time → Unix UTC seconds (Howard Hinnant's days_from_civil). Used to
 * turn the manual-entry rollers into an epoch; the inverse of the gmtime_r() the clock display uses,
 * with no dependency on timegm() or the process timezone. mon is 1-12, day 1-31. */
static time_t utc_from_ymd_hms(int year, int mon, int day, int h, int m, int s)
{
    year -= (mon <= 2);
    int era = (year >= 0 ? year : year - 399) / 400;
    unsigned yoe = (unsigned)(year - era * 400);
    unsigned doy = (153 * (unsigned)(mon + (mon > 2 ? -3 : 9)) + 2) / 5 + (unsigned)day - 1;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    long days = (long)era * 146097 + (long)doe - 719468;
    return (time_t)days * 86400 + (time_t)h * 3600 + (time_t)m * 60 + s;
}

/* ── Brightness <-> Matter level (1-254) mapping ─────────────── */
static inline uint8_t pct_to_level(int pct)
{
    if (pct < 1) pct = 1;
    if (pct > 100) pct = 100;
    int level = 1 + (pct * 253) / 100;      /* 1% -> 3, 100% -> 254 */
    if (level < 1) level = 1;
    if (level > 254) level = 254;
    return (uint8_t)level;
}

static inline int level_to_pct(uint8_t level)
{
    if (level < 1) level = 1;
    int pct = ((level - 1) * 100 + 126) / 253;   /* rounded */
    if (pct < 1) pct = 1;
    if (pct > 100) pct = 100;
    return pct;
}

/* ── Color temperature <-> Matter mireds mapping ─────────────── */
static inline int clamp_kelvin(int k)
{
    if (k < CT_KELVIN_MIN) k = CT_KELVIN_MIN;
    if (k > CT_KELVIN_MAX) k = CT_KELVIN_MAX;
    return k;
}

static inline uint16_t kelvin_to_mireds(int kelvin)
{
    kelvin = clamp_kelvin(kelvin);
    return (uint16_t)(1000000 / kelvin);
}

static inline int mireds_to_kelvin(uint16_t mireds)
{
    if (mireds == 0) mireds = 1;
    return clamp_kelvin(1000000 / mireds);
}

/* ── Neo-brutalist palette + bold display fonts ─────────────── */
/* Muted so the vibrant AMOLED doesn't over-saturate them (matches the mockup, not neon). */
#define COL_VOID   0x000000
#define COL_LIME   0xA6CC52
#define COL_AMBER  0xDCA24C
#define COL_SKY    0x6E9AD2
#define COL_VIOLET 0x998EC6
#define COL_CORAL  0xD67C5D
#define COL_BONE   0xE4E1D5
#define COL_INK    0x0B0B0B
#define COL_DIM    0x1C1C1C   /* light-toggle card when OFF */
#define COL_ALERT  0xC94A4A   /* critical alert card (water leak) — a clear red in the palette family */

LV_FONT_DECLARE(brutal_44);   /* heavy numerals (Home)     */
LV_FONT_DECLARE(brutal_42);   /* stat values (Indoor)      */
LV_FONT_DECLARE(brutal_30);   /* headings/values           */
#define FONT_LABEL &lv_font_montserrat_14

/* Scale an RGB colour's brightness to `pct` percent (for deriving a darker "extrude" shade). */
static uint32_t darken(uint32_t rgb, int pct)
{
    uint32_t r = ((rgb >> 16) & 0xFF) * pct / 100;
    uint32_t g = ((rgb >> 8)  & 0xFF) * pct / 100;
    uint32_t b = ( rgb        & 0xFF) * pct / 100;
    return (r << 16) | (g << 8) | b;
}

/* A neo-brutalist "sticker" card. LVGL's style shadow is blurred, which reads as a flat glow on the
 * true-black OLED — so we fake a HARD offset shadow: a transparent wrapper (the flex item) holds a
 * solid rectangle shifted CARD_SH px down-right, with the real card on top. The block is a DARKER
 * shade of the card's own colour, so it reads as the card's extruded side (3-D) without the harsh
 * white-on-black contrast. pad_all 12; add children to the returned card. */
#define CARD_SH 6
static lv_obj_t *make_card(lv_obj_t *parent, uint32_t bg, lv_coord_t w, lv_coord_t h)
{
    lv_obj_t *wrap = lv_obj_create(parent);
    lv_obj_remove_style_all(wrap);
    lv_obj_set_size(wrap, w, h);
    lv_obj_clear_flag(wrap, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(wrap, LV_OBJ_FLAG_OVERFLOW_VISIBLE);   /* let the shadow peek out of the wrapper */

    lv_obj_t *sh = lv_obj_create(wrap);
    lv_obj_remove_style_all(sh);
    lv_obj_set_size(sh, LV_PCT(100), LV_PCT(100));
    lv_obj_set_pos(sh, CARD_SH, CARD_SH);
    lv_obj_set_style_bg_opa(sh, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(sh, lv_color_hex(darken(bg, 58)), 0);
    lv_obj_set_style_radius(sh, 18, 0);
    lv_obj_clear_flag(sh, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *c = lv_obj_create(wrap);
    lv_obj_remove_style_all(c);
    lv_obj_set_size(c, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(c, lv_color_hex(bg), 0);
    lv_obj_set_style_radius(c, 18, 0);
    lv_obj_set_style_border_width(c, 2, 0);                 /* crisp dark edge on the top face */
    lv_obj_set_style_border_color(c, lv_color_hex(COL_INK), 0);
    lv_obj_set_style_pad_all(c, 12, 0);
    lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
    return c;
}

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, uint32_t color, const char *txt)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    if (txt) lv_label_set_text(l, txt);
    return l;
}

/* Uppercase, letter-spaced micro-label. */
static lv_obj_t *make_caplabel(lv_obj_t *parent, uint32_t color, const char *txt)
{
    lv_obj_t *l = make_label(parent, FONT_LABEL, color, txt);
    lv_obj_set_style_text_letter_space(l, 2, 0);
    return l;
}

/* Circular ink icon badge. Returns the disc; caller adds a glyph label centered in it. */
static lv_obj_t *make_badge(lv_obj_t *parent, lv_coord_t d, uint32_t bg)
{
    lv_obj_t *b = lv_obj_create(parent);
    lv_obj_remove_style_all(b);
    lv_obj_set_size(b, d, d);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(bg), 0);
    lv_obj_set_style_radius(b, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
    return b;
}

/* Add a recoloured icon centred in a badge (scaled down for padding). */
static void badge_icon(lv_obj_t *badge, const lv_img_dsc_t *icon, uint32_t tint)
{
    lv_obj_t *ic = lv_img_create(badge);
    lv_img_set_src(ic, icon);
    lv_obj_center(ic);   /* 40 px icon; padding comes from a larger badge */
    lv_obj_set_style_img_recolor(ic, lv_color_hex(tint), 0);
    lv_obj_set_style_img_recolor_opa(ic, LV_OPA_COVER, 0);
}

/* Recolour a card AND its hard-shadow sibling (shadow = darker shade), for cards whose colour is
 * dynamic (e.g. the air-quality card by rating). Caller holds s_lvgl_mtx. */
static void card_set_color(lv_obj_t *card, uint32_t bg)
{
    lv_obj_set_style_bg_color(card, lv_color_hex(bg), 0);
    lv_obj_t *wrap = lv_obj_get_parent(card);
    if (wrap) {
        lv_obj_t *sh = lv_obj_get_child(wrap, 0);   /* the shadow is the wrapper's first child */
        if (sh && sh != card) lv_obj_set_style_bg_color(sh, lv_color_hex(darken(bg, 58)), 0);
    }
}

/* ── Home: light-toggle card ────────────────────────────────── */

/* Knob slide animation. Track is 66 wide, knob 26, 5 px inset: OFF x=5, ON x=35. */
#define KNOB_X_OFF 5
#define KNOB_X_ON  35
static void knob_anim_x_cb(void *obj, int32_t v) { lv_obj_set_x((lv_obj_t *)obj, v); }

static void animate_knob_locked(bool on)
{
    if (!s_light_knob) return;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_light_knob);
    lv_anim_set_exec_cb(&a, knob_anim_x_cb);
    lv_anim_set_values(&a, lv_obj_get_x(s_light_knob), on ? KNOB_X_ON : KNOB_X_OFF);
    lv_anim_set_time(&a, 220);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}

/* Repaint the light-toggle card for s_state. The card is ALWAYS green; on/off is shown by the
 * "ON"/"OFF" label and the knob position (which slides). Caller must hold s_lvgl_mtx. */
static void apply_state_locked(void)
{
    if (!s_light_card) {
        return;
    }
    lv_obj_set_style_bg_color(s_light_card, lv_color_hex(COL_LIME), 0);
    if (s_light_state) {
        lv_label_set_text(s_light_state, s_state ? "ON" : "OFF");
        lv_obj_set_style_text_color(s_light_state, lv_color_hex(COL_INK), 0);
    }
    if (s_light_cap) {
        lv_obj_set_style_text_color(s_light_cap, lv_color_hex(COL_INK), 0);
    }
    if (s_light_icon) {
        lv_obj_set_style_img_recolor(s_light_icon, lv_color_hex(COL_LIME), 0);  /* lime on ink badge */
        lv_obj_set_style_img_recolor_opa(s_light_icon, LV_OPA_COVER, 0);
    }
    if (s_light_knob) {
        lv_obj_set_style_bg_color(s_light_knob, lv_color_hex(COL_LIME), 0);
        animate_knob_locked(s_state);
    }
}

/* Recompute the master toggle from the per-device states: ON if ANY bound light/plug is on, OFF
 * only when ALL are off. Drives the Home toggle so external changes (e.g. from a phone) move it.
 * Caller holds s_lvgl_mtx. */
static bool ui_has_any_device(void)
{
    for (auto &d : s_ui_devices) if (d.valid && d.have_state) return true;
    return false;
}

static void recompute_master_locked(void)
{
    bool any_on = false;
    for (auto &d : s_ui_devices) if (d.valid && d.have_state && d.on) { any_on = true; break; }
    s_state = any_on;
    apply_state_locked();
}

/* Runs inside lv_timer_handler(), i.e. on the LVGL task with s_lvgl_mtx held. */
static void bulb_event_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    /* Move the toggle to the opposite of the current aggregate: if anything is on, this turns
     * everything OFF; if all are off, this turns everything ON. Issue an explicit On/Off (not a
     * blind Toggle) so the knob direction always maps to a definite command. */
    bool target = !s_state;
    s_state = target;            /* optimistic; device reports will correct the aggregate */
    apply_state_locked();
    s_onoff_pending = true;      /* deferred: sent by the LVGL task outside the lock */
    s_onoff_on = target;
    ESP_LOGI(TAG, "Master toggle -> %s (%s queued)", target ? "ON" : "OFF", target ? "On" : "Off");
}

/* ── Brightness page ────────────────────────────────────────── */

/* Update the percentage label from s_pct. Caller must hold s_lvgl_mtx. */
static void apply_pct_label_locked(void)
{
    if (s_pct_label) {
        lv_label_set_text_fmt(s_pct_label, "%d%%", s_pct);
    }
}

/* Slider events run inside lv_timer_handler(), on the LVGL task. */
static void slider_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);

    switch (code) {
    case LV_EVENT_PRESSED:
        s_dragging = true;
        break;

    case LV_EVENT_VALUE_CHANGED:
        s_pct = lv_slider_get_value(s_slider);   /* live update of label while dragging */
        apply_pct_label_locked();
        s_level_dirty_ms = esp_timer_get_time() / 1000;   /* (re)start the debounce */
        break;

    case LV_EVENT_RELEASED:
        s_dragging = false;
        s_level_dirty_ms = esp_timer_get_time() / 1000;   /* send once the final value settles */
        break;

    default:
        break;
    }
}

/* ── UI construction ────────────────────────────────────────── */

/* ── Controls: warmth (colour temperature) + colour (hue) ───── */

static inline int snap_kelvin(int k)
{
    k = ((k + CT_KELVIN_STEP / 2) / CT_KELVIN_STEP) * CT_KELVIN_STEP;
    return clamp_kelvin(k);
}

/* Update the Kelvin value label from s_ct_kelvin. Caller holds s_lvgl_mtx. */
static void apply_ct_locked(void)
{
    if (s_ct_label) {
        lv_label_set_text_fmt(s_ct_label, "%dK", s_ct_kelvin);
    }
}

static void ct_slider_event_cb(lv_event_t *e)
{
    switch (lv_event_get_code(e)) {
    case LV_EVENT_PRESSED:
        s_ct_dragging = true;
        break;
    case LV_EVENT_VALUE_CHANGED:
        s_ct_kelvin = snap_kelvin(lv_slider_get_value(s_ct_slider));
        apply_ct_locked();
        s_ct_dirty_ms = esp_timer_get_time() / 1000;   /* (re)start the debounce */
        break;
    case LV_EVENT_RELEASED:
        s_ct_dragging = false;
        s_ct_kelvin = snap_kelvin(s_ct_kelvin);
        lv_slider_set_value(s_ct_slider, s_ct_kelvin, LV_ANIM_OFF);
        s_ct_dirty_ms = esp_timer_get_time() / 1000;   /* send once the final value settles */
        break;
    default:
        break;
    }
}

static lv_color_t hue_to_color(int deg)
{
    return lv_color_hsv_to_rgb((uint16_t)(deg % 360), 100, 100);
}

static const char *hue_name(int deg)
{
    static const struct { int max; const char *name; } k[] = {
        {15, "RED"}, {45, "ORANGE"}, {70, "YELLOW"}, {160, "GREEN"},
        {200, "CYAN"}, {255, "BLUE"}, {290, "VIOLET"}, {330, "MAGENTA"}, {361, "RED"} };
    for (auto &e : k) { if (deg < e.max) return e.name; }
    return "RED";
}

/* Update the hue swatch + colour name from s_hue_deg. Caller holds s_lvgl_mtx. */
static void apply_hue_locked(void)
{
    if (s_hue_swatch) lv_obj_set_style_bg_color(s_hue_swatch, hue_to_color(s_hue_deg), 0);
    if (s_hue_name)   lv_label_set_text(s_hue_name, hue_name(s_hue_deg));
}

static void hue_slider_event_cb(lv_event_t *e)
{
    switch (lv_event_get_code(e)) {
    case LV_EVENT_PRESSED:
        s_hue_dragging = true;
        break;
    case LV_EVENT_VALUE_CHANGED:
        s_hue_deg = lv_slider_get_value(s_hue_slider);
        apply_hue_locked();
        s_hue_dirty_ms = esp_timer_get_time() / 1000;   /* (re)start the debounce */
        break;
    case LV_EVENT_RELEASED:
        s_hue_dragging = false;
        s_hue_dirty_ms = esp_timer_get_time() / 1000;   /* send once the final value settles */
        break;
    default:
        break;
    }
}

/* Show the Warmth card or the Colour card per s_color_mode, and restyle the segmented toggle. */
static void apply_color_mode_locked(void)
{
    bool warmth = (s_color_mode == COLOR_MODE_WARMTH);
    /* Hide the WRAPPER (the flex item), not the inner card, or its slot + shadow still show. */
    lv_obj_t *ctw  = s_ct_card  ? lv_obj_get_parent(s_ct_card)  : NULL;
    lv_obj_t *huew = s_hue_card ? lv_obj_get_parent(s_hue_card) : NULL;
    if (ctw) {
        if (warmth) lv_obj_clear_flag(ctw, LV_OBJ_FLAG_HIDDEN);
        else        lv_obj_add_flag(ctw, LV_OBJ_FLAG_HIDDEN);
    }
    if (huew) {
        if (!warmth) lv_obj_clear_flag(huew, LV_OBJ_FLAG_HIDDEN);
        else         lv_obj_add_flag(huew, LV_OBJ_FLAG_HIDDEN);
    }
    /* The toggle text is a child label with its own colour, so recolour the label, not the button. */
    if (s_seg_warmth) {
        lv_obj_set_style_bg_color(s_seg_warmth, lv_color_hex(warmth ? COL_BONE : COL_VOID), 0);
        lv_obj_t *l = lv_obj_get_child(s_seg_warmth, 0);
        if (l) lv_obj_set_style_text_color(l, lv_color_hex(warmth ? COL_INK : COL_BONE), 0);
    }
    if (s_seg_colour) {
        lv_obj_set_style_bg_color(s_seg_colour, lv_color_hex(!warmth ? COL_BONE : COL_VOID), 0);
        lv_obj_t *l = lv_obj_get_child(s_seg_colour, 0);
        if (l) lv_obj_set_style_text_color(l, lv_color_hex(!warmth ? COL_INK : COL_BONE), 0);
    }
}

/* Switching mode adopts that mode's slider value on the light immediately: warmth → send the current
 * colour-temperature, colour → send the current hue. Backdate the debounce stamp by SLIDER_DEBOUNCE_MS
 * so the LVGL task sends it on its next pass (a deliberate tap should apply at once, not after a
 * settle delay). */
static void seg_warmth_event_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    s_color_mode = COLOR_MODE_WARMTH;
    apply_color_mode_locked();
    s_ct_dirty_ms = esp_timer_get_time() / 1000 - SLIDER_DEBOUNCE_MS;
}
static void seg_colour_event_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    s_color_mode = COLOR_MODE_COLOUR;
    apply_color_mode_locked();
    s_hue_dirty_ms = esp_timer_get_time() / 1000 - SLIDER_DEBOUNCE_MS;
}

/* ── Climate (temperature + humidity) page ──────────────────── */

/* Refresh the temp/humidity labels from the current readings. Caller holds
 * s_lvgl_mtx. Shows "--" for a reading that has not been received yet. */
static void apply_climate_locked(void)
{
    /* Format with the standard C snprintf, not lv_label_set_text_fmt: LVGL's built-in
     * printf does not support %f unless LV_SPRINTF_USE_FLOAT is enabled, so "%.1f" would
     * render as a literal "f". */
    char buf[16];
    if (s_temp_label) {
        if (s_have_temp) {
            snprintf(buf, sizeof(buf), "%.1f°", s_temp_c);
            lv_label_set_text(s_temp_label, buf);
        } else {
            lv_label_set_text(s_temp_label, "--°");
        }
    }
    if (s_humidity_label) {
        if (s_have_humidity) {
            snprintf(buf, sizeof(buf), "%.0f%%", s_humidity_pct);
            lv_label_set_text(s_humidity_label, buf);
        } else {
            lv_label_set_text(s_humidity_label, "--%");
        }
    }
}

/* ── Air-quality (rating + CO2 + PM2.5) page ────────────────── */

/* Matter AirQualityEnum → display text + accent color. Caller holds s_lvgl_mtx. */
static const char *aq_level_text(int level, uint32_t *rgb_out)
{
    struct { const char *text; uint32_t rgb; } kAq[] = {
        { "UNKNOWN",        0x808080 },  /* 0 */
        { "GOOD",           COL_LIME  },  /* 1 */
        { "FAIR",           0xBFC24E  },  /* 2 */
        { "MODERATE",       COL_AMBER },  /* 3 */
        { "POOR",           COL_CORAL },  /* 4 */
        { "VERY POOR",      0xCB5B5B  },  /* 5 */
        { "EXTREMELY POOR", 0xB06BC0  },  /* 6 */
    };
    if (level < 0 || level > 6) {
        if (rgb_out) *rgb_out = 0x808080;
        return "--";
    }
    if (rgb_out) *rgb_out = kAq[level].rgb;
    return kAq[level].text;
}

/* Refresh the air-quality labels from the current readings. Caller holds s_lvgl_mtx. */
static void apply_airquality_locked(void)
{
    char buf[16];
    if (s_aq_label) {
        uint32_t rgb;
        const char *text = aq_level_text(s_aq_level, &rgb);
        lv_label_set_text(s_aq_label, text);
        if (s_aq_card) card_set_color(s_aq_card, rgb);   /* green when good, warmer/red when poor */
    }
    if (s_co2_label) {
        if (s_have_co2) { snprintf(buf, sizeof(buf), "%.0f", s_co2_ppm); lv_label_set_text(s_co2_label, buf); }
        else lv_label_set_text(s_co2_label, "--");
    }
    if (s_pm25_label) {
        if (s_have_pm25) { snprintf(buf, sizeof(buf), "%.0f", s_pm25_ugm3); lv_label_set_text(s_pm25_label, buf); }
        else lv_label_set_text(s_pm25_label, "--");
    }
}

/* ── Weather (forecast min/max/current + condition) page ────── */

/* One temp value cell: writes "18.7°" or "--°" into a label. Caller holds s_lvgl_mtx. */
static void apply_wx_temp_locked(lv_obj_t *label, bool have, float celsius)
{
    if (!label) {
        return;
    }
    char buf[16];
    if (have) {
        snprintf(buf, sizeof(buf), "%.0f°", celsius);
    } else {
        snprintf(buf, sizeof(buf), "--°");
    }
    lv_label_set_text(label, buf);
}

/* Map a free-text weather condition to one of the generated glyphs (keyword match, day/night
 * aware), with a cloud fallback for anything unrecognised. */
static const lv_img_dsc_t *weather_icon_for(const char *cond)
{
    if (!cond || !cond[0]) return &ic_cloudy;
    char b[40];
    size_t i = 0;
    for (; cond[i] && i < sizeof(b) - 1; i++) b[i] = (char)tolower((unsigned char)cond[i]);
    b[i] = '\0';
    #define HAS(k) (strstr(b, k) != NULL)
    if (HAS("night"))                                   return &ic_clearnight;
    if (HAS("thunder") || HAS("lightning") || HAS("storm")) return &ic_storm;
    if (HAS("hail"))                                    return &ic_hail;
    if (HAS("pour") || HAS("heavy rain"))               return &ic_pouring;
    if (HAS("shower") || HAS("rain") || HAS("drizzle")) return &ic_rainy;
    if (HAS("snow") || HAS("sleet") || HAS("flurr"))    return &ic_snowy;
    if (HAS("fog") || HAS("mist") || HAS("haze") || HAS("smoke")) return &ic_fog;
    if (HAS("wind") || HAS("breez"))                    return &ic_windy;
    if (HAS("part"))                                    return &ic_partly;
    if (HAS("cloud") || HAS("overcast"))                return &ic_cloudy;
    if (HAS("clear") || HAS("sun") || HAS("fair"))      return &ic_sunny;
    #undef HAS
    return &ic_cloudy;
}

/* Set a label to the UPPERCASE of src (or fallback if empty). Caller holds s_lvgl_mtx. */
static void set_upper_locked(lv_obj_t *l, const char *src, const char *fallback)
{
    if (!l) return;
    const char *s = (src && src[0]) ? src : fallback;
    char b[40];
    size_t i = 0;
    for (; s[i] && i < sizeof(b) - 1; i++) b[i] = (char)toupper((unsigned char)s[i]);
    b[i] = '\0';
    lv_label_set_text(l, b);
}

/* Refresh the weather block from the current readings. Caller holds s_lvgl_mtx. */
static void apply_weather_locked(void)
{
    set_upper_locked(s_wx_location_label, s_wx_location, "WEATHER");
    set_upper_locked(s_wx_condition_label, s_wx_condition, "--");
    if (s_wx_icon) {
        lv_img_set_src(s_wx_icon, weather_icon_for(s_wx_condition));
    }
    apply_wx_temp_locked(s_wx_min_label, s_have_wx_min, s_wx_min_c);
    apply_wx_temp_locked(s_wx_max_label, s_have_wx_max, s_wx_max_c);
    apply_wx_temp_locked(s_wx_now_label, s_have_wx_now, s_wx_now_c);
}

/* ── Pairing (QR) and no-binding screens ────────────────────── */

static lv_obj_t *make_fullscreen_locked(lv_obj_t *scr)
{
    lv_obj_t *o = lv_obj_create(scr);
    lv_obj_set_size(o, LV_PCT(100), LV_PCT(100));
    lv_obj_center(o);
    lv_obj_set_style_bg_color(o, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_border_width(o, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(o, 0, LV_PART_MAIN);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    return o;
}

/* Neutral startup screen shown while the device settles (avoids the Matter QR flashing on a
 * commissioned boot). Deliberately static — an animation would stutter under the boot CPU load. */
static void build_boot_screen_locked(lv_obj_t *scr)
{
    s_boot_scr = make_fullscreen_locked(scr);

    lv_obj_t *icon = lv_label_create(s_boot_scr);
    lv_label_set_text(icon, LV_SYMBOL_HOME);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_set_style_text_color(icon, lv_color_hex(0x686868), LV_PART_MAIN);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -22);

    lv_obj_t *msg = lv_label_create(s_boot_scr);
    lv_label_set_text(msg, "Starting");
    lv_obj_set_style_text_font(msg, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(msg, lv_color_hex(0x585858), LV_PART_MAIN);
    lv_obj_align(msg, LV_ALIGN_CENTER, 0, 34);
}

static void build_pairing_screen_locked(lv_obj_t *scr)
{
    s_pair_scr = make_fullscreen_locked(scr);

    lv_obj_t *title = lv_label_create(s_pair_scr);
    lv_label_set_text(title, "Scan to commission");
    lv_obj_set_style_text_color(title, lv_color_hex(0xD0D0D0), LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 16);

    /* White quiet-zone backdrop so the code is scannable */
    lv_obj_t *qbg = lv_obj_create(s_pair_scr);
    lv_obj_set_size(qbg, 300, 300);
    lv_obj_align(qbg, LV_ALIGN_CENTER, 0, -10);
    lv_obj_set_style_bg_color(qbg, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_border_width(qbg, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(qbg, 8, LV_PART_MAIN);
    lv_obj_clear_flag(qbg, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *qr = lv_qrcode_create(qbg, 268, lv_color_hex(0x000000), lv_color_hex(0xFFFFFF));
    lv_obj_center(qr);
    if (s_qr_payload[0]) {
        lv_qrcode_update(qr, s_qr_payload, strlen(s_qr_payload));
    }

    lv_obj_t *code = lv_label_create(s_pair_scr);
    if (s_manual_code[0]) {
        lv_label_set_text_fmt(code, "Manual code: %s", s_manual_code);
    } else {
        lv_label_set_text(code, "Manual code unavailable");
    }
    lv_obj_set_style_text_color(code, lv_color_hex(0xB0B0B0), LV_PART_MAIN);
    lv_obj_align(code, LV_ALIGN_BOTTOM_MID, 0, -24);
}

static void build_nobind_screen_locked(lv_obj_t *scr)
{
    s_nobind_scr = make_fullscreen_locked(scr);

    lv_obj_t *icon = lv_label_create(s_nobind_scr);
    lv_label_set_text(icon, LV_SYMBOL_WARNING);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_set_style_text_color(icon, lv_color_hex(0xFFB300), LV_PART_MAIN);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -70);

    lv_obj_t *msg = lv_label_create(s_nobind_scr);
    lv_label_set_long_mode(msg, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(msg, 300);
    lv_label_set_text(msg, "No light bound.\n\n"
                           "Bind an on/off or dimmable light to this switch "
                           "from your Matter controller to show the controls.");
    lv_obj_set_style_text_align(msg, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(msg, lv_color_hex(0xC0C0C0), LV_PART_MAIN);
    lv_obj_align(msg, LV_ALIGN_CENTER, 0, 10);
}

/* Show exactly one top-level screen for s_mode. Caller must hold s_lvgl_mtx. */
static void apply_mode_locked(void)
{
    if (!s_boot_scr || !s_pair_scr || !s_nobind_scr || !s_tileview) {
        return;
    }
    struct { lv_obj_t *obj; bool show; } screens[] = {
        { s_boot_scr,   s_mode == LIGHT_UI_MODE_BOOTING    },
        { s_pair_scr,   s_mode == LIGHT_UI_MODE_PAIRING    },
        { s_nobind_scr, s_mode == LIGHT_UI_MODE_NO_BINDING },
        { s_tileview,   s_mode == LIGHT_UI_MODE_READY      },
    };
    for (auto &s : screens) {
        if (s.show) {
            lv_obj_clear_flag(s.obj, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s.obj, LV_OBJ_FLAG_HIDDEN);
        }
    }
    /* Always (re)enter the controls on the bulb page. */
    if (s_mode == LIGHT_UI_MODE_READY && s_home_tile) {
        lv_obj_set_tile(s_tileview, s_home_tile, LV_ANIM_OFF);
    }
}

/* Auto-dismiss the alert overlay. Runs inside lv_timer_handler() (LVGL task, lock held). */
static void alert_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);
    if (s_alert_overlay) {
        lv_obj_add_flag(s_alert_overlay, LV_OBJ_FLAG_HIDDEN);
    }
    s_alert_visible = false;
    if (s_alert_timer) {
        lv_timer_pause(s_alert_timer);
    }
}

/* Build the (hidden) alert overlay on the top layer so it draws above every page and screen.
 * Caller must hold s_lvgl_mtx. */
static void build_alert_overlay_locked(void)
{
    /* Full-screen black backdrop (matches the pages' black ground). */
    s_alert_overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(s_alert_overlay);
    lv_obj_set_size(s_alert_overlay, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_opa(s_alert_overlay, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_alert_overlay, lv_color_hex(COL_VOID), LV_PART_MAIN);
    lv_obj_clear_flag(s_alert_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_alert_overlay, LV_OBJ_FLAG_OVERFLOW_VISIBLE);   /* don't clip the card shadow */

    /* One big "sticker" card filling the safe area (14 px margins), same footprint as the Home
     * cards. Colour set per-alert. Contents are a flex column centred both ways. */
    s_alert_card = make_card(s_alert_overlay, COL_ALERT, 340, 414);
    lv_obj_align(lv_obj_get_parent(s_alert_card), LV_ALIGN_TOP_MID, 0, 14);
    lv_obj_set_flex_flow(s_alert_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_alert_card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(s_alert_card, 12, 0);

    lv_obj_t *badge = make_badge(s_alert_card, 64, COL_INK);
    s_alert_icon = lv_label_create(badge);
    lv_obj_center(s_alert_icon);
    lv_obj_set_style_text_font(s_alert_icon, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_alert_icon, lv_color_hex(COL_BONE), 0);
    lv_label_set_text(s_alert_icon, LV_SYMBOL_WARNING);

    s_alert_title = make_label(s_alert_card, &brutal_30, COL_INK, "");
    lv_obj_set_style_text_align(s_alert_title, LV_TEXT_ALIGN_CENTER, 0);

    s_alert_sub = make_label(s_alert_card, &lv_font_montserrat_16, COL_INK, "");
    lv_obj_set_style_text_opa(s_alert_sub, LV_OPA_70, 0);
    lv_label_set_long_mode(s_alert_sub, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_alert_sub, 280);
    lv_obj_set_style_text_align(s_alert_sub, LV_TEXT_ALIGN_CENTER, 0);

    s_alert_timer = lv_timer_create(alert_timer_cb, ALERT_VISIBLE_MS, NULL);
    lv_timer_pause(s_alert_timer);

    lv_obj_add_flag(s_alert_overlay, LV_OBJ_FLAG_HIDDEN);
}

/* Configure a tileview tile as a black, non-scrolling, top-aligned flex column with padding. */
static void setup_page(lv_obj_t *t)
{
    lv_obj_set_style_bg_color(t, lv_color_hex(COL_VOID), 0);
    lv_obj_clear_flag(t, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(t, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(t, 14, 0);
    lv_obj_set_style_pad_row(t, 11, 0);
    lv_obj_set_flex_align(t, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_add_flag(t, LV_OBJ_FLAG_OVERFLOW_VISIBLE);   /* don't clip card hard-shadows */
}

/* Larger weather L/H chip: small caption + bold value (brutal_30). Returns the value label. */
static lv_obj_t *make_wx_chip(lv_obj_t *parent, const char *cap)
{
    lv_obj_t *chip = lv_obj_create(parent);
    lv_obj_remove_style_all(chip);
    lv_obj_set_style_bg_opa(chip, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(chip, lv_color_hex(COL_INK), 0);
    lv_obj_set_style_radius(chip, 13, 0);
    lv_obj_set_style_pad_hor(chip, 11, 0);
    lv_obj_set_style_pad_ver(chip, 5, 0);
    lv_obj_set_style_pad_column(chip, 5, 0);
    lv_obj_set_size(chip, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(chip, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(chip, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(chip, LV_OBJ_FLAG_SCROLLABLE);
    make_label(chip, FONT_LABEL, COL_BONE, cap);
    lv_obj_t *v = make_label(chip, &brutal_44, COL_BONE, "--°");
    /* Arial-Black's line box has descent space the digits don't use, so they look top-heavy in
     * the chip; nudge the value down a few px to sit optically centred. */
    lv_obj_set_style_translate_y(v, 6, 0);
    return v;
}

/* Refresh the ambient clock from the system time (set by the Matter controller). The RTC starts
 * at the 1970 epoch, so a value before ~2023 means time has not synced yet — show a placeholder
 * rather than a wrong time. Runs on the LVGL task (timer callback, lock held). */
static void update_clock_locked(void)
{
    if (!s_clock_time) {
        return;
    }
    time_t now = time(NULL);
    if (now < 1700000000) {   /* not synced yet (Jan 2023) */
        lv_label_set_text(s_clock_time, "--:--");
        if (s_clock_date) lv_label_set_text(s_clock_date, "SYNCING");
        return;
    }
    /* time() is UTC; add the controller-provided offset and break down with gmtime_r (which does
     * NOT re-apply a timezone), so the fields read as local wall-clock. */
    time_t local = now + s_utc_offset_s;
    struct tm tmv;
    gmtime_r(&local, &tmv);
    char hm[8], dow[16], dm[16], date[40];
    strftime(hm,  sizeof(hm),  "%H:%M", &tmv);
    strftime(dow, sizeof(dow), "%A",    &tmv);   /* Sunday   */
    strftime(dm,  sizeof(dm),  "%d %b", &tmv);   /* 12 Jul   */
    for (char *p = dow; *p; p++) *p = (char)toupper((unsigned char)*p);
    for (char *p = dm;  *p; p++) *p = (char)toupper((unsigned char)*p);
    snprintf(date, sizeof(date), "%s\n%s", dow, dm);
    lv_label_set_text(s_clock_time, hm);
    if (s_clock_date) lv_label_set_text(s_clock_date, date);
}

static void clock_timer_cb(lv_timer_t *timer) { LV_UNUSED(timer); update_clock_locked(); }

/* ── Per-device "Lights" page (modal, opened by tapping the Home light icon) ── */

static ui_device_t *ui_device_find(uint64_t node, uint16_t ep, bool create)
{
    for (auto &d : s_ui_devices) if (d.valid && d.node == node && d.ep == ep) return &d;
    if (!create) return NULL;
    for (auto &d : s_ui_devices) if (!d.valid) {
        d = {}; d.valid = true; d.node = node; d.ep = ep; d.kind = DEV_KIND_LIGHT;
        return &d;
    }
    return NULL;
}

static void device_card_event_cb(lv_event_t *e);   /* fwd */

/* Rebuild the device grid from s_ui_devices. Cheap (few devices); called on any change. Never call
 * from within a card's own event handler (it deletes the cards) — use lv_async_call for that. */
static void rebuild_devices_grid_locked(void)
{
    if (!s_devices_grid) return;
    lv_coord_t saved_scroll = lv_obj_get_scroll_y(s_devices_grid);   /* keep position across rebuild */
    lv_obj_clean(s_devices_grid);
    int shown = 0;
    for (auto &d : s_ui_devices) {
        if (!d.valid || !d.have_state) continue;
        shown++;
        /* On = amber card; off = dark grey with amber accents (icon + caption). */
        lv_obj_t *card = make_card(s_devices_grid, d.on ? COL_AMBER : COL_DIM, LV_PCT(48), 100);
        lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_user_data(card, &d);
        lv_obj_add_event_cb(card, device_card_event_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_row(card, 6, 0);
        lv_obj_t *badge = make_badge(card, 44, COL_INK);
        badge_icon(badge, d.kind == DEV_KIND_PLUG ? &ic_plug : &ic_bulb, COL_AMBER);
        make_caplabel(card, d.on ? COL_INK : COL_AMBER, d.kind == DEV_KIND_PLUG ? "PLUG" : "LIGHT");
    }
    if (shown == 0) {
        lv_obj_t *msg = make_label(s_devices_grid, &lv_font_montserrat_16, COL_BONE, "No lights found");
        lv_obj_set_style_text_opa(msg, LV_OPA_60, 0);
    }
    /* Restore the scroll offset (a toggle rebuilds the grid; without this it jumps to the top). */
    lv_obj_update_layout(s_devices_grid);
    lv_obj_scroll_to_y(s_devices_grid, saved_scroll, LV_ANIM_OFF);
}

static void rebuild_devices_async(void *unused) { LV_UNUSED(unused); rebuild_devices_grid_locked(); }

static void device_card_event_cb(lv_event_t *e)
{
    ui_device_t *d = (ui_device_t *)lv_obj_get_user_data(lv_event_get_target(e));
    if (!d || !d->valid) return;
    if (s_dev_toggle_n < (int)(sizeof(s_dev_toggle_q) / sizeof(s_dev_toggle_q[0]))) {
        s_dev_toggle_q[s_dev_toggle_n++] = { d->fabric, d->node, d->ep };   /* sent outside the lock */
    }
    d->on = !d->on;   /* optimistic; the OnOff subscription report will confirm/correct it */
    lv_async_call(rebuild_devices_async, NULL);   /* rebuild AFTER this event returns (it deletes the card) */
    ESP_LOGI(TAG, "Device tap: node 0x%llx ep %u -> %s",
             (unsigned long long)d->node, d->ep, d->on ? "ON" : "OFF");
}

static void device_back_event_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    if (s_devices_scr) lv_obj_add_flag(s_devices_scr, LV_OBJ_FLAG_HIDDEN);
}

static void open_devices_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    if (s_devices_scr) {
        lv_obj_move_foreground(s_devices_scr);
        lv_obj_clear_flag(s_devices_scr, LV_OBJ_FLAG_HIDDEN);
    }
}

static void build_devices_screen_locked(lv_obj_t *scr)
{
    s_devices_scr = lv_obj_create(scr);
    lv_obj_remove_style_all(s_devices_scr);
    lv_obj_set_size(s_devices_scr, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_devices_scr, lv_color_hex(COL_VOID), 0);
    lv_obj_set_style_bg_opa(s_devices_scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_devices_scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(s_devices_scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(s_devices_scr, 14, 0);
    lv_obj_set_style_pad_row(s_devices_scr, 12, 0);
    lv_obj_add_flag(s_devices_scr, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    /* Header: back button + title. */
    lv_obj_t *hdr = lv_obj_create(s_devices_scr);
    lv_obj_remove_style_all(hdr);
    lv_obj_set_size(hdr, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(hdr, 12, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *back = make_badge(hdr, 40, COL_BONE);
    lv_obj_add_flag(back, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(back, device_back_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *ba = lv_label_create(back);
    lv_label_set_text(ba, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(ba, lv_color_hex(COL_INK), 0);
    lv_obj_center(ba);

    make_caplabel(hdr, COL_BONE, "LIGHTS");

    /* Card grid (2 per row). Fills the height below the header and packs cards from the TOP-LEFT
     * (START on both axes), so one device sits top-left, not centred. Scrolls vertically when the
     * devices overflow the screen. */
    s_devices_grid = lv_obj_create(s_devices_scr);
    lv_obj_remove_style_all(s_devices_grid);
    lv_obj_set_width(s_devices_grid, LV_PCT(100));
    lv_obj_set_flex_grow(s_devices_grid, 1);              /* fill remaining height */
    lv_obj_set_flex_flow(s_devices_grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_row(s_devices_grid, 11, 0);
    lv_obj_set_style_pad_column(s_devices_grid, 11, 0);
    lv_obj_set_style_pad_bottom(s_devices_grid, 8, 0);    /* room for the last row's drop shadow */
    lv_obj_set_flex_align(s_devices_grid, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_scroll_dir(s_devices_grid, LV_DIR_VER);    /* only scroll vertically */
    lv_obj_set_scrollbar_mode(s_devices_grid, LV_SCROLLBAR_MODE_AUTO);

    rebuild_devices_grid_locked();
    lv_obj_add_flag(s_devices_scr, LV_OBJ_FLAG_HIDDEN);
}

/* ── Time & Date settings page (modal, opened by tapping the clock card) ── */

#define TIMESET_YEAR_START 2024
#define TIMESET_YEAR_COUNT 16     /* 2024 … 2039 */
static const char *const TIMESET_MONTHS =
    "JAN\nFEB\nMAR\nAPR\nMAY\nJUN\nJUL\nAUG\nSEP\nOCT\nNOV\nDEC";

/* Build "start .. start+count-1", zero-padded to `width`, newline-separated, for an lv_roller. */
static void build_num_opts(char *buf, size_t n, int start, int count, int width)
{
    size_t off = 0;
    for (int i = 0; i < count && off < n; i++) {
        off += snprintf(buf + off, n - off, "%s%0*d", i ? "\n" : "", width, start + i);
    }
}

/* A dark roller with a violet selected-row highlight, matching the clock card's accent. */
static lv_obj_t *make_time_roller(lv_obj_t *parent, const char *opts, lv_coord_t w)
{
    lv_obj_t *r = lv_roller_create(parent);
    lv_roller_set_options(r, opts, LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(r, 3);
    lv_obj_set_width(r, w);
    lv_obj_set_style_bg_color(r, lv_color_hex(COL_DIM), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(r, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_text_color(r, lv_color_hex(COL_BONE), LV_PART_MAIN);
    lv_obj_set_style_text_font(r, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_line_space(r, 8, LV_PART_MAIN);
    lv_obj_set_style_radius(r, 12, LV_PART_MAIN);
    lv_obj_set_style_border_width(r, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(r, lv_color_hex(COL_INK), LV_PART_MAIN);
    lv_obj_set_style_bg_color(r, lv_color_hex(COL_VIOLET), LV_PART_SELECTED);
    lv_obj_set_style_bg_opa(r, LV_OPA_COVER, LV_PART_SELECTED);
    lv_obj_set_style_text_color(r, lv_color_hex(COL_INK), LV_PART_SELECTED);
    return r;
}

/* Transparent, centred flex row (holds the rollers). */
static lv_obj_t *make_hrow(lv_obj_t *parent, lv_coord_t pad_col)
{
    lv_obj_t *r = lv_obj_create(parent);
    lv_obj_remove_style_all(r);
    lv_obj_set_size(r, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(r, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(r, pad_col, 0);
    lv_obj_clear_flag(r, LV_OBJ_FLAG_SCROLLABLE);
    return r;
}

static void timeset_back_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    if (s_timeset_scr) lv_obj_add_flag(s_timeset_scr, LV_OBJ_FLAG_HIDDEN);
}

static void auto_switch_cb(lv_event_t *e)
{
    lv_obj_t *sw = lv_event_get_target(e);
    s_time_auto = lv_obj_has_state(sw, LV_STATE_CHECKED);
    time_auto_save(s_time_auto);
    ESP_LOGI(TAG, "Time auto-sync from controller %s", s_time_auto ? "ON" : "OFF");
}

/* Apply the rollers to the system clock: interpret the fields as local wall time, back out the
 * controller's timezone offset to store UTC (the display re-adds it), and remember the base so a
 * later controller push can be undone while auto-sync is off. */
static void timeset_set_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    if (!s_ro_year) return;
    int year = TIMESET_YEAR_START + (int)lv_roller_get_selected(s_ro_year);
    int mon  = (int)lv_roller_get_selected(s_ro_mon) + 1;    /* 1-12 */
    int day  = (int)lv_roller_get_selected(s_ro_day) + 1;    /* 1-31 */
    int hour = (int)lv_roller_get_selected(s_ro_hour);
    int min  = (int)lv_roller_get_selected(s_ro_min);

    time_t local = utc_from_ymd_hms(year, mon, day, hour, min, 0);
    time_t utc   = local - s_utc_offset_s;
    struct timeval tv; tv.tv_sec = utc; tv.tv_usec = 0;
    settimeofday(&tv, NULL);

    s_manual_utc_base       = utc;
    s_manual_uptime_base_us = esp_timer_get_time();
    s_have_manual           = true;

    update_clock_locked();
    if (s_timeset_scr) lv_obj_add_flag(s_timeset_scr, LV_OBJ_FLAG_HIDDEN);
    ESP_LOGI(TAG, "Manual time set: %04d-%02d-%02d %02d:%02d local (auto=%s)",
             year, mon, day, hour, min, s_time_auto ? "on" : "off");
}

/* Seed the rollers from the current displayed local time (or a sane default if unsynced), then show
 * the page over the top of whatever screen is active. */
static void open_timeset_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    if (!s_timeset_scr) return;
    time_t now = time(NULL);
    struct tm tmv;
    if (now < 1700000000) {                 /* not synced — start from a neutral default */
        memset(&tmv, 0, sizeof(tmv));
        tmv.tm_year = 2026 - 1900; tmv.tm_mday = 1; tmv.tm_hour = 12;
    } else {
        time_t local = now + s_utc_offset_s;
        gmtime_r(&local, &tmv);
    }
    int yi = tmv.tm_year + 1900 - TIMESET_YEAR_START;
    if (yi < 0) yi = 0;
    if (yi > TIMESET_YEAR_COUNT - 1) yi = TIMESET_YEAR_COUNT - 1;
    lv_roller_set_selected(s_ro_year, yi,              LV_ANIM_OFF);
    lv_roller_set_selected(s_ro_mon,  tmv.tm_mon,      LV_ANIM_OFF);
    lv_roller_set_selected(s_ro_day,  tmv.tm_mday - 1, LV_ANIM_OFF);
    lv_roller_set_selected(s_ro_hour, tmv.tm_hour,     LV_ANIM_OFF);
    lv_roller_set_selected(s_ro_min,  tmv.tm_min,      LV_ANIM_OFF);
    if (s_auto_switch) {
        if (s_time_auto) lv_obj_add_state(s_auto_switch, LV_STATE_CHECKED);
        else             lv_obj_clear_state(s_auto_switch, LV_STATE_CHECKED);
    }
    lv_obj_move_foreground(s_timeset_scr);
    lv_obj_clear_flag(s_timeset_scr, LV_OBJ_FLAG_HIDDEN);
}

static void build_timeset_screen_locked(lv_obj_t *scr)
{
    s_timeset_scr = lv_obj_create(scr);
    lv_obj_remove_style_all(s_timeset_scr);
    lv_obj_set_size(s_timeset_scr, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_timeset_scr, lv_color_hex(COL_VOID), 0);
    lv_obj_set_style_bg_opa(s_timeset_scr, LV_OPA_COVER, 0);
    lv_obj_set_flex_flow(s_timeset_scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_timeset_scr, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(s_timeset_scr, 14, 0);
    lv_obj_set_style_pad_row(s_timeset_scr, 10, 0);
    lv_obj_set_scroll_dir(s_timeset_scr, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_timeset_scr, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_add_flag(s_timeset_scr, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    /* Header: back button + violet title (ties it to the clock card). */
    lv_obj_t *hdr = lv_obj_create(s_timeset_scr);
    lv_obj_remove_style_all(hdr);
    lv_obj_set_size(hdr, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(hdr, 12, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *back = make_badge(hdr, 40, COL_BONE);
    lv_obj_add_flag(back, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(back, timeset_back_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *ba = lv_label_create(back);
    lv_label_set_text(ba, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_color(ba, lv_color_hex(COL_INK), 0);
    lv_obj_center(ba);
    make_caplabel(hdr, COL_VIOLET, "TIME & DATE");

    char buf[256];

    /* TIME: HH : MM */
    make_caplabel(s_timeset_scr, COL_BONE, "TIME");
    lv_obj_t *trow = make_hrow(s_timeset_scr, 8);
    build_num_opts(buf, sizeof(buf), 0, 24, 2); s_ro_hour = make_time_roller(trow, buf, 76);
    lv_obj_t *colon = make_label(trow, &brutal_30, COL_BONE, ":");
    LV_UNUSED(colon);
    build_num_opts(buf, sizeof(buf), 0, 60, 2); s_ro_min  = make_time_roller(trow, buf, 76);

    /* DATE: DD MON YYYY */
    make_caplabel(s_timeset_scr, COL_BONE, "DATE");
    lv_obj_t *drow = make_hrow(s_timeset_scr, 8);
    build_num_opts(buf, sizeof(buf), 1, 31, 2);           s_ro_day  = make_time_roller(drow, buf, 62);
    s_ro_mon  = make_time_roller(drow, TIMESET_MONTHS, 92);
    build_num_opts(buf, sizeof(buf), TIMESET_YEAR_START, TIMESET_YEAR_COUNT, 4);
    s_ro_year = make_time_roller(drow, buf, 96);

    /* SET button (violet, matches the clock card). */
    lv_obj_t *setbtn = make_card(s_timeset_scr, COL_VIOLET, LV_PCT(100), 52);
    lv_obj_add_flag(setbtn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(setbtn, timeset_set_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *setlbl = make_caplabel(setbtn, COL_INK, "SET TIME");
    lv_obj_set_style_text_font(setlbl, &lv_font_montserrat_16, 0);
    lv_obj_center(setlbl);

    /* AUTOMATIC toggle: when on (default), a controller time push overrides the manual time. */
    lv_obj_t *arow = make_card(s_timeset_scr, COL_DIM, LV_PCT(100), 62);
    lv_obj_set_flex_flow(arow, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(arow, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t *acol = lv_obj_create(arow);
    lv_obj_remove_style_all(acol);
    lv_obj_set_size(acol, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(acol, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(acol, 3, 0);
    lv_obj_clear_flag(acol, LV_OBJ_FLAG_SCROLLABLE);
    make_caplabel(acol, COL_BONE, "AUTOMATIC");
    lv_obj_t *sub = make_label(acol, FONT_LABEL, COL_BONE, "SYNC FROM CONTROLLER");
    lv_obj_set_style_text_opa(sub, LV_OPA_60, 0);
    lv_obj_set_style_text_letter_space(sub, 1, 0);
    s_auto_switch = lv_switch_create(arow);
    lv_obj_set_style_bg_color(s_auto_switch, lv_color_hex(COL_INK), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_auto_switch, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_auto_switch, lv_color_hex(COL_VIOLET), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(s_auto_switch, lv_color_hex(COL_BONE), LV_PART_KNOB);
    if (s_time_auto) lv_obj_add_state(s_auto_switch, LV_STATE_CHECKED);
    lv_obj_add_event_cb(s_auto_switch, auto_switch_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_add_flag(s_timeset_scr, LV_OBJ_FLAG_HIDDEN);
}

/* ── Settings page (swipe up from Home) ─────────────────────── */

static void style_slider(lv_obj_t *s, uint32_t ind, uint32_t knob);   /* defined with the Controls page */

/* Confirmation overlay OK: latch the terminal action (performed by the LVGL task outside the lock)
 * and dismiss. Reboot/factory-reset don't return, so this only queues them. */
static void confirm_ok_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    if (s_confirm_action == CONFIRM_REBOOT)  s_reboot_pending  = true;
    if (s_confirm_action == CONFIRM_FACTORY) s_factory_pending = true;
    if (s_confirm_overlay) lv_obj_add_flag(s_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    s_confirm_action = CONFIRM_NONE;
}

static void confirm_cancel_cb(lv_event_t *e)
{
    LV_UNUSED(e);
    if (s_confirm_overlay) lv_obj_add_flag(s_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
    s_confirm_action = CONFIRM_NONE;
}

/* Populate + show the confirm overlay for the given CONFIRM_* action. Caller holds s_lvgl_mtx
 * (runs inside an LVGL event handler). */
static void show_confirm_locked(int action)
{
    if (!s_confirm_overlay) return;
    s_confirm_action = action;
    bool factory = (action == CONFIRM_FACTORY);
    card_set_color(s_confirm_card, factory ? COL_ALERT : COL_VIOLET);
    lv_label_set_text(s_confirm_icon, factory ? LV_SYMBOL_TRASH : LV_SYMBOL_REFRESH);
    if (factory) {
        lv_label_set_text(s_confirm_title, "FACTORY RESET?");
        lv_label_set_text(s_confirm_msg,
                          "Erases all Matter pairings and settings. Remove this switch's "
                          "bindings on your controller first. This can't be undone.");
        lv_label_set_text(s_confirm_ok_lbl, "RESET");
    } else {
        lv_label_set_text(s_confirm_title, "REBOOT?");
        lv_label_set_text(s_confirm_msg, "The display will restart. Your clock and settings are kept.");
        lv_label_set_text(s_confirm_ok_lbl, "REBOOT");
    }
    lv_obj_move_foreground(s_confirm_overlay);
    lv_obj_clear_flag(s_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
}

static void reboot_btn_cb(lv_event_t *e)  { LV_UNUSED(e); show_confirm_locked(CONFIRM_REBOOT); }
static void factory_btn_cb(lv_event_t *e) { LV_UNUSED(e); show_confirm_locked(CONFIRM_FACTORY); }

/* A full-width text button (flex-centred label). Returns the button; caller wires the event. */
static lv_obj_t *make_action_btn(lv_obj_t *parent, uint32_t bg, uint32_t fg, const char *sym,
                                 const char *txt, lv_event_cb_t cb)
{
    lv_obj_t *b = make_card(parent, bg, LV_PCT(100), 52);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
    if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_flow(b, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(b, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(b, 8, 0);
    lv_obj_t *ic = make_label(b, &lv_font_montserrat_18, fg, sym);
    LV_UNUSED(ic);
    lv_obj_t *l = make_caplabel(b, fg, txt);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_16, 0);
    return b;
}

/* Build the (hidden) confirmation overlay once on the top layer. Caller holds s_lvgl_mtx. */
static void build_confirm_overlay_locked(void)
{
    s_confirm_overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(s_confirm_overlay);
    lv_obj_set_size(s_confirm_overlay, lv_pct(100), lv_pct(100));
    lv_obj_set_style_bg_opa(s_confirm_overlay, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_confirm_overlay, lv_color_hex(COL_VOID), LV_PART_MAIN);
    lv_obj_clear_flag(s_confirm_overlay, LV_OBJ_FLAG_SCROLLABLE);
    /* Clickable so it swallows taps that miss the buttons instead of falling through to the page
     * below — makes the dialog truly modal. */
    lv_obj_add_flag(s_confirm_overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(s_confirm_overlay, LV_OBJ_FLAG_OVERFLOW_VISIBLE);

    s_confirm_card = make_card(s_confirm_overlay, COL_VIOLET, 320, 320);
    lv_obj_align(lv_obj_get_parent(s_confirm_card), LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_flex_flow(s_confirm_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_confirm_card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(s_confirm_card, 12, 0);

    lv_obj_t *badge = make_badge(s_confirm_card, 56, COL_INK);
    s_confirm_icon = lv_label_create(badge);
    lv_obj_center(s_confirm_icon);
    lv_obj_set_style_text_font(s_confirm_icon, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s_confirm_icon, lv_color_hex(COL_BONE), 0);
    lv_label_set_text(s_confirm_icon, LV_SYMBOL_REFRESH);

    s_confirm_title = make_label(s_confirm_card, &brutal_30, COL_INK, "REBOOT?");
    lv_obj_set_style_text_align(s_confirm_title, LV_TEXT_ALIGN_CENTER, 0);

    s_confirm_msg = make_label(s_confirm_card, &lv_font_montserrat_16, COL_INK, "");
    lv_obj_set_style_text_opa(s_confirm_msg, LV_OPA_80, 0);
    lv_label_set_long_mode(s_confirm_msg, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_confirm_msg, 264);
    lv_obj_set_style_text_align(s_confirm_msg, LV_TEXT_ALIGN_CENTER, 0);

    /* Button row: Cancel (bone) + Confirm (ink). */
    lv_obj_t *row = lv_obj_create(s_confirm_card);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(row, 10, 0);
    lv_obj_set_style_pad_top(row, 4, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *cancel = lv_btn_create(row);
    lv_obj_remove_style_all(cancel);
    lv_obj_set_flex_grow(cancel, 1);
    lv_obj_set_height(cancel, 48);
    lv_obj_set_style_radius(cancel, 14, 0);
    lv_obj_set_style_bg_opa(cancel, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(cancel, lv_color_hex(COL_BONE), 0);
    lv_obj_set_style_border_width(cancel, 2, 0);
    lv_obj_set_style_border_color(cancel, lv_color_hex(COL_INK), 0);
    lv_obj_add_event_cb(cancel, confirm_cancel_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *cl = make_caplabel(cancel, COL_INK, "CANCEL");
    lv_obj_set_style_text_font(cl, &lv_font_montserrat_16, 0);
    lv_obj_center(cl);

    lv_obj_t *ok = lv_btn_create(row);
    lv_obj_remove_style_all(ok);
    lv_obj_set_flex_grow(ok, 1);
    lv_obj_set_height(ok, 48);
    lv_obj_set_style_radius(ok, 14, 0);
    lv_obj_set_style_bg_opa(ok, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(ok, lv_color_hex(COL_INK), 0);
    lv_obj_set_style_border_width(ok, 2, 0);
    lv_obj_set_style_border_color(ok, lv_color_hex(COL_INK), 0);
    lv_obj_add_event_cb(ok, confirm_ok_cb, LV_EVENT_CLICKED, NULL);
    s_confirm_ok_lbl = make_caplabel(ok, COL_BONE, "REBOOT");
    lv_obj_set_style_text_font(s_confirm_ok_lbl, &lv_font_montserrat_16, 0);
    lv_obj_center(s_confirm_ok_lbl);

    lv_obj_add_flag(s_confirm_overlay, LV_OBJ_FLAG_HIDDEN);
}

/* Live-update the backlight as the user drags; persist on release. Runs on the LVGL task. */
static void set_bright_slider_cb(lv_event_t *e)
{
    switch (lv_event_get_code(e)) {
    case LV_EVENT_VALUE_CHANGED:
        s_disp_bright_pct = lv_slider_get_value(s_set_bright_slider);
        if (s_set_bright_label) lv_label_set_text_fmt(s_set_bright_label, "%d%%", s_disp_bright_pct);
        amoled_set_brightness(disp_level_from_pct(s_disp_bright_pct));   /* live preview */
        break;
    case LV_EVENT_RELEASED:
        disp_bright_save(s_disp_bright_pct);
        ESP_LOGI(TAG, "Display brightness -> %d%%", s_disp_bright_pct);
        break;
    default:
        break;
    }
}

/* One settings "slider card" (used for display brightness — a continuous value where a slider fits):
 * accent bg, value label (top-left), caption below it, badge (top-right), slider along the bottom
 * with a comfortable gap above it. Caller stores the slider + value-label pointers. */
static void make_slider_card(lv_obj_t *parent, uint32_t bg, const char *cap, const lv_img_dsc_t *icon,
                             lv_obj_t **out_value, lv_obj_t **out_slider, int rmin, int rmax, int rval,
                             lv_event_cb_t cb)
{
    lv_obj_t *c = make_card(parent, bg, LV_PCT(100), 120);
    lv_obj_t *badge = make_badge(c, 40, COL_INK);
    lv_obj_align(badge, LV_ALIGN_TOP_RIGHT, 0, 0);
    badge_icon(badge, icon, COL_BONE);
    *out_value = make_label(c, &brutal_30, COL_INK, "");
    lv_obj_align(*out_value, LV_ALIGN_TOP_LEFT, 0, 2);
    lv_obj_t *l = make_label(c, &lv_font_montserrat_16, COL_INK, cap);
    lv_obj_set_style_text_letter_space(l, 1, 0);
    lv_obj_set_style_text_opa(l, LV_OPA_70, 0);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 2, 38);
    lv_obj_t *s = lv_slider_create(c);
    lv_obj_set_size(s, LV_PCT(96), 22);
    lv_obj_align(s, LV_ALIGN_BOTTOM_MID, 0, -6);      /* extra gap below the caption */
    lv_slider_set_range(s, rmin, rmax);
    lv_slider_set_value(s, rval, LV_ANIM_OFF);
    style_slider(s, COL_INK, COL_BONE);
    /* Settings pages horizontally; the tile also allows a vertical (back-to-Home) swipe. Own both
     * gestures so a drag on the slider never pages the tileview. */
    lv_obj_clear_flag(s, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
    lv_obj_add_event_cb(s, cb, LV_EVENT_ALL, NULL);
    *out_slider = s;
}

/* ── Display-off timeout: a −/+ stepper over sensible presets, so the value snaps to clean
 * numbers and is easy to tap. ── */
static const int kIdlePresets[] = { 10, 15, 20, 30, 45, 60, 90, 120, 180, 300, 600 };
#define IDLE_PRESET_COUNT ((int)(sizeof(kIdlePresets) / sizeof(kIdlePresets[0])))

static int idle_nearest_index(int secs)
{
    int best = 0, bestd = 1 << 30;
    for (int i = 0; i < IDLE_PRESET_COUNT; i++) {
        int d = kIdlePresets[i] - secs;
        if (d < 0) d = -d;
        if (d < bestd) { bestd = d; best = i; }
    }
    return best;
}

/* Apply + persist a new idle-off timeout and refresh the value label. Runs on the LVGL task. */
static void idle_apply_locked(int secs)
{
    s_idle_timeout_ms = (int64_t)secs * 1000;
    if (s_set_timeout_label) lv_label_set_text_fmt(s_set_timeout_label, "%d S", secs);
    idle_timeout_save(secs);
    s_last_active_ms = esp_timer_get_time() / 1000;   /* restart the idle countdown from now */
    ESP_LOGI(TAG, "Display idle-off -> %d s", secs);
}

/* − / + stepper button handler: nudge to the previous/next preset (dir = -1 / +1 via user_data). */
static void idle_step_cb(lv_event_t *e)
{
    int dir = (int)(intptr_t)lv_event_get_user_data(e);
    int idx = idle_nearest_index((int)(s_idle_timeout_ms / 1000)) + dir;
    if (idx < 0) idx = 0;
    if (idx > IDLE_PRESET_COUNT - 1) idx = IDLE_PRESET_COUNT - 1;
    idle_apply_locked(kIdlePresets[idx]);
}

static lv_obj_t *make_step_btn(lv_obj_t *parent, const char *sym, int dir)
{
    lv_obj_t *b = make_badge(parent, 46, COL_INK);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(b, idle_step_cb, LV_EVENT_CLICKED, (void *)(intptr_t)dir);
    lv_obj_t *l = make_label(b, &lv_font_montserrat_28, COL_BONE, sym);
    lv_obj_center(l);
    return b;
}

/* Display-off timeout card (sky): caption top-left, moon badge top-right, [−] value [+] stepper
 * centred along the bottom. */
static void build_timeout_card_locked(lv_obj_t *t)
{
    lv_obj_t *c = make_card(t, COL_SKY, LV_PCT(100), 120);
    lv_obj_t *badge = make_badge(c, 40, COL_INK);
    lv_obj_align(badge, LV_ALIGN_TOP_RIGHT, 0, 0);
    /* The moon glyph sits high in its canvas, so plain centring leaves it above the badge centre —
     * nudge it down a few px (inline instead of badge_icon(), which centres exactly). */
    lv_obj_t *moon = lv_img_create(badge);
    lv_img_set_src(moon, &ic_clearnight);
    lv_obj_center(moon);
    lv_obj_set_style_translate_y(moon, 3, 0);
    lv_obj_set_style_img_recolor(moon, lv_color_hex(COL_BONE), 0);
    lv_obj_set_style_img_recolor_opa(moon, LV_OPA_COVER, 0);
    lv_obj_t *l = make_label(c, &lv_font_montserrat_16, COL_INK, "DISPLAY OFF");
    lv_obj_set_style_text_letter_space(l, 1, 0);
    lv_obj_set_style_text_opa(l, LV_OPA_70, 0);
    lv_obj_align(l, LV_ALIGN_TOP_LEFT, 2, 2);

    lv_obj_t *row = lv_obj_create(c);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(row, 12, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(row, LV_ALIGN_BOTTOM_MID, 0, -2);

    make_step_btn(row, LV_SYMBOL_MINUS, -1);
    s_set_timeout_label = make_label(row, &brutal_30, COL_INK, "");
    lv_obj_set_width(s_set_timeout_label, 130);
    lv_obj_set_style_text_align(s_set_timeout_label, LV_TEXT_ALIGN_CENTER, 0);
    make_step_btn(row, LV_SYMBOL_PLUS, +1);

    lv_label_set_text_fmt(s_set_timeout_label, "%d S", (int)(s_idle_timeout_ms / 1000));
}

static void build_settings_page_locked(lv_obj_t *t)
{
    setup_page(t);
    lv_obj_set_style_pad_row(t, 10, 0);
    /* The buttons flex-grow to fill the page, but their 6 px drop shadow extends below the wrapper.
     * setup_page's default 14 px bottom pad would let the last shadow reach the display's rounded
     * bottom corner and get clipped. Pad the bottom by 14 + CARD_SH so the grown buttons stop where
     * the other pages' bottom cards stop and the shadow clears the corner. */
    lv_obj_set_style_pad_bottom(t, 14 + CARD_SH + 4, 0);
    make_caplabel(t, COL_BONE, "SETTINGS");

    /* Display brightness (amber, continuous slider). */
    make_slider_card(t, COL_AMBER, "DISPLAY BRIGHTNESS", &ic_sunny, &s_set_bright_label, &s_set_bright_slider,
                     DISP_BRIGHT_MIN_PCT, 100, s_disp_bright_pct, set_bright_slider_cb);
    lv_label_set_text_fmt(s_set_bright_label, "%d%%", s_disp_bright_pct);

    /* Display-off timeout (sky, −/+ stepper over presets). */
    build_timeout_card_locked(t);

    /* Reboot (violet) + Factory reset (red) actions, each behind a confirmation overlay. The two
     * buttons flex-grow to share the remaining height, so the page fills the full display and the
     * targets are large and easy to tap. */
    lv_obj_t *rb = make_action_btn(t, COL_VIOLET, COL_INK, LV_SYMBOL_REFRESH, "REBOOT", reboot_btn_cb);
    lv_obj_t *fb = make_action_btn(t, COL_ALERT, COL_BONE, LV_SYMBOL_TRASH, "FACTORY RESET", factory_btn_cb);
    lv_obj_set_flex_grow(lv_obj_get_parent(rb), 1);   /* wrapper is the flex item (see make_card) */
    lv_obj_set_flex_grow(lv_obj_get_parent(fb), 1);
}

/* ── Home page: weather + light toggle ──────────────────────── */
static void build_home_page_locked(lv_obj_t *t)
{
    setup_page(t);
    lv_obj_set_style_pad_row(t, 15, 0);   /* a touch more space between the weather + light cards */
    make_caplabel(t, COL_BONE, "HOME");

    /* Ambient clock card (violet) at the top: big time left, weekday + date stacked right.
     * Updated once a second by clock_timer_cb; time comes from the Matter controller (no cloud,
     * no Home Assistant — the fabric itself sets the clock). */
    lv_obj_t *clk = make_card(t, COL_VIOLET, LV_PCT(100), 76);
    lv_obj_add_flag(clk, LV_OBJ_FLAG_CLICKABLE);   /* tap to open Time & Date settings */
    lv_obj_add_event_cb(clk, open_timeset_cb, LV_EVENT_CLICKED, NULL);
    s_clock_time = make_label(clk, &brutal_44, COL_INK, "--:--");
    lv_obj_align(s_clock_time, LV_ALIGN_LEFT_MID, 2, 0);
    s_clock_date = make_label(clk, &lv_font_montserrat_16, COL_INK, "SYNCING");
    lv_obj_set_style_text_align(s_clock_date, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_letter_space(s_clock_date, 1, 0);
    lv_obj_set_style_text_line_space(s_clock_date, 3, 0);
    lv_obj_set_style_text_opa(s_clock_date, LV_OPA_70, 0);
    lv_obj_align(s_clock_date, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_timer_create(clock_timer_cb, 1000, NULL);

    /* Weather card (sky): location + condition top-left, icon badge top-right,
     * big current temp bottom-left, L/H chips bottom-right. */
    lv_obj_t *wx = make_card(t, COL_SKY, LV_PCT(100), 176);
    /* Clamp width and ellipsize so a long place name or condition ("Thunderstorm with hail")
     * can't run under the icon badge or off the card edge. Badge is 52 px at the top-right. */
    s_wx_location_label = make_label(wx, &lv_font_montserrat_20, COL_INK, "WEATHER");
    lv_obj_set_style_text_letter_space(s_wx_location_label, 1, 0);
    lv_label_set_long_mode(s_wx_location_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(s_wx_location_label, 248);
    lv_obj_align(s_wx_location_label, LV_ALIGN_TOP_LEFT, 0, 0);
    s_wx_condition_label = make_label(wx, &lv_font_montserrat_18, COL_INK, "--");
    lv_obj_set_style_text_opa(s_wx_condition_label, LV_OPA_70, 0);
    lv_label_set_long_mode(s_wx_condition_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(s_wx_condition_label, 248);
    lv_obj_align(s_wx_condition_label, LV_ALIGN_TOP_LEFT, 0, 26);

    lv_obj_t *wxbadge = make_badge(wx, 52, COL_INK);
    lv_obj_align(wxbadge, LV_ALIGN_TOP_RIGHT, 0, -2);
    s_wx_icon = lv_img_create(wxbadge);
    lv_img_set_src(s_wx_icon, &ic_cloudy);
    lv_obj_center(s_wx_icon);
    lv_obj_set_style_img_recolor(s_wx_icon, lv_color_hex(COL_BONE), 0);
    lv_obj_set_style_img_recolor_opa(s_wx_icon, LV_OPA_COVER, 0);

    s_wx_now_label = make_label(wx, &brutal_44, COL_INK, "--°");
    lv_obj_align(s_wx_now_label, LV_ALIGN_BOTTOM_LEFT, 0, 2);

    lv_obj_t *chips = lv_obj_create(wx);
    lv_obj_remove_style_all(chips);
    lv_obj_set_size(chips, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(chips, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(chips, 7, 0);
    lv_obj_clear_flag(chips, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(chips, LV_ALIGN_BOTTOM_RIGHT, 0, 2);
    s_wx_min_label = make_wx_chip(chips, "L");
    s_wx_max_label = make_wx_chip(chips, "H");

    /* Light toggle card (always green) — tap toggles; state shown by label + sliding knob.
     * Height matches the Indoor air-quality card (100) so their bottoms align and the drop shadow
     * clears the display's rounded bottom-right corner. */
    s_light_card = make_card(t, COL_LIME, LV_PCT(100), 100);
    lv_obj_add_flag(s_light_card, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_light_card, bulb_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bulbbadge = make_badge(s_light_card, 46, COL_INK);
    lv_obj_align(bulbbadge, LV_ALIGN_LEFT_MID, 0, 0);
    /* Tapping the icon opens the per-device page; taps elsewhere on the card still master-toggle. */
    lv_obj_add_flag(bulbbadge, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(bulbbadge, open_devices_cb, LV_EVENT_CLICKED, NULL);
    s_light_icon = lv_img_create(bulbbadge);
    lv_img_set_src(s_light_icon, &ic_bulb);
    lv_obj_center(s_light_icon);
    s_light_cap = make_caplabel(s_light_card, COL_BONE, "LIGHTS");
    lv_obj_align(s_light_cap, LV_ALIGN_LEFT_MID, 60, -15);
    s_light_state = make_label(s_light_card, &brutal_30, COL_BONE, "OFF");
    lv_obj_align(s_light_state, LV_ALIGN_LEFT_MID, 60, 12);
    /* toggle track + knob dot */
    lv_obj_t *track = lv_obj_create(s_light_card);
    lv_obj_remove_style_all(track);
    lv_obj_set_size(track, 66, 36);
    lv_obj_set_style_bg_opa(track, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(track, lv_color_hex(COL_INK), 0);
    lv_obj_set_style_radius(track, 17, 0);
    /* Decorative only — clear CLICKABLE (LVGL sets it on every lv_obj by default) so a tap on the
     * toggle graphic falls through to the card's own click handler instead of being swallowed. */
    lv_obj_clear_flag(track, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align(track, LV_ALIGN_RIGHT_MID, 0, 0);
    s_light_knob = lv_obj_create(track);
    lv_obj_remove_style_all(s_light_knob);
    lv_obj_set_size(s_light_knob, 26, 26);
    lv_obj_set_style_bg_opa(s_light_knob, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_light_knob, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(s_light_knob, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);  /* click-through */
    lv_obj_set_pos(s_light_knob, KNOB_X_OFF, 5);   /* initial OFF; apply_state animates to real state */

    apply_state_locked();
    apply_weather_locked();
    update_clock_locked();
}

/* ── Controls page: brightness + warmth/colour ──────────────── */
static lv_obj_t *make_seg_btn(lv_obj_t *parent, const char *txt, lv_event_cb_t cb)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_remove_style_all(b);
    lv_obj_set_flex_grow(b, 1);
    lv_obj_set_height(b, 54);
    lv_obj_set_style_radius(b, 15, 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(b, 2, 0);
    lv_obj_set_style_border_color(b, lv_color_hex(COL_BONE), 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l = make_caplabel(b, COL_BONE, txt);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_16, 0);
    lv_obj_center(l);
    return b;
}

static void style_slider(lv_obj_t *s, uint32_t ind, uint32_t knob)
{
    lv_obj_set_style_bg_color(s, lv_color_hex(COL_INK), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s, LV_OPA_30, LV_PART_MAIN);
    lv_obj_set_style_radius(s, 13, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s, lv_color_hex(ind), LV_PART_INDICATOR);
    lv_obj_set_style_radius(s, 13, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s, lv_color_hex(knob), LV_PART_KNOB);
    lv_obj_set_style_border_width(s, 3, LV_PART_KNOB);
    lv_obj_set_style_border_color(s, lv_color_hex(COL_INK), LV_PART_KNOB);
    lv_obj_set_style_pad_all(s, 4, LV_PART_KNOB);
    /* The sliders live on a horizontally-swipeable tileview page. Without this, a horizontal drag on
     * the slider chains up to the tileview and flips to the next page instead of moving the slider.
     * Clearing horizontal scroll-chaining makes the slider own the gesture. */
    lv_obj_clear_flag(s, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
}

static void build_controls_page_locked(lv_obj_t *t)
{
    setup_page(t);
    make_caplabel(t, COL_BONE, "LIGHT");

    /* Brightness card (amber): % value, label, slider (evenly spaced) + sun badge top-right. */
    lv_obj_t *bc = make_card(t, COL_AMBER, LV_PCT(100), 152);
    lv_obj_t *bb = make_badge(bc, 40, COL_INK);
    lv_obj_align(bb, LV_ALIGN_TOP_RIGHT, 0, 0);
    badge_icon(bb, &ic_sunny, COL_BONE);
    s_pct_label = make_label(bc, &brutal_44, COL_INK, "100%");
    lv_obj_align(s_pct_label, LV_ALIGN_TOP_LEFT, 0, 2);
    lv_obj_t *bcap = make_label(bc, &lv_font_montserrat_16, COL_INK, "BRIGHTNESS");
    lv_obj_set_style_text_letter_space(bcap, 1, 0);
    lv_obj_set_style_text_opa(bcap, LV_OPA_70, 0);
    lv_obj_align(bcap, LV_ALIGN_TOP_LEFT, 2, 52);
    s_slider = lv_slider_create(bc);
    lv_obj_set_size(s_slider, LV_PCT(90), 24);
    lv_obj_align(s_slider, LV_ALIGN_BOTTOM_MID, 0, -6);
    lv_slider_set_range(s_slider, 1, 100);
    lv_slider_set_value(s_slider, s_pct, LV_ANIM_OFF);
    style_slider(s_slider, COL_INK, COL_BONE);
    lv_obj_add_event_cb(s_slider, slider_event_cb, LV_EVENT_ALL, NULL);

    /* Segmented toggle. */
    lv_obj_t *seg = lv_obj_create(t);
    lv_obj_remove_style_all(seg);
    lv_obj_set_size(seg, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(seg, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(seg, 8, 0);
    lv_obj_set_style_pad_top(seg, CARD_SH, 0);   /* extra gap above buttons to clear the card's drop shadow */
    lv_obj_clear_flag(seg, LV_OBJ_FLAG_SCROLLABLE);
    s_seg_warmth = make_seg_btn(seg, "WARMTH", seg_warmth_event_cb);
    s_seg_colour = make_seg_btn(seg, "COLOUR", seg_colour_event_cb);

    /* Warmth card (coral): kelvin value + gradient slider + sun badge top-right. */
    s_ct_card = make_card(t, COL_CORAL, LV_PCT(100), 152);
    lv_obj_t *cb = make_badge(s_ct_card, 40, COL_INK);
    lv_obj_align(cb, LV_ALIGN_TOP_RIGHT, 0, 0);
    badge_icon(cb, &ic_sunny, COL_BONE);
    s_ct_label = make_label(s_ct_card, &brutal_44, COL_INK, "4000K");
    lv_obj_align(s_ct_label, LV_ALIGN_TOP_LEFT, 0, 2);
    lv_obj_t *ctcap = make_label(s_ct_card, &lv_font_montserrat_16, COL_INK, "COLOUR TEMPERATURE");
    lv_obj_set_style_text_letter_space(ctcap, 1, 0);
    lv_obj_set_style_text_opa(ctcap, LV_OPA_70, 0);
    lv_obj_align(ctcap, LV_ALIGN_TOP_LEFT, 2, 52);
    s_ct_slider = lv_slider_create(s_ct_card);
    lv_obj_set_size(s_ct_slider, LV_PCT(90), 24);
    lv_obj_align(s_ct_slider, LV_ALIGN_BOTTOM_MID, 0, -6);
    lv_slider_set_range(s_ct_slider, CT_KELVIN_MIN, CT_KELVIN_MAX);
    lv_slider_set_value(s_ct_slider, s_ct_kelvin, LV_ANIM_OFF);
    style_slider(s_ct_slider, COL_INK, COL_BONE);
    lv_obj_add_event_cb(s_ct_slider, ct_slider_event_cb, LV_EVENT_ALL, NULL);

    /* Colour card (violet): colour name + swatch + hue strip. */
    s_hue_card = make_card(t, COL_VIOLET, LV_PCT(100), 152);
    s_hue_name = make_label(s_hue_card, &brutal_44, COL_INK, "RED");
    lv_obj_set_style_text_letter_space(s_hue_name, -3, 0);
    lv_obj_align(s_hue_name, LV_ALIGN_TOP_LEFT, 0, 2);
    lv_obj_t *hcap = make_label(s_hue_card, &lv_font_montserrat_16, COL_INK, "COLOUR");
    lv_obj_set_style_text_letter_space(hcap, 1, 0);
    lv_obj_set_style_text_opa(hcap, LV_OPA_70, 0);
    lv_obj_align(hcap, LV_ALIGN_TOP_LEFT, 2, 52);
    s_hue_swatch = lv_obj_create(s_hue_card);
    lv_obj_remove_style_all(s_hue_swatch);
    lv_obj_set_size(s_hue_swatch, 40, 40);
    lv_obj_set_style_bg_opa(s_hue_swatch, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_hue_swatch, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(s_hue_swatch, 3, 0);
    lv_obj_set_style_border_color(s_hue_swatch, lv_color_hex(COL_INK), 0);
    lv_obj_clear_flag(s_hue_swatch, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(s_hue_swatch, LV_ALIGN_TOP_RIGHT, 0, 0);
    s_hue_slider = lv_slider_create(s_hue_card);
    lv_obj_set_size(s_hue_slider, LV_PCT(90), 24);
    lv_obj_align(s_hue_slider, LV_ALIGN_BOTTOM_MID, 0, -6);
    lv_slider_set_range(s_hue_slider, 0, 359);
    lv_slider_set_value(s_hue_slider, s_hue_deg, LV_ANIM_OFF);
    lv_obj_clear_flag(s_hue_slider, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);  /* own the drag; don't swipe the page */
    lv_obj_set_style_bg_opa(s_hue_slider, LV_OPA_TRANSP, LV_PART_INDICATOR);
    /* Rainbow gradient across the track (red → magenta), matching the mockup. */
    static lv_grad_dsc_t hue_grad;
    hue_grad.dir = LV_GRAD_DIR_HOR;
    hue_grad.stops_count = 7;
    static const uint32_t hue_cols[7] = {
        0xE01E1E, 0xE8801E, 0xE8D21E, 0x35C93A, 0x24C7D6, 0x2E5CE6, 0xC838B8
    };
    for (int i = 0; i < 7; i++) {
        hue_grad.stops[i].color = lv_color_hex(hue_cols[i]);
        hue_grad.stops[i].frac  = (uint8_t)(i * 255 / 6);
    }
    lv_obj_set_style_bg_opa(s_hue_slider, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_grad(s_hue_slider, &hue_grad, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_hue_slider, 3, LV_PART_MAIN);
    lv_obj_set_style_border_color(s_hue_slider, lv_color_hex(COL_INK), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_hue_slider, lv_color_hex(COL_BONE), LV_PART_KNOB);
    lv_obj_set_style_border_width(s_hue_slider, 3, LV_PART_KNOB);
    lv_obj_set_style_border_color(s_hue_slider, lv_color_hex(COL_INK), LV_PART_KNOB);
    lv_obj_add_event_cb(s_hue_slider, hue_slider_event_cb, LV_EVENT_ALL, NULL);

    apply_ct_locked();
    apply_hue_locked();
    apply_color_mode_locked();
}

/* ── Indoor page: climate stat grid ─────────────────────────── */
/* A stat card in the 2-col grid: icon badge top-left, big value (returned), caption below. */
static lv_obj_t *make_stat(lv_obj_t *parent, uint32_t bg, const char *cap, const lv_img_dsc_t *icon)
{
    lv_obj_t *c = make_card(parent, bg, LV_PCT(48), 132);
    lv_obj_t *badge = make_badge(c, 40, COL_INK);
    lv_obj_align(badge, LV_ALIGN_TOP_LEFT, 0, 0);
    badge_icon(badge, icon, COL_BONE);
    lv_obj_t *v = make_label(c, &brutal_42, COL_INK, "--");
    lv_obj_align(v, LV_ALIGN_BOTTOM_LEFT, 0, -16);   /* balanced gap: icon -> value -> label */
    lv_obj_t *l = make_label(c, &lv_font_montserrat_16, COL_INK, cap);
    lv_obj_set_style_text_letter_space(l, 1, 0);
    lv_obj_set_style_text_opa(l, LV_OPA_70, 0);
    lv_obj_align(l, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    return v;
}

static void build_indoor_page_locked(lv_obj_t *t)
{
    setup_page(t);
    make_caplabel(t, COL_BONE, "INDOOR");

    lv_obj_t *grid = lv_obj_create(t);
    lv_obj_remove_style_all(grid);
    lv_obj_set_size(grid, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_row(grid, 11, 0);
    lv_obj_set_style_pad_column(grid, 11, 0);
    lv_obj_set_flex_align(grid, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(grid, LV_OBJ_FLAG_OVERFLOW_VISIBLE);   /* don't clip stat-card hard-shadows */

    s_temp_label     = make_stat(grid, COL_CORAL,  "TEMP",     &ic_temp);
    s_humidity_label = make_stat(grid, COL_SKY,    "HUMIDITY", &ic_humidity);
    s_co2_label      = make_stat(grid, COL_VIOLET, "CO2",      &ic_co2);
    s_pm25_label     = make_stat(grid, COL_AMBER,  "PM2.5",    &ic_pm25);

    /* Air quality (wide) — bg recoloured by rating; leaf badge + big value. */
    s_aq_card = make_card(grid, COL_LIME, LV_PCT(100), 100);

    /* Badge pinned to the left; label+value stacked and centred horizontally in the card. */
    lv_obj_t *abadge = make_badge(s_aq_card, 48, COL_INK);
    lv_obj_align(abadge, LV_ALIGN_LEFT_MID, 0, -3);
    badge_icon(abadge, &ic_air, COL_BONE);

    lv_obj_t *tcol = lv_obj_create(s_aq_card);
    lv_obj_remove_style_all(tcol);
    lv_obj_set_size(tcol, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(tcol, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(tcol, 2, 0);
    lv_obj_clear_flag(tcol, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(tcol, LV_ALIGN_LEFT_MID, 60, -1);   /* just right of the badge, left-aligned */
    lv_obj_t *acap = make_label(tcol, &lv_font_montserrat_16, COL_INK, "AIR QUALITY");
    lv_obj_set_style_text_letter_space(acap, 1, 0);
    lv_obj_set_style_text_opa(acap, LV_OPA_70, 0);
    lv_obj_set_style_translate_x(acap, 4, 0);   /* nudge the label right relative to the value */
    s_aq_label = make_label(tcol, &brutal_42, COL_INK, "--");
    lv_obj_set_style_text_letter_space(s_aq_label, -3, 0);

    apply_climate_locked();
    apply_airquality_locked();
}

static void build_ui_locked(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), LV_PART_MAIN);

    /* Three pages: Controls (left) · Home (center) · Indoor (right). */
    s_tileview = lv_tileview_create(scr);
    lv_obj_set_style_bg_color(s_tileview, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(s_tileview, LV_SCROLLBAR_MODE_OFF);

    /* Row 0: Controls · Home · Indoor. Below Home (row 1): Settings, reached by swiping up on the
     * Home page (Home allows a downward move; Settings allows an upward move back). */
    lv_obj_t *controls_tile = lv_tileview_add_tile(s_tileview, 0, 0, LV_DIR_HOR);
    s_home_tile             = lv_tileview_add_tile(s_tileview, 1, 0, LV_DIR_HOR | LV_DIR_BOTTOM);  /* Home */
    lv_obj_t *indoor_tile   = lv_tileview_add_tile(s_tileview, 2, 0, LV_DIR_HOR);
    lv_obj_t *settings_tile = lv_tileview_add_tile(s_tileview, 1, 1, LV_DIR_TOP);

    build_controls_page_locked(controls_tile);
    build_home_page_locked(s_home_tile);
    build_indoor_page_locked(indoor_tile);
    build_settings_page_locked(settings_tile);
    lv_obj_set_tile(s_tileview, s_home_tile, LV_ANIM_OFF);

    build_boot_screen_locked(scr);
    build_pairing_screen_locked(scr);
    build_nobind_screen_locked(scr);
    build_devices_screen_locked(scr);
    build_timeset_screen_locked(scr);
    build_alert_overlay_locked();
    build_confirm_overlay_locked();

    apply_mode_locked();
}

/* ── LVGL task ──────────────────────────────────────────────── */

/* Put the display to sleep: stop driving the UI and cut the AMOLED power rail.
 * Runs on the LVGL task, outside the LVGL lock (touches the panel, not widgets). */
/* ── Low power ─────────────────────────────────────────────────────────────────────────────────
 * A no-light-sleep lock is held the whole time the display is on, so the panel, touch and UI are
 * never interrupted by a sleep; releasing it when the screen sleeps is what lets the SoC light-sleep
 * between Thread polls. Without CONFIG_PM_ENABLE all of this compiles away. */
static esp_pm_lock_handle_t s_pm_lock = NULL;
static bool                 s_pm_held = false;

#if CONFIG_PM_ENABLE
/* Touch-to-wake while the screen is off. The asleep poll is backed off to 100 ms so the SoC can
 * light-sleep, which is far too slow to catch a quick tap — the FT3168 asserts its INT line only
 * briefly. So while the display is off the INT line is an interrupt rather than something we poll:
 * it wakes the SoC out of light sleep and notifies the LVGL task straight away.
 *
 * The trigger must be a level, not an edge (gpio_wakeup_enable() rejects edges, and sets the
 * interrupt type as a side effect), so the ISR masks itself to avoid re-entering while the line is
 * held low, and the task re-arms it once the line is released. */
static TaskHandle_t s_lvgl_task_h = NULL;

static void IRAM_ATTR touch_int_isr(void *arg)
{
    (void)arg;
    gpio_intr_disable((gpio_num_t)AMOLED_TOUCH_INT_GPIO);
    BaseType_t hp = pdFALSE;
    if (s_lvgl_task_h) {
        vTaskNotifyGiveFromISR(s_lvgl_task_h, &hp);
    }
    portYIELD_FROM_ISR(hp);
}

static void touch_wake_init(void)
{
    const gpio_num_t pin = (gpio_num_t)AMOLED_TOUCH_INT_GPIO;
    /* Light-sleep wake source (also sets the interrupt type to low-level). */
    if (gpio_wakeup_enable(pin, GPIO_INTR_LOW_LEVEL) != ESP_OK ||
        esp_sleep_enable_gpio_wakeup() != ESP_OK) {
        ESP_LOGW(TAG, "Touch wake source unavailable — falling back to polling");
    }
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {   /* already installed is fine */
        ESP_LOGW(TAG, "gpio_install_isr_service: %s", esp_err_to_name(err));
        return;
    }
    if (gpio_isr_handler_add(pin, touch_int_isr, NULL) == ESP_OK) {
        gpio_intr_disable(pin);   /* armed only while the display is asleep */
    }
}
#endif  /* CONFIG_PM_ENABLE */

/* Arm (or disarm) the touch interrupt. Only meaningful in the low-power build. */
static void touch_wake_arm(bool arm)
{
#if CONFIG_PM_ENABLE
    const gpio_num_t pin = (gpio_num_t)AMOLED_TOUCH_INT_GPIO;
    if (arm) {
        gpio_intr_enable(pin);
    } else {
        gpio_intr_disable(pin);
    }
#else
    (void)arm;
#endif
}

static void display_pm_hold(bool hold)
{
#if CONFIG_PM_ENABLE
    if (!s_pm_lock) {
        return;
    }
    if (hold && !s_pm_held) {
        esp_pm_lock_acquire(s_pm_lock);
        s_pm_held = true;
    } else if (!hold && s_pm_held) {
        esp_pm_lock_release(s_pm_lock);
        s_pm_held = false;
    }
#else
    (void)hold;
#endif
}

static void enter_sleep(void)
{
    ESP_LOGI(TAG, "Idle timeout -> display sleep");
    amoled_lvgl_set_touch_enabled(false);  /* ignore touches as UI input while off */
    amoled_sleep();                         /* panel off + cut AMOLED power (P4 low) */
    s_awake = false;
    touch_wake_arm(true);                   /* a tap now interrupts instead of waiting for a poll */
    display_pm_hold(false);                 /* the SoC may now light-sleep between Thread polls */
}

/* Wake the display: restore power, re-init the panel, and force a redraw. */
static void exit_sleep(void)
{
    ESP_LOGI(TAG, "Touch -> display wake");
    touch_wake_arm(false);         /* back to polling; LVGL reads the panel directly while awake */
    display_pm_hold(true);         /* full clocks before touching the panel */
    amoled_wake();                 /* restore power + re-init the SH8601 */
    amoled_set_brightness(disp_level_from_pct(s_disp_bright_pct));
    s_awake = true;
    s_need_redraw = true;          /* GRAM was lost — repaint on the next handler pass */
    s_consume_touch = true;        /* don't let the wake touch fall through to a control */
}

/* Stash the current wall clock into RTC_NOINIT memory (survives the soft reset) and restart, so the
 * clock the user set is still correct after the reboot. Restored in light_ui_start(). Does not
 * return. Runs on the LVGL task, outside the LVGL lock. */
static void do_reboot(void)
{
    time_t now = time(NULL);
    if (now > 1700000000) {
        s_rtc_time_utc   = now;
        s_rtc_time_magic = RTC_TIME_MAGIC;
        ESP_LOGI(TAG, "Rebooting — clock stashed (%lld)", (long long)now);
    } else {
        s_rtc_time_magic = 0;
        ESP_LOGI(TAG, "Rebooting — no valid clock to stash");
    }
    esp_restart();
}

/* FreeRTOS idle hook: runs on the idle task, so its call rate tracks how much CPU is going unused.
 * During the boot CASE/Thread storm the idle task barely runs; once it settles this ramps back up.
 * Must be cheap and must return true (lets the idle task still tick-sleep normally). */
static bool ui_idle_hook(void)
{
    s_idle_ticks++;
    return true;
}

static void lvgl_task(void *arg)
{
    LV_UNUSED(arg);
    ESP_LOGI(TAG, "LVGL task started");
    s_last_active_ms = esp_timer_get_time() / 1000;

    /* Boot-screen dismissal state (only touched by this task). */
    int64_t  boot_t0_ms        = s_last_active_ms;
    int64_t  boot_idle_last_ms = s_last_active_ms;
    uint32_t boot_idle_prev    = 0;
    uint32_t boot_idle_peak    = 0;
    uint32_t boot_idle_quiet   = 0;

    while (true) {
        const int64_t now_ms  = esp_timer_get_time() / 1000;
        const bool    touched = (gpio_get_level((gpio_num_t)AMOLED_TOUCH_INT_GPIO) == 0);
        if (!s_awake && !touched) {
            /* The ISR masks itself when it fires; re-arm once the line is released, otherwise a
             * spurious pulse would leave the display unwakeable. */
            touch_wake_arm(true);
        }
        const bool    pairing = (s_mode == LIGHT_UI_MODE_PAIRING);
        const bool    booting = (s_mode == LIGHT_UI_MODE_BOOTING);

        /* ---- Boot screen: leave for the settled UI once the startup storm has passed ---- */
        if (booting && s_boot_target_valid) {
            const int64_t elapsed = now_ms - boot_t0_ms;
            bool leave = false;
            const char *why = "idle";
            if (elapsed >= BOOT_MAX_MS) {
                leave = true;
                why = "cap";
            } else if (now_ms - boot_idle_last_ms >= BOOT_IDLE_SAMPLE_MS) {
                const uint32_t cur   = s_idle_ticks;
                const uint32_t delta = cur - boot_idle_prev;
                boot_idle_prev    = cur;
                boot_idle_last_ms = now_ms;
                if (delta > boot_idle_peak) {
                    boot_idle_peak = delta;   /* auto-calibrate "fully idle" from the calmest window */
                }
                /* Quiet = idle rate back near its peak (CPU no longer hogged by the storm). */
                if (boot_idle_peak > 0 && delta * 100 >= boot_idle_peak * BOOT_IDLE_FRAC_PCT) {
                    boot_idle_quiet++;
                } else {
                    boot_idle_quiet = 0;
                }
                if (elapsed >= BOOT_MIN_MS && boot_idle_quiet >= BOOT_QUIET_SAMPLES) {
                    leave = true;
                }
            }
            if (leave) {
                LVGL_LOCK();
                s_mode = s_boot_target;
                s_boot_target_valid = false;
                ESP_LOGI(TAG, "Boot settled after %lld ms (%s) -> UI mode %d",
                         (long long)elapsed, why, (int)s_mode);
                apply_mode_locked();
                LVGL_UNLOCK();
                s_need_redraw = true;
            }
        }

        /* ---- Display power state machine (no LVGL lock needed) ---- */
        if (!s_power_save_enabled) {
            /* Power management is OFF (not fully commissioned, or a commissioning
             * window is open). Keep the display on and never cycle the panel
             * power, so none of this can affect commissioning timing. */
            if (!s_awake) {
                exit_sleep();
                amoled_lvgl_set_touch_enabled(true);
                s_consume_touch = false;
            }
            s_force_wake = false;
            s_last_active_ms = now_ms;
        } else if (s_awake) {
            /* Touch, an in-progress wake, pairing (QR must stay visible), the boot screen, or a
             * visible alert all count as "active" and keep the screen on. */
            if (touched || pairing || booting || s_consume_touch || s_alert_visible) {
                s_last_active_ms = now_ms;
            }
            if (!pairing && !booting && (now_ms - s_last_active_ms >= s_idle_timeout_ms)) {
                enter_sleep();
            }
        } else if (touched || s_force_wake) {
            s_force_wake = false;
            exit_sleep();
            s_last_active_ms = now_ms;
        }
        /* Re-enable UI touch only once the wake touch is released. */
        if (s_awake && s_consume_touch && !touched) {
            amoled_lvgl_set_touch_enabled(true);
            s_consume_touch = false;
            s_last_active_ms = now_ms;
        }

        /* ---- LVGL — only while awake (never draw to an unpowered panel) ---- */
        if (s_awake) {
            /* Keep the ICD out of idle mode while the screen is on, so attribute reports (and so
             * the on-screen state) stay live. Rate-limited: one nudge a second is plenty. */
            if (now_ms - s_last_icd_ms >= 1000) {
                s_last_icd_ms = now_ms;
                app_driver_notify_active();
            }
            int  send_level = -1;
            int  send_ct_kelvin = -1;
            int  send_hue_deg = -1;
            int  send_onoff = -1;   /* -1 = none, 0 = Off, 1 = On */
            bool do_reboot_now  = false;
            bool do_factory_now = false;
            dev_toggle_t dev_out[8];
            int          dev_n = 0;

            LVGL_LOCK();
            if (s_need_redraw) {
                lv_obj_invalidate(lv_scr_act());
                s_need_redraw = false;
            }
            lv_timer_handler();
            /* Latch pending work while holding the lock, act on it after releasing. */
            if (s_onoff_pending)  { s_onoff_pending  = false; send_onoff = s_onoff_on ? 1 : 0; }
            /* Debounced slider sends: fire one command once the value has settled for the debounce. */
            if (s_level_dirty_ms && now_ms - s_level_dirty_ms >= SLIDER_DEBOUNCE_MS) { s_level_dirty_ms = 0; send_level = s_pct; }
            if (s_ct_dirty_ms    && now_ms - s_ct_dirty_ms    >= SLIDER_DEBOUNCE_MS) { s_ct_dirty_ms = 0;    send_ct_kelvin = s_ct_kelvin; }
            if (s_hue_dirty_ms   && now_ms - s_hue_dirty_ms   >= SLIDER_DEBOUNCE_MS) { s_hue_dirty_ms = 0;   send_hue_deg = s_hue_deg; }
            if (s_reboot_pending)  { s_reboot_pending  = false; do_reboot_now  = true; }
            if (s_factory_pending) { s_factory_pending = false; do_factory_now = true; }
            for (int i = 0; i < s_dev_toggle_n; i++) dev_out[i] = s_dev_toggle_q[i];
            dev_n = s_dev_toggle_n;
            s_dev_toggle_n = 0;
            LVGL_UNLOCK();

            /* Terminal actions (outside the lock — neither returns). */
            if (do_factory_now) {
                app_driver_factory_reset();   /* erases NVS + restarts */
            }
            if (do_reboot_now) {
                do_reboot();                  /* stash clock + esp_restart */
            }

            /* Send Matter commands outside the LVGL lock to keep the
             * LVGL-mutex / CHIP-stack-lock ordering consistent (see top comment). */
            if (send_onoff >= 0) {
                app_driver_send_onoff(send_onoff != 0);
            }
            if (send_level >= 0) {
                app_driver_send_level(pct_to_level(send_level));
            }
            if (send_ct_kelvin >= 0) {
                app_driver_send_color_temp(kelvin_to_mireds(send_ct_kelvin));
            }
            if (send_hue_deg >= 0) {
                /* Matter Hue is 0-254 over 0-360°; drive at full saturation. */
                app_driver_send_color_hue_sat((uint8_t)(send_hue_deg * 254 / 359), 254);
            }
            for (int i = 0; i < dev_n; i++) {
                app_driver_send_toggle_node(dev_out[i].fabric, dev_out[i].node, dev_out[i].ep);
            }
        }

#if CONFIG_PM_ENABLE
        /* A 10 ms poll never lets the tickless idle sleep long enough to be worth entering, so back
         * right off once the screen is asleep. The touch ISR notifies this task, so a tap is acted
         * on immediately rather than at the end of the interval. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(s_awake ? 10 : 100));
#else
        vTaskDelay(pdMS_TO_TICKS(10));
#endif
    }
}

/* ── Public API ─────────────────────────────────────────────── */

esp_err_t light_ui_start(const char *qr_payload, const char *manual_code, bool commissioned)
{
    /* Draw buffers are sized against this reserve (see amoled_lvgl_set_dma_reserve). An
     * uncommissioned device is about to run the Matter commissioning handshake with BLE still
     * resident, which needs far more heap than steady-state operation, so trade buffer size for
     * headroom until it is paired. */
    amoled_lvgl_set_dma_reserve(commissioned ? (40 * 1024) : (72 * 1024));

#if CONFIG_PM_ENABLE
    /* Created before the display comes up, and held from here on, so bring-up runs at full clock. */
    esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "display_on", &s_pm_lock);
#endif
    display_pm_hold(true);

    if (qr_payload) {
        strncpy(s_qr_payload, qr_payload, sizeof(s_qr_payload) - 1);
    }
    if (manual_code) {
        strncpy(s_manual_code, manual_code, sizeof(s_manual_code) - 1);
    }

    s_lvgl_mtx = xSemaphoreCreateRecursiveMutex();
    if (!s_lvgl_mtx) {
        ESP_LOGE(TAG, "Failed to create LVGL mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Seed the clock offset from the last zone the controller pushed, so it's correct on boot
     * (the controller doesn't re-send the timezone every reboot). Set before the UI is built. */
    int32_t saved_off;
    if (tz_offset_load(&saved_off)) {
        s_utc_offset_s = saved_off;
        ESP_LOGI(TAG, "Loaded saved UTC offset: %ld s", (long)saved_off);
    }
    bool auto_on;
    if (time_auto_load(&auto_on)) {
        s_time_auto = auto_on;
        ESP_LOGI(TAG, "Loaded time auto-sync: %s", auto_on ? "ON" : "OFF");
    }

    /* Load the Settings-page display prefs (backlight %, idle-off seconds), clamped to valid ranges. */
    int saved_bright;
    if (disp_bright_load(&saved_bright)) {
        if (saved_bright < DISP_BRIGHT_MIN_PCT) saved_bright = DISP_BRIGHT_MIN_PCT;
        if (saved_bright > 100) saved_bright = 100;
        s_disp_bright_pct = saved_bright;
        ESP_LOGI(TAG, "Loaded display brightness: %d%%", s_disp_bright_pct);
    }
    int saved_idle;
    if (idle_timeout_load(&saved_idle)) {
        if (saved_idle < LIGHT_UI_IDLE_TIMEOUT_MIN_S) saved_idle = LIGHT_UI_IDLE_TIMEOUT_MIN_S;
        if (saved_idle > LIGHT_UI_IDLE_TIMEOUT_MAX_S) saved_idle = LIGHT_UI_IDLE_TIMEOUT_MAX_S;
        s_idle_timeout_ms = (int64_t)saved_idle * 1000;
        ESP_LOGI(TAG, "Loaded display idle-off: %d s", saved_idle);
    }

    /* Restore a clock stashed by a UI-initiated reboot, so the wall time survives the restart. Must
     * run before the neutral-default block below (otherwise a soft reboot would look like a cold one).
     * The magic is only valid after a soft reset; a cold power-on leaves RTC_NOINIT garbage, which
     * the magic check rejects, and we fall through to the default clock. */
    if (s_rtc_time_magic == RTC_TIME_MAGIC && s_rtc_time_utc > 1700000000) {
        struct timeval tv; tv.tv_sec = s_rtc_time_utc; tv.tv_usec = 0;
        settimeofday(&tv, NULL);
        ESP_LOGI(TAG, "Restored stashed clock across reboot (%lld)", (long long)s_rtc_time_utc);
    }
    s_rtc_time_magic = 0;   /* consume it so a later cold boot can't reuse a stale value */

    /* If the RTC has no real time yet (cold boot → 1970), start the clock at a neutral default
     * (00:00, 1 Jan 2026) so the card shows a live, ticking clock instead of "SYNCING". Store it so
     * the displayed local wall time reads 00:00 01 Jan 2026 regardless of any saved offset; a
     * controller push (auto-sync on) or a manual entry overrides it. */
    if (time(NULL) < 1700000000) {
        time_t def_local = utc_from_ymd_hms(2026, 1, 1, 0, 0, 0);
        struct timeval tv; tv.tv_sec = def_local - s_utc_offset_s; tv.tv_usec = 0;
        settimeofday(&tv, NULL);
        ESP_LOGI(TAG, "No synced time at boot — starting clock at 00:00 01 Jan 2026");
    }

    ESP_RETURN_ON_ERROR(amoled_init(), TAG, "amoled_init");

    esp_lcd_touch_handle_t touch = NULL;
    if (amoled_touch_init(&touch) != ESP_OK) {
        ESP_LOGW(TAG, "Touch init failed — continuing without touch");
        touch = NULL;
    }

    ESP_RETURN_ON_ERROR(amoled_lvgl_init(amoled_get_panel(), touch), TAG, "amoled_lvgl_init");

    LVGL_LOCK();
    build_ui_locked();
    LVGL_UNLOCK();

    amoled_set_brightness(disp_level_from_pct(s_disp_bright_pct));

    /* Idle hook feeds the boot-screen dismissal (CPU-idle detection). Best-effort. */
    esp_register_freertos_idle_hook(ui_idle_hook);

#if CONFIG_PM_ENABLE
    touch_wake_init();
#endif
    if (xTaskCreate(lvgl_task, "lvgl", 8192, NULL, 2,
#if CONFIG_PM_ENABLE
                    &s_lvgl_task_h
#else
                    NULL
#endif
                    ) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LVGL task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Light UI ready");
    return ESP_OK;
}

void light_ui_set_state(bool on)
{
    if (!s_lvgl_mtx) {
        return; /* UI not started yet */
    }
    LVGL_LOCK();
    /* Once per-device OnOff data exists, the master toggle follows the aggregate (any on = on),
     * not this single light's state. Only fall back to it when no devices are known yet. */
    if (ui_has_any_device()) {
        recompute_master_locked();
    } else {
        s_state = on;
        apply_state_locked();
    }
    LVGL_UNLOCK();
}

void light_ui_set_brightness(uint8_t level)
{
    if (!s_lvgl_mtx) {
        return; /* UI not started yet */
    }
    LVGL_LOCK();
    /* Don't fight the user while they are dragging the slider, nor while a debounced send for the
     * value they just set is still pending (else a stale report would clobber s_pct before we send). */
    if (!s_dragging && s_level_dirty_ms == 0) {
        s_pct = level_to_pct(level);
        if (s_slider) {
            lv_slider_set_value(s_slider, s_pct, LV_ANIM_OFF);
        }
        apply_pct_label_locked();
    }
    LVGL_UNLOCK();
}

void light_ui_set_color_temp(uint16_t mireds)
{
    if (!s_lvgl_mtx) {
        return; /* UI not started yet */
    }
    LVGL_LOCK();
    /* Don't fight the user while they are dragging the CT slider, nor while a debounced send is pending. */
    if (!s_ct_dragging && s_ct_dirty_ms == 0) {
        s_ct_kelvin = mireds_to_kelvin(mireds);
        if (s_ct_slider) {
            lv_slider_set_value(s_ct_slider, s_ct_kelvin, LV_ANIM_OFF);
        }
        apply_ct_locked();
    }
    LVGL_UNLOCK();
}

void light_ui_set_temperature(float celsius)
{
    if (!s_lvgl_mtx) {
        return; /* UI not started yet */
    }
    LVGL_LOCK();
    s_temp_c = celsius;
    s_have_temp = true;
    apply_climate_locked();
    LVGL_UNLOCK();
}

void light_ui_set_humidity(float percent)
{
    if (!s_lvgl_mtx) {
        return; /* UI not started yet */
    }
    LVGL_LOCK();
    s_humidity_pct = percent;
    s_have_humidity = true;
    apply_climate_locked();
    LVGL_UNLOCK();
}

void light_ui_set_air_quality(int level)
{
    if (!s_lvgl_mtx) {
        return; /* UI not started yet */
    }
    LVGL_LOCK();
    s_aq_level = level;
    apply_airquality_locked();
    LVGL_UNLOCK();
}

void light_ui_set_co2(float ppm)
{
    if (!s_lvgl_mtx) {
        return; /* UI not started yet */
    }
    LVGL_LOCK();
    s_co2_ppm = ppm;
    s_have_co2 = true;
    apply_airquality_locked();
    LVGL_UNLOCK();
}

void light_ui_set_pm25(float ugm3)
{
    if (!s_lvgl_mtx) {
        return; /* UI not started yet */
    }
    LVGL_LOCK();
    s_pm25_ugm3 = ugm3;
    s_have_pm25 = true;
    apply_airquality_locked();
    LVGL_UNLOCK();
}

void light_ui_set_weather_min(float celsius)
{
    if (!s_lvgl_mtx) return;
    LVGL_LOCK();
    s_wx_min_c = celsius;
    s_have_wx_min = true;
    apply_weather_locked();
    LVGL_UNLOCK();
}

void light_ui_set_weather_max(float celsius)
{
    if (!s_lvgl_mtx) return;
    LVGL_LOCK();
    s_wx_max_c = celsius;
    s_have_wx_max = true;
    apply_weather_locked();
    LVGL_UNLOCK();
}

void light_ui_set_weather_current(float celsius)
{
    if (!s_lvgl_mtx) return;
    LVGL_LOCK();
    s_wx_now_c = celsius;
    s_have_wx_now = true;
    apply_weather_locked();
    LVGL_UNLOCK();
}

void light_ui_set_weather_condition(const char *condition)
{
    if (!s_lvgl_mtx || !condition) return;
    LVGL_LOCK();
    strncpy(s_wx_condition, condition, sizeof(s_wx_condition) - 1);
    s_wx_condition[sizeof(s_wx_condition) - 1] = '\0';
    apply_weather_locked();
    LVGL_UNLOCK();
}

void light_ui_set_weather_location(const char *location)
{
    if (!s_lvgl_mtx || !location) return;
    LVGL_LOCK();
    strncpy(s_wx_location, location, sizeof(s_wx_location) - 1);
    s_wx_location[sizeof(s_wx_location) - 1] = '\0';
    apply_weather_locked();
    LVGL_UNLOCK();
}

void light_ui_set_utc_offset(int32_t offset_seconds)
{
    if (!s_lvgl_mtx) return;
    /* Manual mode: ignore controller timezone changes so the wall time the user set doesn't shift. */
    if (!s_time_auto) return;
    LVGL_LOCK();
    bool changed = (offset_seconds != s_utc_offset_s);
    s_utc_offset_s = offset_seconds;
    update_clock_locked();
    LVGL_UNLOCK();
    /* Persist across reboots (outside the LVGL lock; NVS is thread-safe). */
    if (changed) {
        tz_offset_save(offset_seconds);
    }
}

bool light_ui_time_auto_enabled(void)
{
    return s_time_auto;
}

void light_ui_reassert_manual_time(void)
{
    if (!s_lvgl_mtx || !s_have_manual) return;
    /* Reconstruct the manual wall time from its base + the monotonic uptime elapsed since, and
     * re-apply it — used to undo a controller SetUTCTime that landed while auto-sync is off. */
    int64_t elapsed_us = esp_timer_get_time() - s_manual_uptime_base_us;
    time_t utc = s_manual_utc_base + (time_t)(elapsed_us / 1000000);
    struct timeval tv; tv.tv_sec = utc; tv.tv_usec = 0;
    settimeofday(&tv, NULL);
    LVGL_LOCK();
    update_clock_locked();
    LVGL_UNLOCK();
    ESP_LOGI(TAG, "Re-applied manual time (auto-sync off); ignored controller push");
}

void light_ui_set_device_state(uint8_t fabric_index, uint64_t node_id, uint16_t endpoint, bool on)
{
    if (!s_lvgl_mtx) return;
    LVGL_LOCK();
    ui_device_t *d = ui_device_find(node_id, endpoint, true);
    if (d) {
        d->fabric = fabric_index;
        bool changed = (!d->have_state) || (d->on != on);
        d->have_state = true;
        d->on = on;
        if (changed) rebuild_devices_grid_locked();
        recompute_master_locked();   /* master toggle = ON if any device on, OFF only if all off */
    }
    LVGL_UNLOCK();
}

void light_ui_set_device_kind(uint64_t node_id, uint16_t endpoint, int is_plug)
{
    if (!s_lvgl_mtx) return;
    LVGL_LOCK();
    ui_device_t *d = ui_device_find(node_id, endpoint, true);
    if (d) {
        int k = is_plug ? DEV_KIND_PLUG : DEV_KIND_LIGHT;
        if (d->kind != k) {
            d->kind = k;
            if (d->have_state) rebuild_devices_grid_locked();   /* only visible cards need a redraw */
        }
    }
    LVGL_UNLOCK();
}

void light_ui_show_alert(const char *title, const char *subtitle, bool critical)
{
    if (!s_lvgl_mtx) return;
    LVGL_LOCK();
    if (s_alert_overlay && s_alert_card) {
        /* Colour the card (and its hard shadow): red for a critical alarm (water leak),
         * amber for a warning (contact open). */
        card_set_color(s_alert_card, critical ? COL_ALERT : COL_AMBER);
        lv_label_set_text(s_alert_title, title ? title : "");
        lv_label_set_text(s_alert_sub, subtitle ? subtitle : "");
        lv_obj_move_foreground(s_alert_overlay);
        lv_obj_clear_flag(s_alert_overlay, LV_OBJ_FLAG_HIDDEN);
        if (s_alert_timer) {
            lv_timer_reset(s_alert_timer);   /* restart the auto-dismiss countdown */
            lv_timer_resume(s_alert_timer);
        }
        s_alert_visible = true;
    }
    LVGL_UNLOCK();
    /* Wake the panel if it is asleep so the alert is actually seen (handled by the LVGL task). */
    s_force_wake = true;
}

void light_ui_set_mode(light_ui_mode_t mode)
{
    if (!s_lvgl_mtx) {
        return; /* UI not started yet */
    }
    LVGL_LOCK();
    /* While still on the boot screen, DEFER a switch to a settled screen (READY / NO_BINDING):
     * record it as the target and keep the boot screen up until the LVGL task sees the startup
     * storm end (or the cap). PAIRING is honoured immediately — an uncommissioned device has
     * nothing to wait for and should show its QR at once. */
    if (s_mode == LIGHT_UI_MODE_BOOTING && mode != LIGHT_UI_MODE_PAIRING) {
        if (mode != LIGHT_UI_MODE_BOOTING) {
            s_boot_target = mode;
            s_boot_target_valid = true;
        }
        LVGL_UNLOCK();
        return;
    }
    if (s_mode != mode) {
        ESP_LOGI(TAG, "UI mode -> %d", (int)mode);
    }
    s_mode = mode;
    apply_mode_locked();
    LVGL_UNLOCK();

    /* Make sure the QR is actually visible when we (re)enter pairing, even if
     * the screen had gone to sleep. */
    if (mode == LIGHT_UI_MODE_PAIRING) {
        s_force_wake = true;
    }
}

void light_ui_set_power_save_enabled(bool enable)
{
    if (s_power_save_enabled == enable) {
        return;
    }
    s_power_save_enabled = enable;
    ESP_LOGI(TAG, "Display power management %s", enable ? "enabled" : "disabled");
    if (!enable) {
        /* Leaving power-save: make sure the screen comes back on. The LVGL task
         * wakes it on its next pass (the !s_power_save_enabled branch). */
        s_force_wake = true;
    }
}
