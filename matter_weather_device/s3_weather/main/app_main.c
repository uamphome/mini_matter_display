/*
   ESP32-S3 weather front-end for the Thread Border Router board.

   Owns Wi-Fi (its own radio — no coexistence), geocodes the user's location by
   name and fetches the forecast from Open-Meteo, then streams the readings to the
   on-board ESP32-H2 (the Matter/Thread device) over the inter-chip UART.

   Wi-Fi credentials, location, units and refresh interval are entered over USB
   serial and persisted to NVS — either in the full-screen arrow-key `setup` page,
   or with the one-per-line commands (`wifi`, `location`, …) that also work in
   browser serial consoles, which can only send whole lines.

   Line protocol (this side is the sender):
     S3 -> H2:  WX <min> <max> <cur> <condition…>   °C or °F per unit, "-" = null
                LOC <location…>
     H2 -> S3:  READY | NEED_WEATHER
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <math.h>

#include "cJSON.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_console.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

static const char *TAG = "s3_weather";

// ── Inter-chip UART to the H2 ────────────────────────────────────────────────
// Per the ESP Thread Border Router board schematic (UART0 interface):
// H2_RXD0 <- S3_GPIO18, H2_TXD0 -> S3_GPIO17.
#define H2_UART_PORT   UART_NUM_1
#define H2_UART_TX_PIN 18     // S3 TX (GPIO18) -> H2_RXD0
#define H2_UART_RX_PIN 17     // S3 RX (GPIO17) <- H2_TXD0
#define H2_UART_BAUD   115200

// ── Persisted config (whole struct stored as one NVS blob) ───────────────────
typedef struct {
    char     ssid[33];
    char     pass[65];
    char     location[40];   // display name (also what LOC sends)
    double   lat, lon;
    bool     have_geocode;
    bool     fahrenheit;
    uint32_t interval_s;
} cfg_t;

static cfg_t cfg = { .interval_s = 3600 };

static bool cfg_ready(void) { return cfg.ssid[0] && cfg.have_geocode; }

static void cfg_load(void)
{
    nvs_handle_t h;
    if (nvs_open("weather", NVS_READONLY, &h) != ESP_OK) return;
    size_t n = sizeof(cfg);
    cfg_t tmp;
    if (nvs_get_blob(h, "cfg", &tmp, &n) == ESP_OK && n == sizeof(cfg)) cfg = tmp;
    if (cfg.interval_s < 60) cfg.interval_s = 3600;
    nvs_close(h);
}

static void cfg_save(void)
{
    nvs_handle_t h;
    if (nvs_open("weather", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "cfg", &cfg, sizeof(cfg));
    nvs_commit(h);
    nvs_close(h);
}

// ── Last reading cache (for resend on NEED_WEATHER) ──────────────────────────
static float s_tmin = NAN, s_tmax = NAN, s_tcur = NAN;
static char  s_cond[24] = "unknown";
static bool  s_have_reading = false;

// ── Wi-Fi (station, always on with auto-reconnect) ───────────────────────────
#define WIFI_GOT_IP BIT0
static EventGroupHandle_t s_wifi_events;
static volatile bool s_online = false;
static TaskHandle_t s_weather_task = NULL;
static void weather_trigger(void);

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_online = false;
        xEventGroupClearBits(s_wifi_events, WIFI_GOT_IP);
        esp_wifi_connect();   // keep trying
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_online = true;
        xEventGroupSetBits(s_wifi_events, WIFI_GOT_IP);
        ESP_LOGI(TAG, "Wi-Fi online");
        weather_trigger();
    }
}

static void wifi_start(void)
{
    if (!cfg.ssid[0]) return;
    wifi_config_t wc = {};
    strncpy((char *) wc.sta.ssid, cfg.ssid, sizeof(wc.sta.ssid) - 1);
    strncpy((char *) wc.sta.password, cfg.pass, sizeof(wc.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void wifi_restart(void)
{
    esp_wifi_disconnect();
    esp_wifi_stop();
    wifi_start();
}

static void wifi_init(void)
{
    s_wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));   // TLS stability
}

// ── HTTPS GET → buffer ───────────────────────────────────────────────────────
typedef struct { char *data; int len; int cap; } http_resp_t;

static esp_err_t http_ev(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->user_data) {
        http_resp_t *r = (http_resp_t *) evt->user_data;
        int n = evt->data_len;
        if (r->len + n < r->cap - 1) { memcpy(r->data + r->len, evt->data, n); r->len += n; }
    }
    return ESP_OK;
}

static int http_get_json(const char *url, char *buf, int cap)
{
    http_resp_t resp = { buf, 0, cap };
    esp_http_client_config_t config = {};
    config.url = url;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.timeout_ms = 15000;
    config.event_handler = http_ev;
    config.user_data = &resp;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || status != 200) {
        ESP_LOGW(TAG, "HTTP GET failed: err=%s status=%d", esp_err_to_name(err), status);
        return -1;
    }
    buf[resp.len] = '\0';
    return resp.len;
}

static void url_encode(char *dst, size_t cap, const char *src)
{
    static const char *hex = "0123456789ABCDEF";
    size_t j = 0;
    for (size_t i = 0; src[i] && j + 4 < cap; i++) {
        unsigned char c = (unsigned char) src[i];
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
            dst[j++] = (char) c;
        } else {
            dst[j++] = '%'; dst[j++] = hex[c >> 4]; dst[j++] = hex[c & 0x0F];
        }
    }
    dst[j] = '\0';
}

// ── UART link to the H2 ──────────────────────────────────────────────────────
static void h2_uart_init(void)
{
    uart_config_t uc = {
        .baud_rate = H2_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(H2_UART_PORT, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(H2_UART_PORT, &uc));
    ESP_ERROR_CHECK(uart_set_pin(H2_UART_PORT, H2_UART_TX_PIN, H2_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void h2_send_line(const char *line)
{
    uart_write_bytes(H2_UART_PORT, line, strlen(line));
    uart_write_bytes(H2_UART_PORT, "\n", 1);
}

// Send the cached reading + location to the H2. temps per unit; NAN -> "-".
static void h2_send_weather(void)
{
    char buf[192], smin[16], smax[16], scur[16];
    #define FMT(dst, v) do { if (isnan(v)) strcpy(dst, "-"); else snprintf(dst, sizeof(dst), "%.1f", (double)(v)); } while (0)
    FMT(smin, s_tmin); FMT(smax, s_tmax); FMT(scur, s_tcur);
    #undef FMT
    snprintf(buf, sizeof(buf), "WX %s %s %s %s", smin, smax, scur, s_cond);
    h2_send_line(buf);
    snprintf(buf, sizeof(buf), "LOC %s", cfg.location);
    h2_send_line(buf);
    ESP_LOGI(TAG, "-> H2: WX %s %s %s %s / LOC %s", smin, smax, scur, s_cond, cfg.location);
}

// Read the H2's upstream lines (READY / NEED_WEATHER) and resend on request.
static void h2_rx_task(void *arg)
{
    char line[64];
    size_t len = 0;
    uint8_t ch;
    while (true) {
        int n = uart_read_bytes(H2_UART_PORT, &ch, 1, pdMS_TO_TICKS(1000));
        if (n == 1 && (ch == '\n' || ch == '\r')) {
            line[len] = '\0';
            if (len && (strcmp(line, "NEED_WEATHER") == 0 || strcmp(line, "READY") == 0)) {
                ESP_LOGI(TAG, "H2: %s", line);
                if (s_have_reading) h2_send_weather();   // immediate resync
                weather_trigger();                        // and refresh
            }
            len = 0;
        } else if (n == 1 && len < sizeof(line) - 1) {
            line[len++] = (char) ch;
        }
    }
}

// ── Weather fetch (Open-Meteo forecast) ──────────────────────────────────────
static const char *wmo_text(int code)
{
    switch (code) {
    case 0: return "Clear";
    case 1: return "Mainly Clear";
    case 2: return "Partly Cloudy";
    case 3: return "Overcast";
    case 45: return "Fog";
    case 48: return "Rime Fog";
    case 51: return "Light Drizzle";
    case 53: return "Drizzle";
    case 55: return "Heavy Drizzle";
    case 56: case 57: return "Freezing Drizzle";
    case 61: return "Light Rain";
    case 63: return "Rain";
    case 65: return "Heavy Rain";
    case 66: case 67: return "Freezing Rain";
    case 71: return "Light Snow";
    case 73: return "Snow";
    case 75: return "Heavy Snow";
    case 77: return "Snow Grains";
    case 80: return "Light Showers";
    case 81: return "Showers";
    case 82: return "Violent Showers";
    case 85: case 86: return "Snow Showers";
    case 95: return "Thunderstorm";
    case 96: case 99: return "Thunderstorm";
    default: return NULL;
    }
}

static bool json_daily_first(cJSON *daily, const char *key, float *out)
{
    cJSON *a = cJSON_GetObjectItem(daily, key);
    if (!cJSON_IsArray(a)) return false;
    cJSON *e = cJSON_GetArrayItem(a, 0);
    if (!cJSON_IsNumber(e)) return false;
    *out = (float) e->valuedouble;
    return true;
}

// Fetch, update the cache, and push to the H2. Requires Wi-Fi + geocode.
static void weather_fetch(void)
{
    if (!s_online || !cfg.have_geocode) return;

    char url[320];
    snprintf(url, sizeof(url),
             "https://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
             "&current=temperature_2m&daily=temperature_2m_min,temperature_2m_max,weather_code"
             "&temperature_unit=%s&timezone=auto&forecast_days=1",
             cfg.lat, cfg.lon, cfg.fahrenheit ? "fahrenheit" : "celsius");

    static char buf[2048];
    if (http_get_json(url, buf, sizeof(buf)) < 0) return;

    cJSON *root = cJSON_Parse(buf);
    if (!root) { ESP_LOGW(TAG, "weather JSON parse failed"); return; }

    cJSON *daily = cJSON_GetObjectItem(root, "daily");
    cJSON *current = cJSON_GetObjectItem(root, "current");
    float minv = NAN, maxv = NAN, curv = NAN;
    int code = -1;
    if (daily) {
        json_daily_first(daily, "temperature_2m_min", &minv);
        json_daily_first(daily, "temperature_2m_max", &maxv);
        float c;
        if (json_daily_first(daily, "weather_code", &c)) code = (int) c;
    }
    if (current) {
        cJSON *t = cJSON_GetObjectItem(current, "temperature_2m");
        if (cJSON_IsNumber(t)) curv = (float) t->valuedouble;
    }
    const char *cond = wmo_text(code);

    s_tmin = minv; s_tmax = maxv; s_tcur = curv;
    snprintf(s_cond, sizeof(s_cond), "%s", cond ? cond : "unknown");
    s_have_reading = true;
    cJSON_Delete(root);

    h2_send_weather();
}

static void weather_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS((uint32_t) cfg.interval_s * 1000));
        weather_fetch();
    }
}

static void weather_trigger(void)
{
    if (s_weather_task) xTaskNotifyGive(s_weather_task);
}

// ── Geocoding (place name → lat/lon) ─────────────────────────────────────────
typedef struct {
    char name[40];
    char region[40];
    char cc[8];
    float lat, lon;
} geo_result_t;

static geo_result_t s_geo[5];
static int s_geo_count = 0;

// Fills s_geo[]; returns count, or -1 on network error. Needs Wi-Fi online.
static int geocode_search(const char *query)
{
    s_geo_count = 0;
    if (!s_online) return -1;
    char q[128];
    url_encode(q, sizeof(q), query);
    char url[256];
    snprintf(url, sizeof(url),
             "https://geocoding-api.open-meteo.com/v1/search?name=%s&count=5&language=en&format=json", q);

    static char buf[8192];
    if (http_get_json(url, buf, sizeof(buf)) < 0) return -1;

    cJSON *root = cJSON_Parse(buf);
    if (!root) return -1;
    cJSON *results = cJSON_GetObjectItem(root, "results");
    if (cJSON_IsArray(results)) {
        int n = cJSON_GetArraySize(results);
        if (n > 5) n = 5;
        for (int i = 0; i < n; i++) {
            cJSON *r = cJSON_GetArrayItem(results, i);
            cJSON *name = cJSON_GetObjectItem(r, "name");
            cJSON *admin1 = cJSON_GetObjectItem(r, "admin1");
            cJSON *cc = cJSON_GetObjectItem(r, "country_code");
            cJSON *lat = cJSON_GetObjectItem(r, "latitude");
            cJSON *lon = cJSON_GetObjectItem(r, "longitude");
            if (!cJSON_IsNumber(lat) || !cJSON_IsNumber(lon)) continue;
            geo_result_t *g = &s_geo[s_geo_count];
            snprintf(g->name, sizeof(g->name), "%s", cJSON_IsString(name) ? name->valuestring : "?");
            snprintf(g->region, sizeof(g->region), "%s", cJSON_IsString(admin1) ? admin1->valuestring : "");
            snprintf(g->cc, sizeof(g->cc), "%s", cJSON_IsString(cc) ? cc->valuestring : "");
            g->lat = (float) lat->valuedouble;
            g->lon = (float) lon->valuedouble;
            s_geo_count++;
        }
    }
    cJSON_Delete(root);
    return s_geo_count;
}

// Adopt one of the s_geo[] search results as the configured location.
static void geo_select(int i)
{
    cfg.lat = s_geo[i].lat;
    cfg.lon = s_geo[i].lon;
    cfg.have_geocode = true;
    snprintf(cfg.location, sizeof(cfg.location), "%.16s", s_geo[i].name);
}

// ── Arrow-key "setup" TUI (menuconfig-style, over the USB serial console) ─────
// Raw keystrokes are read straight from the usb_serial_jtag driver (installed by
// the console REPL) so we can decode arrow-key escape sequences; ANSI codes draw
// the page. Logs are muted while the page is open so they don't corrupt it.
static bool s_creds_dirty = false;

enum { KEY_UP = 256, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_ENTER, KEY_BACKSPACE, KEY_ESC, KEY_NONE };

static void tui_out(const char *s) { usb_serial_jtag_write_bytes(s, strlen(s), portMAX_DELAY); }

static void tui_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    tui_out(buf);
}

static int tui_read_key(void)
{
    uint8_t c;
    if (usb_serial_jtag_read_bytes(&c, 1, portMAX_DELAY) != 1) return KEY_NONE;
    if (c == '\r' || c == '\n') return KEY_ENTER;
    if (c == 0x7f || c == 0x08) return KEY_BACKSPACE;
    if (c == 0x1b) {   // ESC alone, or the start of an arrow-key sequence
        uint8_t a;
        if (usb_serial_jtag_read_bytes(&a, 1, pdMS_TO_TICKS(40)) != 1) return KEY_ESC;
        if (a != '[' && a != 'O') return KEY_ESC;
        uint8_t b;
        if (usb_serial_jtag_read_bytes(&b, 1, pdMS_TO_TICKS(40)) != 1) return KEY_ESC;
        switch (b) {
        case 'A': return KEY_UP;
        case 'B': return KEY_DOWN;
        case 'C': return KEY_RIGHT;
        case 'D': return KEY_LEFT;
        default:  return KEY_ESC;
        }
    }
    return c;
}

#define TUI_CLEAR "\033[2J\033[H"
#define TUI_HIDE  "\033[?25l"
#define TUI_SHOW  "\033[?25h"

static void tui_row(bool sel, const char *label, const char *value)
{
    if (sel) tui_out("\033[7m");
    tui_printf("   %-15s : %s", label, value);
    if (sel) tui_out("\033[0m");
    tui_out("\r\n");
}

// Full-screen text editor. Returns true if confirmed (Enter), false on Esc.
static bool tui_edit_text(const char *title, char *buf, size_t cap, bool mask)
{
    char tmp[128];
    snprintf(tmp, sizeof(tmp), "%s", buf);
    size_t len = strlen(tmp);
    size_t limit = (cap - 1 < sizeof(tmp) - 1) ? cap - 1 : sizeof(tmp) - 1;
    for (;;) {
        tui_out(TUI_CLEAR TUI_SHOW);
        tui_printf("   %s\r\n\r\n   > ", title);
        if (mask) for (size_t i = 0; i < len; i++) tui_out("*");
        else      tui_out(tmp);
        tui_out("\r\n\r\n   Enter = save    Backspace = delete    Esc = cancel\r\n");
        int k = tui_read_key();
        if (k == KEY_ENTER)      { snprintf(buf, cap, "%s", tmp); return true; }
        if (k == KEY_ESC)        return false;
        if (k == KEY_BACKSPACE)  { if (len) tmp[--len] = '\0'; }
        else if (k >= 0x20 && k < 0x7f && len < limit) { tmp[len++] = (char) k; tmp[len] = '\0'; }
    }
}

// Wait (up to ~15 s) for Wi-Fi to come online, showing a spinner. Returns s_online.
static bool tui_wait_online(void)
{
    for (int i = 0; i < 150 && !s_online; i++) {
        if (i % 5 == 0) { tui_out(TUI_CLEAR); tui_printf("   Connecting to Wi-Fi%.*s\r\n", (i / 5) % 4, "..."); }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return s_online;
}

// Location sub-flow: ensure Wi-Fi, search Open-Meteo, arrow-pick a match.
static void tui_location(void)
{
    if (!s_online) {
        if (!cfg.ssid[0]) {
            tui_out(TUI_CLEAR "   Set Wi-Fi first, then search a location.\r\n\r\n   Press any key…\r\n");
            tui_read_key(); return;
        }
        wifi_restart(); s_creds_dirty = false;
        if (!tui_wait_online()) {
            tui_out(TUI_CLEAR "   Wi-Fi connect failed — check SSID/password.\r\n\r\n   Press any key…\r\n");
            tui_read_key(); return;
        }
    }
    char query[64] = "";
    if (!tui_edit_text("Search location (e.g. Melbourne)", query, sizeof(query), false)) return;
    if (!query[0]) return;

    tui_out(TUI_CLEAR "   Searching…\r\n");
    int n = geocode_search(query);
    if (n < 0)  { tui_out(TUI_CLEAR "   Search failed (network).\r\n\r\n   Press any key…\r\n"); tui_read_key(); return; }
    if (n == 0) { tui_out(TUI_CLEAR "   No matches.\r\n\r\n   Press any key…\r\n"); tui_read_key(); return; }

    int sel = 0;
    for (;;) {
        tui_out(TUI_CLEAR TUI_HIDE);
        tui_printf("   Pick a location for \"%s\"\r\n\r\n", query);
        for (int i = 0; i < n; i++) {
            char line[96];
            snprintf(line, sizeof(line), "%s%s%s%s%s", s_geo[i].name,
                     s_geo[i].region[0] ? ", " : "", s_geo[i].region,
                     s_geo[i].cc[0] ? ", " : "", s_geo[i].cc);
            tui_row(i == sel, "", line);
        }
        tui_out("\r\n   \342\206\221/\342\206\223 move    Enter select    Esc cancel\r\n");
        int k = tui_read_key();
        if (k == KEY_UP)        sel = (sel + n - 1) % n;
        else if (k == KEY_DOWN) sel = (sel + 1) % n;
        else if (k == KEY_ENTER) {
            geo_select(sel);
            return;
        } else if (k == KEY_ESC) return;
    }
}

static void tui_edit_interval(void)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%u", (unsigned) (cfg.interval_s / 60));
    if (!tui_edit_text("Refresh interval in minutes (min 1)", buf, sizeof(buf), false)) return;
    long m = atol(buf);
    if (m < 1) m = 60;
    cfg.interval_s = (uint32_t) (m * 60);
}

enum { F_SSID, F_PASS, F_LOC, F_UNIT, F_INTERVAL, F_SAVE, F_EXIT, F_COUNT };

static void save_apply(void)
{
    cfg_save();
    if (cfg.ssid[0] && (s_creds_dirty || !s_online)) { wifi_restart(); s_creds_dirty = false; }
    weather_trigger();
}

static void setup_tui(void)
{
    esp_log_level_set("*", ESP_LOG_NONE);   // mute logs so they don't corrupt the page
    int sel = 0;
    const char *status = "";
    bool done = false;
    while (!done) {
        char masked[33]; size_t pl = strlen(cfg.pass);
        for (size_t i = 0; i < pl && i < sizeof(masked) - 1; i++) masked[i] = '*';
        masked[pl < sizeof(masked) - 1 ? pl : sizeof(masked) - 1] = '\0';
        char interval[16]; snprintf(interval, sizeof(interval), "%u min", (unsigned) (cfg.interval_s / 60));

        tui_out(TUI_CLEAR TUI_HIDE);
        tui_out("   Weather Device \342\200\224 Setup\r\n");
        tui_out("   =========================\r\n\r\n");
        tui_row(sel == F_SSID,     "Wi-Fi SSID",     cfg.ssid[0] ? cfg.ssid : "(not set)");
        tui_row(sel == F_PASS,     "Wi-Fi Password", pl ? masked : "(not set)");
        tui_row(sel == F_LOC,      "Location",       cfg.have_geocode ? cfg.location : "(not set)");
        tui_row(sel == F_UNIT,     "Units",          cfg.fahrenheit ? "Fahrenheit" : "Celsius");
        tui_row(sel == F_INTERVAL, "Refresh",        interval);
        tui_out("\r\n");
        tui_row(sel == F_SAVE, "", "[ Save ]");
        tui_row(sel == F_EXIT, "", "[ Save & Exit ]");
        tui_out("\r\n   \342\206\221/\342\206\223 move    Enter select/edit    \342\206\220/\342\206\222 toggle    Esc or q save & exit\r\n");
        if (status[0]) tui_printf("\r\n   %s\r\n", status);
        status = "";

        int k = tui_read_key();
        if (k == KEY_UP)        sel = (sel + F_COUNT - 1) % F_COUNT;
        else if (k == KEY_DOWN) sel = (sel + 1) % F_COUNT;
        // 'q' as well as Esc: a terminal that can only send whole lines (a browser
        // serial console) can't send a bare Esc, so it needs a printable way out.
        else if (k == KEY_ESC || k == 'q' || k == 'Q') { save_apply(); done = true; }
        else if ((k == KEY_LEFT || k == KEY_RIGHT) && sel == F_UNIT) cfg.fahrenheit = !cfg.fahrenheit;
        else if (k == KEY_ENTER) {
            switch (sel) {
            case F_SSID:     if (tui_edit_text("Wi-Fi SSID", cfg.ssid, sizeof(cfg.ssid), false)) s_creds_dirty = true; break;
            case F_PASS:     if (tui_edit_text("Wi-Fi Password", cfg.pass, sizeof(cfg.pass), true)) s_creds_dirty = true; break;
            case F_LOC:      tui_location(); break;
            case F_UNIT:     cfg.fahrenheit = !cfg.fahrenheit; break;
            case F_INTERVAL: tui_edit_interval(); break;
            case F_SAVE:     save_apply(); status = "Saved."; break;
            case F_EXIT:     save_apply(); done = true; break;
            }
        }
    }
    tui_out(TUI_CLEAR TUI_SHOW);
    tui_printf("Setup saved. SSID=%s  location=%s  unit=%s  every %u min\r\n",
               cfg.ssid[0] ? cfg.ssid : "(unset)",
               cfg.have_geocode ? cfg.location : "(unset)",
               cfg.fahrenheit ? "F" : "C", (unsigned) (cfg.interval_s / 60));
    esp_log_level_set("*", ESP_LOG_INFO);   // restore logs
}

static int cmd_setup(int argc, char **argv)
{
    setup_tui();
    return 0;
}

// ── Line commands ────────────────────────────────────────────────────────────
// The `setup` page needs arrow keys and a full-screen redraw. Browser serial
// consoles (ESP Launchpad, esp-web-tools) send input a whole line at a time and
// never forward escape sequences, so every setting is also a command here.

// Bring Wi-Fi up if it isn't already and wait ~15 s for an address. Geocoding
// needs the network, and over a line console there's no spinner to show.
static bool cli_wait_online(void)
{
    if (s_online) return true;
    if (!cfg.ssid[0]) return false;
    wifi_restart();
    s_creds_dirty = false;
    printf("Connecting to Wi-Fi…\n");
    for (int i = 0; i < 150 && !s_online; i++) vTaskDelay(pdMS_TO_TICKS(100));
    return s_online;
}

static int cmd_show(int argc, char **argv)
{
    printf("Wi-Fi SSID : %s\n", cfg.ssid[0] ? cfg.ssid : "(not set)");
    printf("Password   : %s\n", cfg.pass[0] ? "(set)" : "(not set)");
    if (cfg.have_geocode) printf("Location   : %s  (%.4f, %.4f)\n", cfg.location, cfg.lat, cfg.lon);
    else                  printf("Location   : (not set)\n");
    printf("Units      : %s\n", cfg.fahrenheit ? "Fahrenheit" : "Celsius");
    printf("Refresh    : %u min\n", (unsigned) (cfg.interval_s / 60));
    printf("Wi-Fi      : %s\n", s_online ? "online" : "offline");
    return 0;
}

static int cmd_wifi(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: wifi <ssid> [password]   (quote values containing spaces)\n");
        return 1;
    }
    snprintf(cfg.ssid, sizeof(cfg.ssid), "%s", argv[1]);
    snprintf(cfg.pass, sizeof(cfg.pass), "%s", argc > 2 ? argv[2] : "");
    s_creds_dirty = true;
    printf("Wi-Fi set to \"%s\". Run 'save' to connect.\n", cfg.ssid);
    return 0;
}

static void print_geo_results(int n)
{
    for (int i = 0; i < n; i++) {
        printf("  %d) %s%s%s%s%s\n", i + 1, s_geo[i].name,
               s_geo[i].region[0] ? ", " : "", s_geo[i].region,
               s_geo[i].cc[0] ? ", " : "", s_geo[i].cc);
    }
}

static int cmd_location(int argc, char **argv)
{
    if (argc < 2) { printf("usage: location <place name>\n"); return 1; }

    // Join the remaining words: place names have spaces and shouldn't need quoting.
    char query[64] = "";
    for (int i = 1; i < argc; i++) {
        if (i > 1) strlcat(query, " ", sizeof(query));
        strlcat(query, argv[i], sizeof(query));
    }

    if (!cli_wait_online()) {
        printf("Wi-Fi isn't connected — set 'wifi <ssid> <password>' and 'save' first.\n");
        return 1;
    }
    int n = geocode_search(query);
    if (n < 0)  { printf("Search failed (network).\n"); return 1; }
    if (n == 0) { printf("No matches for \"%s\".\n", query); return 1; }

    print_geo_results(n);
    if (n == 1) {
        geo_select(0);
        printf("Location set to %s. Run 'save' to apply.\n", cfg.location);
    } else {
        printf("Run 'pick <n>' to choose one.\n");
    }
    return 0;
}

static int cmd_pick(int argc, char **argv)
{
    if (argc < 2) { printf("usage: pick <n>\n"); return 1; }
    int i = atoi(argv[1]) - 1;
    if (i < 0 || i >= s_geo_count) {
        printf("No result %s — run 'location <place name>' first.\n", argv[1]);
        return 1;
    }
    geo_select(i);
    printf("Location set to %s. Run 'save' to apply.\n", cfg.location);
    return 0;
}

static int cmd_units(int argc, char **argv)
{
    char u = argc > 1 ? argv[1][0] : '\0';
    if (u != 'c' && u != 'C' && u != 'f' && u != 'F') { printf("usage: units <c|f>\n"); return 1; }
    cfg.fahrenheit = (u == 'f' || u == 'F');
    printf("Units: %s. Run 'save' to apply.\n", cfg.fahrenheit ? "Fahrenheit" : "Celsius");
    return 0;
}

static int cmd_refresh(int argc, char **argv)
{
    long m = argc > 1 ? atol(argv[1]) : 0;
    if (m < 1) { printf("usage: refresh <minutes>   (minimum 1)\n"); return 1; }
    cfg.interval_s = (uint32_t) (m * 60);
    printf("Refresh: %ld min. Run 'save' to apply.\n", m);
    return 0;
}

static int cmd_save(int argc, char **argv)
{
    save_apply();
    printf("Saved.\n");
    return cmd_show(0, NULL);
}

// Fetch the weather now (everything else is configured in `setup` or with the
// line commands above).
static int cmd_poll(int argc, char **argv)
{
    if (!cfg_ready()) {
        printf("Not configured yet — run 'setup', or 'wifi' + 'location' + 'save'.\n");
        return 1;
    }
    printf("Fetching weather…\n");
    weather_trigger();
    return 0;
}

static const esp_console_cmd_t s_commands[] = {
    { .command = "show",     .hint = NULL, .func = &cmd_show,     .help = "Show the current configuration" },
    { .command = "wifi",     .hint = NULL, .func = &cmd_wifi,     .help = "Set Wi-Fi credentials: wifi <ssid> [password]" },
    { .command = "location", .hint = NULL, .func = &cmd_location, .help = "Search for a location: location <place name>" },
    { .command = "pick",     .hint = NULL, .func = &cmd_pick,     .help = "Choose one of the last search results: pick <n>" },
    { .command = "units",    .hint = NULL, .func = &cmd_units,    .help = "Temperature units: units <c|f>" },
    { .command = "refresh",  .hint = NULL, .func = &cmd_refresh,  .help = "Refresh interval: refresh <minutes>" },
    { .command = "save",     .hint = NULL, .func = &cmd_save,     .help = "Save the configuration, reconnect and fetch" },
    { .command = "setup",    .hint = NULL, .func = &cmd_setup,    .help = "Open the full-screen setup page (needs a real terminal)" },
    { .command = "poll",     .hint = NULL, .func = &cmd_poll,     .help = "Fetch the weather now" },
};

static void console_init(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t rc = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    rc.prompt = "weather>";
    rc.max_cmdline_length = 160;
    rc.task_stack_size = 10240;   // geocode/fetch TLS runs on the console task
    esp_console_dev_usb_serial_jtag_config_t hw = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw, &rc, &repl));
    esp_console_register_help_command();
    for (size_t i = 0; i < sizeof(s_commands) / sizeof(s_commands[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&s_commands[i]));
    }
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    cfg_load();
    h2_uart_init();
    wifi_init();
    xTaskCreate(h2_rx_task, "h2_rx", 3072, NULL, 5, NULL);
    xTaskCreate(weather_task, "weather", 6144, NULL, 5, &s_weather_task);

    ESP_LOGI(TAG, "S3 weather front-end up. configured=%s",
             cfg_ready() ? "yes" : "no (type 'setup', or 'help' for the line commands)");
    if (cfg.ssid[0]) wifi_start();

    console_init();   // start the interactive REPL (reads stdin)
}
