/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include <esp_err.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Which screen the UI is showing. */
typedef enum {
    LIGHT_UI_MODE_PAIRING = 0,  /* not commissioned: show the Matter QR code only */
    LIGHT_UI_MODE_NO_BINDING,   /* commissioned but nothing bound: show a notice  */
    LIGHT_UI_MODE_READY,        /* commissioned + a light bound: show the controls */
    LIGHT_UI_MODE_BOOTING,      /* startup: neutral loading screen (see note below) */
} light_ui_mode_t;

/** Bring up the AMOLED display + touch + LVGL, build all screens, and start the
 *  LVGL task. The UI starts in pairing mode (QR code); call light_ui_set_mode()
 *  to switch screens. Tapping the bulb sends a Matter OnOff::Toggle, and the
 *  sliders send LevelControl / ColorControl commands to the bound light(s).
 *
 *  @param qr_payload   Matter onboarding QR string ("MT:...") to render.
 *  @param manual_code  Manual pairing code digits to show under the QR.
 *  @param commissioned Whether the device has already joined a fabric. When false, commissioning
 *                      is still to come, so the LVGL draw buffers are kept small to leave heap for
 *                      the handshake (which runs with BLE resident); full-size buffers are used
 *                      once paired. See amoled_lvgl_set_dma_reserve().
 *  @return ESP_OK on success.
 */
esp_err_t light_ui_start(const char *qr_payload, const char *manual_code, bool commissioned);

/** Switch which screen is shown (pairing QR / no-binding notice / controls).
 *  Swiping between control pages is only possible in READY mode. Thread-safe;
 *  safe to call from the Matter/chip stack thread. No-op before the UI starts.
 *
 *  A request for READY / NO_BINDING while the device is still booting is deferred until the
 *  startup storm has passed, so the real UI never paints mid-storm; PAIRING is honoured at once.
 */
void light_ui_set_mode(light_ui_mode_t mode);

/** Enable or disable display power management (idle sleep + touch-to-wake).
 *  When disabled, the display is kept on and the panel power is never cycled,
 *  so it cannot interfere with commissioning. The app should only enable this
 *  once the device is fully commissioned and no commissioning window is open.
 *  Thread-safe; safe to call from the Matter/chip stack thread.
 */
void light_ui_set_power_save_enabled(bool enable);

/** Update the on-screen bulb to reflect the real light state (amber = on,
 *  dim grey = off). Thread-safe: may be called from the Matter/chip stack
 *  thread (e.g. from the On/Off subscription callback). No-op if the UI has
 *  not been started yet.
 */
void light_ui_set_state(bool on);

/** Update the brightness slider to reflect the bound light's real level.
 *  Thread-safe; safe to call from the Matter/chip stack thread. Ignored while
 *  the user is actively dragging the slider, and a no-op before the UI starts.
 *
 *  @param level  Matter LevelControl level, 1-254.
 */
void light_ui_set_brightness(uint8_t level);

/** Update the color-temperature slider to reflect the bound light's real value.
 *  Thread-safe; safe to call from the Matter/chip stack thread. Ignored while
 *  the user is dragging the CT slider, and a no-op before the UI starts.
 *
 *  @param mireds  Color temperature in mireds (1000000 / Kelvin).
 */
void light_ui_set_color_temp(uint16_t mireds);

/** Update the climate page with a temperature reading from a bound Matter
 *  temperature sensor. Thread-safe; safe to call from the Matter/chip stack
 *  thread (e.g. from the sensor subscription callback). No-op before the UI
 *  starts. Until the first reading arrives the page shows "--".
 *
 *  @param celsius  Temperature in degrees Celsius.
 */
void light_ui_set_temperature(float celsius);

/** Update the climate page with a relative-humidity reading from a bound Matter
 *  humidity sensor. Thread-safe; safe to call from the Matter/chip stack thread.
 *  No-op before the UI starts. Until the first reading arrives the page shows "--".
 *
 *  @param percent  Relative humidity, 0-100 %.
 */
void light_ui_set_humidity(float percent);

/** Update the air-quality page with the overall rating from a bound Matter air-quality
 *  monitor. Thread-safe; safe to call from the Matter/chip stack thread. No-op before the
 *  UI starts.
 *
 *  @param level  Matter AirQualityEnum: 0 Unknown, 1 Good, 2 Fair, 3 Moderate, 4 Poor,
 *                5 Very Poor, 6 Extremely Poor.
 */
void light_ui_set_air_quality(int level);

/** Update the air-quality page with a CO2 concentration reading. Thread-safe. No-op before
 *  the UI starts; shows "--" until the first reading arrives.
 *
 *  @param ppm  CO2 concentration in parts per million.
 */
void light_ui_set_co2(float ppm);

/** Update the air-quality page with a PM2.5 concentration reading. Thread-safe. No-op before
 *  the UI starts; shows "--" until the first reading arrives.
 *
 *  @param ugm3  PM2.5 concentration in micrograms per cubic metre.
 */
void light_ui_set_pm25(float ugm3);

/** Update the weather page's forecast min / max / current temperatures from the bound weather
 *  aggregator device. Thread-safe; no-op before the UI starts; shows "--" until data arrives.
 *
 *  @param celsius  Temperature in degrees Celsius.
 */
void light_ui_set_weather_min(float celsius);
void light_ui_set_weather_max(float celsius);
void light_ui_set_weather_current(float celsius);

/** Update the weather page's short text forecast (from the aggregator's UserLabel "condition").
 *  Thread-safe; no-op before the UI starts. The string is copied.
 */
void light_ui_set_weather_condition(const char *condition);

/** Update the weather page's location name (from the aggregator's FixedLabel "location").
 *  Thread-safe; no-op before the UI starts. The string is copied.
 */
void light_ui_set_weather_location(const char *location);

/** Flash a full-screen alert overlay above whatever page is showing (a bound contact sensor
 *  opening, or a water-leak detector tripping). Wakes the display if asleep and auto-dismisses
 *  after a few seconds. Thread-safe; no-op before the UI starts.
 *
 *  @param title     Short headline, e.g. "Water Leak" or "Contact Open".
 *  @param subtitle  Optional second line (may be NULL/empty), e.g. a location or sensor name.
 *  @param critical  true → red (water leak); false → amber (contact/warning).
 */
void light_ui_show_alert(const char *title, const char *subtitle, bool critical);

/** Set the local offset (seconds) for the on-screen clock. app_main derives it from the Matter
 *  TimeSynchronization cluster as standard-timezone offset + the active daylight-saving offset, so
 *  the clock switches automatically at DST boundaries. Thread-safe; no-op before the UI starts.
 *
 *  @param offset_seconds  Total seconds east of UTC incl. DST (e.g. +7200 for UTC+1 in summer).
 */
void light_ui_set_utc_offset(int32_t offset_seconds);

/** Whether the on-screen clock is in automatic (controller-synced) mode: true (the default) when a
 *  controller's SetUTCTime may override a manually-set time, false when the user turned auto-sync
 *  off on the Time & Date page. Checked by app_main before honouring a time push. Thread-safe. */
bool light_ui_time_auto_enabled(void);

/** Re-apply the user's manually-set time, advanced by the uptime elapsed since they set it. Called
 *  when a controller pushes UTC while auto-sync is off, to undo the overwrite. No-op if no manual
 *  time was set this boot. Thread-safe. */
void light_ui_reassert_manual_time(void);

/** Register / update one individually-controllable bound device (light or smart plug) on the
 *  per-device "Lights" page, keyed by (node, endpoint). Called when its OnOff state is reported.
 *  Creates a card for the device if it's new. Thread-safe; safe to call from the Matter/chip stack
 *  thread; no-op before the UI starts.
 *
 *  @param fabric_index  Fabric the device is on (needed to toggle it).
 *  @param node_id       Matter node id of the device.
 *  @param endpoint      Endpoint on that node exposing OnOff.
 *  @param on            Current on/off state.
 */
void light_ui_set_device_state(uint8_t fabric_index, uint64_t node_id, uint16_t endpoint, bool on);

/** Set a known device's icon kind (light vs smart plug), from its Matter device type. Keyed by
 *  (node, endpoint). Thread-safe; no-op before the UI starts or if the device isn't known yet.
 *
 *  @param is_plug  1 = smart plug icon, 0 = light icon.
 */
void light_ui_set_device_kind(uint64_t node_id, uint16_t endpoint, int is_plug);

#ifdef __cplusplus
}
#endif
