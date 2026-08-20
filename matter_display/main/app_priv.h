/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include <esp_err.h>
#include <esp_matter.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include "esp_openthread_types.h"
#endif

typedef void *app_driver_handle_t;

/** Initialize the switch driver
 *
 * This initializes the switch driver associated with the selected board.
 *
 * @return Handle on success.
 * @return NULL in case of failure.
 */
app_driver_handle_t app_driver_switch_init();

/** Send a Matter OnOff::Toggle command to the bound light(s).
 *
 * Acquires the CHIP stack lock internally, so it must be called from a task
 * that does NOT already hold it (e.g. the button task or the LVGL task).
 */
void app_driver_send_toggle();

/** Send an explicit Matter OnOff::On or OnOff::Off to ALL bound lights/plugs (the group fan-out)
 * instead of Toggle, so the master toggle's direction maps to a definite state.
 *
 * @param on  true → On, false → Off.
 *
 * Acquires the CHIP stack lock internally; call from a task that does NOT hold it.
 */
void app_driver_send_onoff(bool on);

/** Send a Matter LevelControl::MoveToLevelWithOnOff command to the bound
 * light(s), setting brightness (and turning the light on if it was off).
 *
 * @param level  Target level, 1-254 (Matter LevelControl range).
 *
 * Acquires the CHIP stack lock internally, so it must be called from a task
 * that does NOT already hold it (e.g. the LVGL task).
 */
void app_driver_send_level(uint8_t level);

/** Send a Matter ColorControl::MoveToColorTemperature command to the bound
 * light(s).
 *
 * @param mireds  Target color temperature in mireds (1000000 / Kelvin).
 *
 * Acquires the CHIP stack lock internally, so it must be called from a task
 * that does NOT already hold it (e.g. the LVGL task).
 */
void app_driver_send_color_temp(uint16_t mireds);

/** Send a Matter ColorControl::MoveToHueAndSaturation command to the bound light(s).
 *
 * @param hue  Target hue, 0-254 (maps over 0-360°).
 * @param sat  Target saturation, 0-254 (254 = fully saturated).
 *
 * Acquires the CHIP stack lock internally; call from a task that does NOT hold it.
 */
void app_driver_send_color_hue_sat(uint8_t hue, uint8_t sat);

/** Send a Matter OnOff::Toggle to ONE specific bound device (not the whole group), identified by
 * fabric + node + endpoint. Used by the per-device "Lights" page.
 *
 * Acquires the CHIP stack lock internally; call from a task that does NOT hold it.
 */
void app_driver_send_toggle_node(uint8_t fabric_index, uint64_t node_id, uint16_t endpoint);

/** Nudge the Matter ICD into active mode so attribute reports keep flowing promptly. Called
 * repeatedly while the display is on. Compiles to a no-op unless the ICD server is enabled.
 * Schedules its work on the CHIP thread, so it is safe to call from any task. */
void app_driver_notify_active(void);

/** Matter factory reset: erase all fabrics / bindings / stored data and restart. Triggered from
 * the Settings page behind a confirmation overlay. Does not return on success. Call from a task
 * that does NOT hold the CHIP stack lock. */
void app_driver_factory_reset(void);

/** Notify the sensor layer that the binding table changed, so the Indoor page's per-cluster source
 * of truth is recomputed from the next round of reports. Call from the Matter task. */
void app_sensor_bindings_changed(void);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG()                                           \
    {                                                                                   \
        .radio_mode = RADIO_MODE_NATIVE,                                                \
    }

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG()                                            \
    {                                                                                   \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE,                              \
    }

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG()                                            \
    {                                                                                   \
        .storage_partition_name = "nvs", .netif_queue_size = 10, .task_queue_size = 10, \
    }
#endif
