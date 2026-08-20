/*
   Matter Display — app entry point.

   Creates the Matter node (a colour dimmer switch, plus client clusters for the sensors and the
   weather aggregator it binds to), starts Matter over Thread, brings up the AMOLED UI, and wires
   the Matter events (commissioning, bindings, time sync) to the display.

   This code is in the Public Domain (or CC0 licensed, at your option.) Unless required by
   applicable law or agreed to in writing, this software is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_timer.h>
#include <time.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>
#include <esp_matter_providers.h>

#include <common_macros.h>
#include <app_priv.h>
#include <app_reset.h>
#include "light_ui.h"
#include "audio.h"

#include <app/server/Server.h>
#include <app/clusters/bindings/binding-table.h>
#include <setup_payload/OnboardingCodesUtil.h>
#include <setup_payload/SetupPayload.h>
#include <lib/support/Span.h>
#include <esp_matter_client.h>
#include <app/AttributePathParams.h>
#include <app/ConcreteAttributePath.h>
#include <lib/core/TLVReader.h>
#include <app/clusters/time-synchronization-server/DefaultTimeSyncDelegate.h>
#include <app/clusters/time-synchronization-server/TimeSyncDataProvider.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif
#if CONFIG_DYNAMIC_PASSCODE_COMMISSIONABLE_DATA_PROVIDER
#include <custom_provider/dynamic_commissionable_data_provider.h>
#endif

static const char *TAG = "app_main";
uint16_t switch_endpoint_id = 0;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;

#if CONFIG_DYNAMIC_PASSCODE_COMMISSIONABLE_DATA_PROVIDER
dynamic_commissionable_data_provider g_dynamic_passcode_provider;
#endif

static bool do_subscribe = true;

// Tracks whether a commissioning window is currently open. Display power
// management is kept off while it is, so it can never interfere with pairing.
static bool s_commissioning_window_open = false;

// Defined further down; forward-declared so the delegate can call it.
static void app_update_local_offset(void);  // recompute standard+DST local offset from the cluster

// True once the controller has pushed us the time this boot. Until then we report
// NoTimeGranularity, which is what tells the controller the device needs the time.
static bool s_time_from_controller = false;

// TimeSync delegate, driven by the Matter controller:
//  - TimeZoneListChanged        → recompute the local offset (standard + active DST).
//  - UTCTimeAvailabilityChanged → the controller set the wall clock.
//  - UpdateTimeFromPlatformSource → keep reporting "no time" until it has pushed.
class AppTimeSyncDelegate : public chip::app::Clusters::TimeSynchronization::DefaultTimeSyncDelegate {
public:
    void TimeZoneListChanged(const chip::Span<chip::TimeSyncDataProvider::TimeZoneStore> timeZoneList) override
    {
        ESP_LOGI(TAG, "Controller pushed timezone (%u entr%s)", (unsigned) timeZoneList.size(),
                 timeZoneList.size() == 1 ? "y" : "ies");
        app_update_local_offset();
    }

    void UTCTimeAvailabilityChanged(uint64_t time) override
    {
        // Respect the "Automatic" toggle on the Time & Date page: with auto-sync off, keep
        // the manually-set time and undo the controller's clock write.
        if (!light_ui_time_auto_enabled()) {
            ESP_LOGI(TAG, "Controller set UTC ignored (manual time mode)");
            light_ui_reassert_manual_time();
            return;
        }
        ESP_LOGI(TAG, "Controller set UTC");
        s_time_from_controller = true;
        app_update_local_offset();
    }

    CHIP_ERROR UpdateTimeFromPlatformSource(
        chip::Callback::Callback<chip::app::Clusters::TimeSynchronization::OnTimeSyncCompletion> * callback) override
    {
        if (!s_time_from_controller) {
            // No controller-set time yet: don't advertise the RTC as a synced source, so the
            // controller keeps pushing SetUTCTime.
            return CHIP_ERROR_NOT_IMPLEMENTED;
        }
        return DefaultTimeSyncDelegate::UpdateTimeFromPlatformSource(callback);
    }
};

// Recompute the UI screen and whether display power management may run, from the
// commissioning state. The CHIP stack lock must be held by the caller (reads the
// FabricTable).
static void app_refresh_ui_mode()
{
    uint8_t fabric_count = chip::Server::GetInstance().GetFabricTable().FabricCount();
    bool commissioned = (fabric_count > 0);

    light_ui_mode_t mode;
    if (!commissioned) {
        mode = LIGHT_UI_MODE_PAIRING;
    } else {
        // Once commissioned, always show the main UI: the clock and weather pages are
        // useful even with nothing bound yet.
        mode = LIGHT_UI_MODE_READY;
    }

    // Display sleep / touch-to-wake only runs once commissioned and with no
    // commissioning window open, so its panel power cycling and I2C/SPI re-init can
    // never disturb pairing.
    light_ui_set_power_save_enabled(commissioned && !s_commissioning_window_open);
    light_ui_set_mode(mode);
}

// Sensor nodes we have already issued a subscription connect for (this boot), so repeated
// triggers (boot / IP-change / binding-change) don't stack CASE handshakes to the same node.
#define MAX_SENSOR_NODES 8
static chip::NodeId s_handled_sensor_nodes[MAX_SENSOR_NODES];
static size_t s_handled_sensor_node_count = 0;

static bool sensor_node_is_handled(chip::NodeId nodeId)
{
    for (size_t i = 0; i < s_handled_sensor_node_count; i++) {
        if (s_handled_sensor_nodes[i] == nodeId) {
            return true;
        }
    }
    return false;
}

static void sensor_node_mark_handled(chip::NodeId nodeId)
{
    if (sensor_node_is_handled(nodeId) || s_handled_sensor_node_count >= MAX_SENSOR_NODES) {
        return;
    }
    s_handled_sensor_nodes[s_handled_sensor_node_count++] = nodeId;
}

// Whether this binding makes its node a candidate for sensor/weather subscriptions. Explicit
// sensor/label cluster bindings count, and so do "all clusters" wildcard bindings — some
// controllers only create wildcard bindings for the weather aggregator. Bound lights also use
// wildcard bindings; subscribing to one is harmless since we only ask for sensor/label clusters.
static bool binding_is_sensor(const chip::app::Clusters::Binding::TableEntry &binding)
{
    if (!binding.clusterId.has_value()) {
        return true;  // wildcard ("all clusters") — could be the weather aggregator
    }
    chip::ClusterId cid = binding.clusterId.value();
    return cid == chip::app::Clusters::TemperatureMeasurement::Id ||
           cid == chip::app::Clusters::RelativeHumidityMeasurement::Id ||
           cid == chip::app::Clusters::AirQuality::Id ||
           cid == chip::app::Clusters::CarbonDioxideConcentrationMeasurement::Id ||
           cid == chip::app::Clusters::Pm25ConcentrationMeasurement::Id ||
           cid == chip::app::Clusters::UserLabel::Id ||
           cid == chip::app::Clusters::FixedLabel::Id ||
           cid == chip::app::Clusters::BooleanState::Id ||
           cid == chip::app::Clusters::BooleanStateConfiguration::Id;
}

// Establish a CASE session to a sensor node and subscribe on success. Defined in app_driver.cpp,
// which drives CASE with its own per-node callbacks so several nodes can connect in parallel
// (esp_matter's client::connect() uses shared static callbacks that would clobber each other).
esp_err_t app_sensor_connect_node(chip::FabricIndex fabric_index, chip::NodeId node_id);

// Called by app_driver when a node's CASE connect fails: drop it from the handled set so a later
// trigger (IP-change / binding-change) retries it. (On success the node stays handled.)
void app_sensor_node_connect_failed(chip::NodeId nodeId)
{
    for (size_t i = 0; i < s_handled_sensor_node_count; i++) {
        if (s_handled_sensor_nodes[i] == nodeId) {
            s_handled_sensor_nodes[i] = s_handled_sensor_nodes[--s_handled_sensor_node_count];
            return;
        }
    }
}

// Connect every not-yet-handled bound sensor node, in parallel. Called on boot / IP-change /
// binding-change; the handled set dedupes by node (the weather aggregator alone contributes one
// binding per temperature endpoint). Must run on the Matter task, with the stack lock held.
static void subscribe_to_sensor_bindings()
{
    for (const auto &binding : chip::app::Clusters::Binding::Table::GetInstance()) {
        if (binding.type != chip::app::Clusters::Binding::MATTER_UNICAST_BINDING) {
            continue;
        }
        if (!binding_is_sensor(binding) || sensor_node_is_handled(binding.nodeId)) {
            continue;
        }
        sensor_node_mark_handled(binding.nodeId);
        app_sensor_connect_node(binding.fabricIndex, binding.nodeId);
    }
}

// Kick off the subscription on the CHIP event-loop task. app_main() runs on the small "main"
// task, and connect() can call back synchronously on a cached CASE session, which would overflow
// that stack.
static void sensor_kickoff_work(intptr_t)
{
    subscribe_to_sensor_bindings();
}

// --- DST-aware local time -------------------------------------------------------------------
// This is an IPv6-only Thread build with no route to an NTP server, so the only source of UTC is
// the Matter controller's SetUTCTime (the display shows "syncing" until it arrives, and the Time &
// Date page can set it by hand). Timezone + DST also come from the controller and are persisted by
// the TimeSynchronization cluster, so no timezone is ever hardcoded.

// Recompute the clock's local offset = standard-timezone offset + the currently-active DST
// offset, both from the TimeSynchronization cluster's persisted storage. Re-evaluated on a timer
// so DST switches automatically. Must run with the CHIP stack lock held.
static void app_update_local_offset(void)
{
    namespace TSync = chip::app::Clusters::TimeSynchronization;

    chip::TimeSyncDataProvider provider;
    provider.Init(chip::Server::GetInstance().GetPersistentStorage());

    int32_t std_offset = 0;
    chip::TimeSyncDataProvider::TimeZoneStore tzBuf[CHIP_CONFIG_TIME_ZONE_LIST_MAX_SIZE];
    chip::TimeSyncDataProvider::TimeZoneObj tzObj{
        chip::Span<chip::TimeSyncDataProvider::TimeZoneStore>(tzBuf), 0
    };
    if (provider.LoadTimeZone(tzObj) == CHIP_NO_ERROR && tzObj.validSize > 0) {
        std_offset = tzBuf[0].timeZone.offset;   // seconds east of UTC (standard time)
    }

    int32_t dst_offset = 0;
    TSync::Structs::DSTOffsetStruct::Type dstBuf[CHIP_CONFIG_DST_OFFSET_LIST_MAX_SIZE];
    chip::TimeSyncDataProvider::DSTOffsetObj dstObj{
        chip::app::DataModel::List<TSync::Structs::DSTOffsetStruct::Type>(dstBuf), 0
    };
    if (provider.LoadDSTOffset(dstObj) == CHIP_NO_ERROR) {
        // validStarting/validUntil are Matter epoch-us (since 2000-01-01). Convert to Unix seconds
        // and pick the entry whose window contains "now".
        static const uint64_t kChipEpochUnixSec = 946684800ULL;  // 2000-01-01 in Unix seconds
        time_t now = time(nullptr);
        for (size_t i = 0; i < dstObj.validSize; i++) {
            time_t start = (time_t) (dstBuf[i].validStarting / 1000000ULL + kChipEpochUnixSec);
            bool ended   = false;
            if (!dstBuf[i].validUntil.IsNull()) {
                time_t until = (time_t) (dstBuf[i].validUntil.Value() / 1000000ULL + kChipEpochUnixSec);
                ended = (now >= until);
            }
            if (now >= start && !ended) {
                dst_offset = dstBuf[i].offset;
                break;
            }
        }
    }

    int32_t total = std_offset + dst_offset;
    static int32_t s_last_total = INT32_MIN;
    if (total != s_last_total) {
        s_last_total = total;
        ESP_LOGI(TAG, "Local time offset: %+ld s (standard %+ld, DST %+ld)", (long) total,
                 (long) std_offset, (long) dst_offset);
    }
    light_ui_set_utc_offset(total);
}

// esp_timer callbacks run on the timer task; hop onto the Matter task (holds the CHIP lock) to read
// the cluster's persistent storage safely.
static void app_offset_recompute_work(intptr_t)
{
    app_update_local_offset();
}

// Periodic (10 min): re-evaluate the DST-aware local offset so it switches automatically when a
// DSTOffset entry's validUntil rolls into the next one. Read-only (no flash writes).
static void app_dst_refresh_timer_cb(void *arg)
{
    CHIP_ERROR err = chip::DeviceLayer::PlatformMgr().ScheduleWork(app_offset_recompute_work, 0);
    if (err != CHIP_NO_ERROR) {
        ESP_LOGW(TAG, "Failed to schedule local-offset recompute: %s", chip::ErrorStr(err));
    }
}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address Changed");
        // Re-establish the sensor subscription once the network is up. This is the
        // path that restores it after a reboot (kBindingsChangedViaCluster only fires
        // when the binding table is written, not when it is restored from NVS at boot).
        subscribe_to_sensor_bindings();
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        app_refresh_ui_mode();
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        // Covers first boot and re-opening after decommissioning (last fabric
        // removed) — show the QR again when there are no fabrics. Also pauses
        // display power management for the duration of the window.
        s_commissioning_window_open = true;
        app_refresh_ui_mode();
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        s_commissioning_window_open = false;
        app_refresh_ui_mode();
        break;

    case chip::DeviceLayer::DeviceEventType::kBindingsChangedViaCluster: {
        ESP_LOGI(TAG, "Binding entry changed");
        // Show or hide the controls depending on whether a light is now bound
        // (on any fabric).
        app_refresh_ui_mode();
        // Recompute the Indoor page's per-cluster source of truth (a device may have been added or
        // removed, changing which endpoint is first in the binding table), then (re)subscribe to any
        // bound sensor so the Indoor / weather pages update. Independent of the light subscription below.
        app_sensor_bindings_changed();
        subscribe_to_sensor_bindings();

        if (do_subscribe) {
            for (const auto  &binding : chip::app::Clusters::Binding::Table::GetInstance()) {
                ESP_LOGI(
                    TAG,
                    "Read cached binding type=%d fabrixIndex=%d nodeId=0x" ChipLogFormatX64
                    " groupId=%d local endpoint=%d remote endpoint=%d cluster=" ChipLogFormatMEI,
                    binding.type, binding.fabricIndex, ChipLogValueX64(binding.nodeId), binding.groupId, binding.local,
                    binding.remote, ChipLogValueMEI(binding.clusterId.value_or(0)));
                if (binding.type == chip::app::Clusters::Binding::MATTER_UNICAST_BINDING && event->BindingsChanged.fabricIndex == binding.fabricIndex) {
                    ESP_LOGI(
                        TAG,
                        "Matched accessingFabricIndex with nodeId=0x" ChipLogFormatX64,
                        ChipLogValueX64(binding.nodeId));

                    auto &server = chip::Server::GetInstance();

                    // Subscribe to the bound light's On/Off state (drives the bulb icon).
                    client::request_handle_t onoff_req;
                    onoff_req.type = esp_matter::client::SUBSCRIBE_ATTR;
                    onoff_req.attribute_path = {binding.remote, chip::app::Clusters::OnOff::Id,
                                                chip::app::Clusters::OnOff::Attributes::OnOff::Id};
                    client::connect(server.GetCASESessionManager(), binding.fabricIndex, binding.nodeId,
                                    &onoff_req);

                    // Subscribe to the bound light's brightness (drives the slider).
                    client::request_handle_t level_req;
                    level_req.type = esp_matter::client::SUBSCRIBE_ATTR;
                    level_req.attribute_path = {binding.remote, chip::app::Clusters::LevelControl::Id,
                                                chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id};
                    client::connect(server.GetCASESessionManager(), binding.fabricIndex, binding.nodeId,
                                    &level_req);

                    // Subscribe to the bound light's color temperature (drives the CT slider).
                    client::request_handle_t ct_req;
                    ct_req.type = esp_matter::client::SUBSCRIBE_ATTR;
                    ct_req.attribute_path = {binding.remote, chip::app::Clusters::ColorControl::Id,
                                             chip::app::Clusters::ColorControl::Attributes::ColorTemperatureMireds::Id};
                    client::connect(server.GetCASESessionManager(), binding.fabricIndex, binding.nodeId,
                                    &ct_req);
                    break;
                }
            }
            do_subscribe = false;
        }
    }
    break;

    default:
        break;
    }
}

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    if (type == PRE_UPDATE) {
        /* Handle the attribute updates here. */
    }

    return ESP_OK;
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize NVS. If the partition is full or was written by a different NVS version
     * (common when reflashing a board that ran other firmware), erase and retry, otherwise
     * Matter can't persist fabrics. */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erasing (%s) — erasing and re-initialising", esp_err_to_name(nvs_err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    if (nvs_err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(nvs_err));
    }

    /* Initialize driver */
    app_driver_handle_t switch_handle = app_driver_switch_init();
    if (switch_handle) {
        app_reset_button_register(switch_handle);
    } else {
        ESP_LOGW(TAG, "No button handle — factory-reset button unavailable (re-flash to reset)");
    }

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));
    endpoint_t *root_node_ep = endpoint::get_first(node);
    ABORT_APP_ON_FAILURE(root_node_ep != nullptr, ESP_LOGE(TAG, "Failed to find root node endpoint"));

    cluster::time_synchronization::config_t time_sync_cfg;
    static AppTimeSyncDelegate time_sync_delegate;
    time_sync_cfg.delegate = &time_sync_delegate;
    cluster_t *time_sync_cluster = cluster::time_synchronization::create(root_node_ep, &time_sync_cfg, CLUSTER_FLAG_SERVER);
    ABORT_APP_ON_FAILURE(time_sync_cluster != nullptr, ESP_LOGE(TAG, "Failed to create time_sync_cluster"));

    // Enable ONLY the TimeZone feature. Advertising TimeSyncClient as well would signal "I fetch
    // my own time from a trusted source", which stops controllers pushing SetUTCTime to us.
    cluster::time_synchronization::feature::time_zone::config_t tz_cfg;
    cluster::time_synchronization::feature::time_zone::add(time_sync_cluster, &tz_cfg);

    color_dimmer_switch::config_t switch_config;
    endpoint_t *endpoint = color_dimmer_switch::create(node, &switch_config, ENDPOINT_FLAG_NONE, switch_handle);
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create color dimmer switch endpoint"));

    /* Add group cluster to the switch endpoint */
    cluster::groups::config_t groups_config;
    cluster::groups::create(endpoint, &groups_config, CLUSTER_FLAG_SERVER | CLUSTER_FLAG_CLIENT);

    /* Add temperature and relative-humidity measurement clusters as clients so this
       endpoint can bind to and subscribe to attribute reports from a sensor device. */
    cluster::temperature_measurement::config_t temp_config;
    cluster::temperature_measurement::create(endpoint, &temp_config, CLUSTER_FLAG_CLIENT);

    cluster::relative_humidity_measurement::config_t humidity_config;
    cluster::relative_humidity_measurement::create(endpoint, &humidity_config, CLUSTER_FLAG_CLIENT);

    /* Air-quality monitor clusters (e.g. IKEA Alpstuga): overall rating plus CO2 and PM2.5
       concentration, added as clients so the switch can bind to and subscribe to them. */
    cluster::air_quality::config_t aq_config;
    cluster::air_quality::create(endpoint, &aq_config, CLUSTER_FLAG_CLIENT);

    cluster::carbon_dioxide_concentration_measurement::config_t co2_config;
    cluster::carbon_dioxide_concentration_measurement::create(endpoint, &co2_config, CLUSTER_FLAG_CLIENT);

    cluster::pm25_concentration_measurement::config_t pm25_config;
    cluster::pm25_concentration_measurement::create(endpoint, &pm25_config, CLUSTER_FLAG_CLIENT);

    /* Label clusters as clients so this endpoint can bind to the weather aggregator: its
       UserLabel carries the text forecast (and is the marker that flags the node as the
       weather device), and its FixedLabels carry the per-endpoint role + type/location. */
    cluster::user_label::config_t user_label_config;
    cluster::user_label::create(endpoint, &user_label_config, CLUSTER_FLAG_CLIENT);

    cluster::fixed_label::config_t fixed_label_config;
    cluster::fixed_label::create(endpoint, &fixed_label_config, CLUSTER_FLAG_CLIENT);

    /* BooleanState client cluster so this endpoint can bind to a contact sensor and a water-leak
       detector; both surface as a transient alert overlay rather than a page. No local
       BooleanStateConfiguration client cluster is created — it isn't needed to subscribe to the
       remote one, and the leak detector is told apart at runtime by the reports it sends. */
    cluster::boolean_state::config_t boolean_state_config;
    cluster::boolean_state::create(endpoint, &boolean_state_config, CLUSTER_FLAG_CLIENT);

    switch_endpoint_id = endpoint::get_id(endpoint);
    ESP_LOGI(TAG, "Switch created with endpoint_id %d", switch_endpoint_id);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

#if CONFIG_DYNAMIC_PASSCODE_COMMISSIONABLE_DATA_PROVIDER
    /* This should be called before esp_matter::start() */
    esp_matter::set_custom_commissionable_data_provider(&g_dynamic_passcode_provider);
#endif

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::factoryreset_register_commands();
    esp_matter::console::init();
#endif

    /* Generate the onboarding QR payload + manual pairing code for the pairing
     * screen (BLE rendezvous, matching what is logged at boot). */
    char qr_payload[96] = {0};
    char manual_code[32] = {0};
    {
        chip::MutableCharSpan qr_span(qr_payload);
        if (GetQRCode(qr_span, chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE)) ==
            CHIP_NO_ERROR && qr_span.size() < sizeof(qr_payload)) {
            qr_payload[qr_span.size()] = '\0';
        }
        chip::MutableCharSpan man_span(manual_code);
        if (GetManualPairingCode(man_span,
                                 chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE)) ==
            CHIP_NO_ERROR && man_span.size() < sizeof(manual_code)) {
            manual_code[man_span.size()] = '\0';
        }
        ESP_LOGI(TAG, "Onboarding QR: %s  manual: %s", qr_payload, manual_code);
    }

    /* Bring up the AMOLED display + touch and draw the UI, after Matter start so the client
     * path is ready. The UI starts in pairing mode. Pass the commissioning state so the display
     * driver can leave extra heap free for the pairing handshake on an uncommissioned device. */
    bool already_commissioned;
    {
        lock::ScopedChipStackLock lock(portMAX_DELAY);
        already_commissioned = (chip::Server::GetInstance().GetFabricTable().FabricCount() > 0);
    }
    err = light_ui_start(qr_payload, manual_code, already_commissioned);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start light UI, err:%d", err);
    } else {
        /* Set the initial screen from the current commissioning + binding state
         * (e.g. already commissioned + bound after a reboot). */
        lock::ScopedChipStackLock lock(portMAX_DELAY);
        app_refresh_ui_mode();
    }

    /* Bring up the ES8311 codec + speaker for the leak-alert chime. Must follow light_ui_start()
     * so the shared I2C bus + TCA9554 IO expander are up. A failure only disables the chime. */
    if (audio_init() != ESP_OK) {
        ESP_LOGW(TAG, "Audio init failed — alert chime disabled");
    }

    /* Kick off the sensor subscription once at startup. This is the reliable trigger on a
     * reboot: on Thread kInterfaceIpAddressChanged fires before our handler is registered. */
    CHIP_ERROR kickoff_err = chip::DeviceLayer::PlatformMgr().ScheduleWork(sensor_kickoff_work, 0);
    if (kickoff_err != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to schedule sensor subscription kickoff: %s", chip::ErrorStr(kickoff_err));
    }

    /* Apply the controller's persisted timezone/DST at boot, then re-check every 10 min so DST
     * switches automatically. UTC itself comes from the controller's SetUTCTime push. */
    CHIP_ERROR off_err = chip::DeviceLayer::PlatformMgr().ScheduleWork(app_offset_recompute_work, 0);
    if (off_err != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to schedule initial local-offset recompute: %s", chip::ErrorStr(off_err));
    }

    static esp_timer_handle_t dst_timer = nullptr;
    const esp_timer_create_args_t dst_timer_args = { .callback = app_dst_refresh_timer_cb,
                                                     .name     = "dst_refresh" };
    if (esp_timer_create(&dst_timer_args, &dst_timer) == ESP_OK) {
        esp_timer_start_periodic(dst_timer, 600ULL * 1000 * 1000);  // every 10 min
    }
}
