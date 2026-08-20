/*
   ESP32-H2 Matter-over-Thread weather device (Thread Border Router board).

   The Matter half of the TBR weather device: it publishes the readings the AMOLED
   display binds to, and takes those readings from the on-board ESP32-S3 (which
   owns Wi-Fi and fetches the forecast) over the inter-chip UART. The structure the
   display expects is:

     Aggregator endpoint
       FixedLabel: type=weather, location=<place>    (set by the S3)
       UserLabel:  condition=<short forecast text>   (set by the S3)
       ├─ Bridged Temperature Sensor  FixedLabel role=min      MeasuredValue
       ├─ Bridged Temperature Sensor  FixedLabel role=max      MeasuredValue
       └─ Bridged Temperature Sensor  FixedLabel role=current  MeasuredValue

   Line protocol (newline-delimited):
     S3 → H2:  WX <min> <max> <cur> <condition…>     temps per unit, "-" = null
               LOC <location…>
     H2 → S3:  READY            (on boot)
               NEED_WEATHER      (until the first WX arrives)

   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this software is
   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.
*/

#include <math.h>
#include <string.h>

#include <app/server/Server.h>
#include <app/reporting/reporting.h>            // MatterReportingAttributeChangeCallback
#include <driver/uart.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_ota.h>
#include <esp_matter_providers.h>                // esp_matter::set_custom_device_info_provider
#include <app/clusters/temperature-measurement-server/TemperatureMeasurementCluster.h>
#include <data_model_provider/esp_matter_data_model_provider.h>
#include <nvs_flash.h>
#include <platform/DeviceInfoProvider.h>
#include <platform/PlatformManager.h>

#include <app_openthread_config.h>
#include <app_reset.h>
#include <bsp/esp-bsp.h>
#include <common_macros.h>

static const char *TAG = "app_main";

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;
using chip::EndpointId;

// ─────────────────────────────────────────────────────────────────────────────
// Endpoint ids, filled in during setup.
// ─────────────────────────────────────────────────────────────────────────────
static uint16_t s_agg_ep = 0;
static uint16_t s_min_ep = 0;
static uint16_t s_max_ep = 0;
static uint16_t s_cur_ep = 0;
static volatile bool s_have_weather = false;

// ─────────────────────────────────────────────────────────────────────────────
// Label storage. FixedLabel/UserLabel values are served through a
// DeviceInfoProvider rather than ordinary attribute storage, so the values the S3
// can change at runtime (location, condition) live in mutable buffers here and we
// report a change afterwards so bound clients re-read them.
// ─────────────────────────────────────────────────────────────────────────────
struct FixedLabelRow {
    EndpointId endpoint;
    const char *label;      // static
    chip::CharSpan value;   // may point at a mutable buffer (location)
};

static char s_location_buf[17] = "";   // Matter FixedLabel value ≤16 chars
static char s_condition_buf[17] = "unknown";

static FixedLabelRow s_fixed_labels[5];
static size_t s_fixed_label_count = 0;
static size_t s_location_row = 0;       // index into s_fixed_labels of the location row

static void fixed_labels_build()
{
    size_t i = 0;
    s_fixed_labels[i++] = { s_agg_ep, "type", chip::CharSpan::fromCharString("weather") };
    s_location_row = i;
    s_fixed_labels[i++] = { s_agg_ep, "location", chip::CharSpan(s_location_buf, 0) };
    s_fixed_labels[i++] = { s_min_ep, "role", chip::CharSpan::fromCharString("min") };
    s_fixed_labels[i++] = { s_max_ep, "role", chip::CharSpan::fromCharString("max") };
    s_fixed_labels[i++] = { s_cur_ep, "role", chip::CharSpan::fromCharString("current") };
    s_fixed_label_count = i;
}

// Serves our fixed labels (type/location/role) and the single "condition" user
// label from the RAM buffers above. Subclasses the abstract DeviceInfoProvider
// directly so no factory-data config is needed; RAM is enough because the S3
// re-sends every value on (re)connect.
class WeatherDeviceInfoProvider : public chip::DeviceLayer::DeviceInfoProvider {
public:
    class FixedIter : public FixedLabelIterator {
    public:
        explicit FixedIter(EndpointId ep) : mEndpoint(ep) {}
        size_t Count() override
        {
            size_t n = 0;
            for (size_t i = 0; i < s_fixed_label_count; i++)
                if (s_fixed_labels[i].endpoint == mEndpoint) n++;
            return n;
        }
        bool Next(FixedLabelType &out) override
        {
            while (mIndex < s_fixed_label_count) {
                const FixedLabelRow &row = s_fixed_labels[mIndex++];
                if (row.endpoint == mEndpoint) {
                    out.label = chip::CharSpan::fromCharString(row.label);
                    out.value = row.value;
                    return true;
                }
            }
            return false;
        }
        void Release() override { chip::Platform::Delete(this); }
    private:
        EndpointId mEndpoint;
        size_t mIndex = 0;
    };

    class UserIter : public UserLabelIterator {
    public:
        explicit UserIter(EndpointId ep) : mEndpoint(ep) {}
        size_t Count() override { return mEndpoint == s_agg_ep ? 1 : 0; }
        bool Next(UserLabelType &out) override
        {
            if (mEndpoint == s_agg_ep && !mDone) {
                mDone = true;
                out.label = chip::CharSpan::fromCharString("condition");
                out.value = chip::CharSpan::fromCharString(s_condition_buf);
                return true;
            }
            return false;
        }
        void Release() override { chip::Platform::Delete(this); }
    private:
        EndpointId mEndpoint;
        bool mDone = false;
    };

    FixedLabelIterator *IterateFixedLabel(EndpointId endpoint) override
    {
        return chip::Platform::New<FixedIter>(endpoint);
    }
    UserLabelIterator *IterateUserLabel(EndpointId endpoint) override
    {
        return chip::Platform::New<UserIter>(endpoint);
    }
    SupportedLocalesIterator *IterateSupportedLocales() override { return nullptr; }
    SupportedCalendarTypesIterator *IterateSupportedCalendarTypes() override { return nullptr; }

protected:
    // We never accept user-label writes (condition is served from our buffer), so
    // these are inert — present only to make the class concrete.
    CHIP_ERROR SetUserLabelAt(EndpointId, size_t, const UserLabelType &) override { return CHIP_NO_ERROR; }
    CHIP_ERROR DeleteUserLabelAt(EndpointId, size_t) override { return CHIP_NO_ERROR; }
    CHIP_ERROR SetUserLabelLength(EndpointId, size_t) override { return CHIP_NO_ERROR; }
    CHIP_ERROR GetUserLabelLength(EndpointId, size_t &val) override { val = 0; return CHIP_NO_ERROR; }
};

static WeatherDeviceInfoProvider s_device_info_provider;

// ─────────────────────────────────────────────────────────────────────────────
// Attribute / label updates — all run on the Matter thread (CHIP stack lock).
// ─────────────────────────────────────────────────────────────────────────────

// centi-°C, or null. Matter stores TemperatureMeasurement in 0.01 °C units.
static void apply_temp(uint16_t endpoint, bool has_value, float celsius)
{
    chip::DeviceLayer::SystemLayer().ScheduleLambda([endpoint, has_value, celsius]() {
        // In this esp-matter revision, TemperatureMeasurement is served by a
        // registered CHIP cluster object; attribute::update() writes esp-matter's
        // separate copy and never reaches it, so reads stay null. Set the value on
        // the registered cluster instance directly — that's the source reads use,
        // and SetMeasuredValue also notifies subscribers.
        chip::app::DataModel::Nullable<int16_t> nv;
        if (has_value) nv.SetNonNull(static_cast<int16_t>(lroundf(celsius * 100.0f)));

        chip::app::ServerClusterInterface *sc =
            esp_matter::data_model::provider::get_instance().registry().Get(
                chip::app::ConcreteClusterPath(endpoint, TemperatureMeasurement::Id));
        if (sc == nullptr) {
            ESP_LOGW(TAG, "temp ep=%u: TemperatureMeasurement cluster not registered", endpoint);
            return;
        }
        CHIP_ERROR err = static_cast<chip::app::Clusters::TemperatureMeasurementCluster *>(sc)->SetMeasuredValue(nv);
        if (err != CHIP_NO_ERROR) {
            ESP_LOGW(TAG, "temp ep=%u SetMeasuredValue failed: %" CHIP_ERROR_FORMAT, endpoint, err.Format());
        }
    });
}

static void apply_condition(const char *text)
{
    strncpy(s_condition_buf, text, sizeof(s_condition_buf) - 1);
    s_condition_buf[sizeof(s_condition_buf) - 1] = '\0';
    // The provider serves the condition straight from s_condition_buf; just tell
    // subscribers the UserLabel list changed so they re-read it.
    chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t) {
        MatterReportingAttributeChangeCallback(s_agg_ep, UserLabel::Id,
                                               UserLabel::Attributes::LabelList::Id);
    }, 0);
}

static void apply_location(const char *text)
{
    strncpy(s_location_buf, text, sizeof(s_location_buf) - 1);
    s_location_buf[sizeof(s_location_buf) - 1] = '\0';
    chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t) {
        s_fixed_labels[s_location_row].value = chip::CharSpan(s_location_buf, strlen(s_location_buf));
        MatterReportingAttributeChangeCallback(s_agg_ep, FixedLabel::Id,
                                               FixedLabel::Attributes::LabelList::Id);
    }, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Serial protocol from the S3.
// ─────────────────────────────────────────────────────────────────────────────

// Parse one decimal temperature field: "-" means null.
static bool parse_temp(const char *tok, float *out)
{
    if (tok == nullptr || strcmp(tok, "-") == 0) return false;
    *out = strtof(tok, nullptr);
    return true;
}

static void handle_line(char *line)
{
    if (strncmp(line, "WX ", 3) == 0) {
        // WX <min> <max> <cur> <condition…>
        char *save = nullptr;
        strtok_r(line, " ", &save);                       // "WX"
        char *min_s = strtok_r(nullptr, " ", &save);
        char *max_s = strtok_r(nullptr, " ", &save);
        char *cur_s = strtok_r(nullptr, " ", &save);
        char *cond  = save;                               // rest of the line (may be empty)

        float v = 0.0f;
        apply_temp(s_min_ep, parse_temp(min_s, &v), v);
        apply_temp(s_max_ep, parse_temp(max_s, &v), v);
        apply_temp(s_cur_ep, parse_temp(cur_s, &v), v);
        if (cond && *cond && strcmp(cond, "-") != 0) apply_condition(cond);

        s_have_weather = true;
        printf("ACK WX\n");
    } else if (strncmp(line, "LOC ", 4) == 0) {
        apply_location(line + 4);
        printf("ACK LOC\n");
    }
    // Unknown lines are ignored (the S3 does the same for our log output).
}

// On the TBR board the S3↔H2 link is the H2's UART0 (H2_TXD0/H2_RXD0, default
// pins). The H2's console is on USB-Serial-JTAG, so UART0 is free for us; ESP_LOG
// still goes to USB serial, keeping this link clean for the protocol.
#define S3_UART_PORT UART_NUM_0

static void s3_send(const char *s)
{
    uart_write_bytes(S3_UART_PORT, s, strlen(s));
    uart_write_bytes(S3_UART_PORT, "\n", 1);
}

static void serial_task(void *)
{
    const uart_config_t uc = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(S3_UART_PORT, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(S3_UART_PORT, &uc));
    // UART0 default pins (H2: U0TXD=GPIO24, U0RXD=GPIO23) are wired to the S3.
    ESP_ERROR_CHECK(uart_set_pin(S3_UART_PORT, 24, 23, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Announce ourselves so the S3 does an immediate full sync, and keep asking
    // for weather until it arrives (covers "H2 powered up before the S3").
    s3_send("READY");
    s3_send("NEED_WEATHER");

    char line[192];
    size_t len = 0;
    TickType_t last_need = xTaskGetTickCount();
    uint8_t ch;
    while (true) {
        int n = uart_read_bytes(S3_UART_PORT, &ch, 1, pdMS_TO_TICKS(1000));
        if (n == 1) {
            if (ch == '\n' || ch == '\r') {
                if (len > 0) { line[len] = '\0'; handle_line(line); len = 0; }
            } else if (len < sizeof(line) - 1) {
                line[len++] = (char) ch;
            }
        }
        if (!s_have_weather &&
            (xTaskGetTickCount() - last_need) > pdMS_TO_TICKS(30000)) {
            s3_send("NEED_WEATHER");
            last_need = xTaskGetTickCount();
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Matter plumbing (mostly boilerplate from the esp-matter sensors example).
// ─────────────────────────────────────────────────────────────────────────────
static esp_err_t factory_reset_button_register()
{
    button_handle_t push_button;
    esp_err_t err = bsp_iot_button_create(&push_button, NULL, BSP_BUTTON_NUM);
    VerifyOrReturnError(err == ESP_OK, err);
    return app_reset_button_register(push_button);
}

static void open_commissioning_window_if_necessary()
{
    VerifyOrReturn(chip::Server::GetInstance().GetFabricTable().FabricCount() == 0);
    chip::CommissioningWindowManager &commissionMgr =
        chip::Server::GetInstance().GetCommissioningWindowManager();
    VerifyOrReturn(commissionMgr.IsCommissioningWindowOpen() == false);
    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(
        chip::System::Clock::Seconds16(300), chip::CommissioningWindowAdvertisement::kDnssdOnly);
    if (err != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
    }
}

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Fabric removed successfully");
        open_commissioning_window_if_necessary();
        break;
    default:
        break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id,
                                       uint8_t effect_id, uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u", type, effect_id);
    return ESP_OK;
}

// Read-only device — we don't expect writes to our attributes.
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id,
                                         uint32_t cluster_id, uint32_t attribute_id,
                                         esp_matter_attr_val_t *val, void *priv_data)
{
    return ESP_OK;
}

// Create one bridged Temperature Sensor under the aggregator and return its id.
static uint16_t create_bridged_temp(node_t *node, endpoint_t *aggregator)
{
    // A complete Temperature Sensor device (Identify + TemperatureMeasurement),
    // MeasuredValue defaults to null until the first update from the S3.
    temperature_sensor::config_t ts_cfg;
    endpoint_t *ep = temperature_sensor::create(node, &ts_cfg, ENDPOINT_FLAG_BRIDGE, NULL);
    ABORT_APP_ON_FAILURE(ep != nullptr, ESP_LOGE(TAG, "Failed to create temp sensor endpoint"));

    // Make it a proper bridged node (BridgedDeviceBasicInformation + device type).
    cluster::bridged_device_basic_information::config_t bdbi_cfg;
    cluster::bridged_device_basic_information::create(ep, &bdbi_cfg, CLUSTER_FLAG_SERVER);
    ABORT_APP_ON_FAILURE(
        endpoint::add_device_type(ep, bridged_node::get_device_type_id(),
                                  bridged_node::get_device_type_version()) == ESP_OK,
        ESP_LOGE(TAG, "Failed to add bridged_node device type"));

    // FixedLabel cluster: the "role" value itself is served by the provider.
    cluster::fixed_label::config_t fl_cfg;
    cluster::fixed_label::create(ep, &fl_cfg, CLUSTER_FLAG_SERVER);

    ABORT_APP_ON_FAILURE(endpoint::set_parent_endpoint(ep, aggregator) == ESP_OK,
                         ESP_LOGE(TAG, "Failed to set parent endpoint"));
    return endpoint::get_id(ep);
}

extern "C" void app_main()
{
    nvs_flash_init();

    esp_err_t err = factory_reset_button_register();
    ABORT_APP_ON_FAILURE(ESP_OK == err, ESP_LOGE(TAG, "Failed to init reset button, err:%d", err));

    // Root node (endpoint 0).
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // Aggregator endpoint: carries the weather FixedLabels (type/location) and the
    // UserLabel (condition). Values come from the DeviceInfoProvider.
    aggregator::config_t agg_cfg;
    endpoint_t *aggregator = aggregator::create(node, &agg_cfg, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(aggregator != nullptr, ESP_LOGE(TAG, "Failed to create aggregator"));
    cluster::fixed_label::config_t agg_fl_cfg;
    cluster::fixed_label::create(aggregator, &agg_fl_cfg, CLUSTER_FLAG_SERVER);
    cluster::user_label::config_t agg_ul_cfg;
    cluster::user_label::create(aggregator, &agg_ul_cfg, CLUSTER_FLAG_SERVER);
    s_agg_ep = endpoint::get_id(aggregator);

    // The three bridged temperature sensors.
    s_min_ep = create_bridged_temp(node, aggregator);
    s_max_ep = create_bridged_temp(node, aggregator);
    s_cur_ep = create_bridged_temp(node, aggregator);
    ESP_LOGI(TAG, "Endpoints: agg=%u min=%u max=%u cur=%u", s_agg_ep, s_min_ep, s_max_ep, s_cur_ep);

    // Now that endpoint ids are known, publish the fixed-label table and install
    // our device info provider (must happen before esp_matter::start()).
    fixed_labels_build();
    esp_matter::set_custom_device_info_provider(&s_device_info_provider);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    esp_openthread_platform_config_t ot_config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&ot_config);
#endif

    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    // Seed the initial "unknown" condition so the UserLabel list is non-empty.
    apply_condition(s_condition_buf);

    // Start the link to the S3.
    xTaskCreate(serial_task, "weather_serial", 4096, NULL, 5, NULL);
}
