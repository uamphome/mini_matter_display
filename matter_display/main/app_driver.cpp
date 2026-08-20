/*
   Matter Display — Matter client side.

   Outgoing commands to the bound light(s) (OnOff / LevelControl / ColorControl), and the incoming
   subscriptions from bound sensor nodes: CASE session setup per node, decoding of the temperature,
   humidity, air-quality, weather and boolean-sensor reports, and routing each reading to the right
   UI page. Also owns the physical button and the factory reset.

   This code is in the Public Domain (or CC0 licensed, at your option.) Unless required by
   applicable law or agreed to in writing, this software is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#include "esp_matter_client.h"
#include <cstddef>
#include <cstdio>
#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include "bsp/esp-bsp.h"

#include <app_priv.h>
#if CONFIG_ENABLE_ICD_SERVER
#include <app/icd/server/ICDNotifier.h>
#endif

#include <app_reset.h>
#include "light_ui.h"
#include "audio.h"

#include <app/server/Server.h>
#include <lib/core/Optional.h>

#include <cinttypes>
#include <app/AttributePathParams.h>
#include <app/ConcreteAttributePath.h>
#include <app/clusters/bindings/binding-table.h>
#include <app/data-model/Decode.h>
#include <app/data-model/Nullable.h>
#include <lib/core/TLVReader.h>
#include <app/CASESessionManager.h>
#include <app/OperationalSessionSetup.h>
#include <lib/core/CHIPCallback.h>
#include <lib/core/ScopedNodeId.h>
#include <messaging/ExchangeMgr.h>

using chip::kInvalidClusterId;
static constexpr chip::CommandId kInvalidCommandId = 0xFFFF'FFFF;

using namespace chip::app::Clusters;
using namespace esp_matter;
using namespace esp_matter::cluster;

static const char *TAG = "app_driver";
extern uint16_t switch_endpoint_id;

class MyReadClientCallback : public chip::app::ReadClient::Callback {
public:
    void OnAttributeData(const chip::app::ConcreteDataAttributePath &aPath,
                         chip::TLV::TLVReader *aReader,
                         const chip::app::StatusIB &aStatus) override
    {
        // Handle the attribute data
        if (aPath.mClusterId == chip::app::Clusters::OnOff::Id) {
            if (aPath.mAttributeId == chip::app::Clusters::OnOff::Attributes::OnOff::Id) {
                bool on_off = false;
                if (aReader && aReader->Get(on_off) == CHIP_NO_ERROR) {
                    ESP_LOGI(TAG, "Received OnOff attribute: %s", on_off ? "ON" : "OFF");
                    // Reflect the bound light's real state on the touch screen icon.
                    light_ui_set_state(on_off);
                } else {
                    ESP_LOGI(TAG, "Received OnOff attribute (decode failed)");
                }
            }
        } else if (aPath.mClusterId == chip::app::Clusters::LevelControl::Id) {
            if (aPath.mAttributeId == chip::app::Clusters::LevelControl::Attributes::CurrentLevel::Id) {
                // CurrentLevel is nullable; Get() fails on a Null value, which we skip.
                uint8_t level = 0;
                if (aReader && aReader->Get(level) == CHIP_NO_ERROR) {
                    ESP_LOGI(TAG, "Received CurrentLevel: %u", level);
                    // Reflect the bound light's real brightness on the slider.
                    light_ui_set_brightness(level);
                }
            }
        } else if (aPath.mClusterId == chip::app::Clusters::ColorControl::Id) {
            if (aPath.mAttributeId == chip::app::Clusters::ColorControl::Attributes::ColorTemperatureMireds::Id) {
                uint16_t mireds = 0;
                if (aReader && aReader->Get(mireds) == CHIP_NO_ERROR && mireds != 0) {
                    ESP_LOGI(TAG, "Received ColorTemperatureMireds: %u", mireds);
                    // Reflect the bound light's real color temperature on the CT slider.
                    light_ui_set_color_temp(mireds);
                }
            }
        }
    }

    void OnEventData(const chip::app::EventHeader &aEventHeader, chip::TLV::TLVReader * apData,
                     const chip::app::StatusIB *aStatus) override
    {
        // Handle event data
    }

    void OnError(CHIP_ERROR aError) override
    {
        // Handle the error
        ESP_LOGI(TAG, "ReadClient Error: %s", ErrorStr(aError));
    }

    void OnDone(chip::app::ReadClient * apReadClient) override
    {
        // Cleanup after done
        ESP_LOGI(TAG, "ReadClient Done");
    }
};
MyReadClientCallback readClientCb;

void app_client_subscribe_command_callback(client::peer_device_t *peer_device, client::request_handle_t *req_handle,
                                           void *priv_data)
{
    /* Subscription intervals for the bound light's state. min_interval caps how fast the bulb
     * icon can react to someone toggling the light elsewhere; max_interval is the liveness
     * heartbeat that bounds how long a dead subscription goes unnoticed. */
    uint16_t min_interval = 1;
    uint16_t max_interval = 300;
    bool keep_subscription = true;
    bool auto_resubscribe = true;
    chip::Platform::ScopedMemoryBufferWithSize<chip::app::AttributePathParams> attrb_path;
    attrb_path.Alloc(1);
    client::interaction::subscribe::send_request(peer_device, &req_handle->attribute_path, attrb_path.AllocatedSize(),
                                                 &req_handle->event_path, 0, min_interval, max_interval, keep_subscription,
                                                 auto_resubscribe, readClientCb);
}

using chip::kInvalidAttributeId;

/* Copy a (non-null-terminated) CharSpan into a C string buffer, truncating if needed. */
static void span_to_cstr(const chip::CharSpan &span, char *buf, size_t buf_size)
{
    size_t len = span.size();
    if (len >= buf_size) {
        len = buf_size - 1;
    }
    if (len > 0 && span.data()) {
        memcpy(buf, span.data(), len);
    }
    buf[len] = '\0';
}

/* The weather aggregator's temperature endpoints are tagged with a FixedLabel "role"
 * (min/max/current). The temperature report and the role can arrive in either order, so per
 * endpoint we buffer both and push to the weather page only once both are known. Weather temps
 * never reach the climate page — routing is decided by the subscription's weather_mode flag. */
enum class weather_role_t : uint8_t { none, min, max, current };
#define MAX_WEATHER_EPS 12
static struct weather_ep_t {
    chip::NodeId node;   // every temperature endpoint buffers its latest reading here, keyed by
    chip::EndpointId ep; // (node, ep) — plain sensors and the weather aggregator can share ep numbers
    weather_role_t role;
    bool have_temp;
    float celsius;
} s_weather_eps[MAX_WEATHER_EPS];
static size_t s_weather_ep_count = 0;

static weather_ep_t *weather_ep_get(chip::NodeId node, chip::EndpointId ep)
{
    for (size_t i = 0; i < s_weather_ep_count; i++) {
        if (s_weather_eps[i].node == node && s_weather_eps[i].ep == ep) {
            return &s_weather_eps[i];
        }
    }
    if (s_weather_ep_count < MAX_WEATHER_EPS) {
        s_weather_eps[s_weather_ep_count] = { node, ep, weather_role_t::none, false, 0.0f };
        return &s_weather_eps[s_weather_ep_count++];
    }
    return nullptr;
}

/* Push an endpoint's buffered temperature to the weather page once its role is known. */
static void weather_ep_flush(const weather_ep_t *e)
{
    if (!e || !e->have_temp) {
        return;
    }
    switch (e->role) {
    case weather_role_t::min:     light_ui_set_weather_min(e->celsius); break;
    case weather_role_t::max:     light_ui_set_weather_max(e->celsius); break;
    case weather_role_t::current: light_ui_set_weather_current(e->celsius); break;
    default: break;  // role not known yet — keep buffered
    }
}

/* ── Node classification: weather aggregator vs plain sensor ───────────────────────────────────
 * Wildcard bindings look identical for both, so nodes are classified by content: a weather marker
 * (FixedLabel role/type/location, UserLabel condition) means weather aggregator; a humidity /
 * air-quality / CO2 / PM2.5 cluster means plain room sensor. A node that has only sent a bare
 * temperature stays 'unknown' until a short debounce resolves it to a plain sensor. Runs on the
 * Matter task (from OnAttributeData or a SystemLayer timer). */
enum class node_class_t : uint8_t { unknown, weather, sensor };
#define MAX_CLS_NODES 8
static struct node_cls_t { chip::NodeId node; node_class_t cls; bool in_use; } s_node_cls[MAX_CLS_NODES];

static node_cls_t *node_cls_get(chip::NodeId n)
{
    for (auto &c : s_node_cls) if (c.in_use && c.node == n) return &c;
    for (auto &c : s_node_cls) if (!c.in_use) { c.node = n; c.cls = node_class_t::unknown; c.in_use = true; return &c; }
    return nullptr;
}

/* ── Indoor page "source of truth": the first instance of each cluster in the binding table ─────
 * Several bound endpoints can expose the same sensor cluster, so each Indoor card shows one
 * deterministic source: the endpoint appearing first in the binding table. Reports from any other
 * endpoint are ignored. Cleared on a binding-table change (app_sensor_bindings_changed). */
enum indoor_metric_t { IM_TEMP = 0, IM_HUMIDITY, IM_AQ, IM_CO2, IM_PM25, IM_COUNT };
static struct indoor_src_t { bool have; chip::NodeId node; chip::EndpointId ep; int rank; } s_indoor_src[IM_COUNT];

#define BINDING_RANK_NONE 0x7fffffff

/* Binding-table rank of a target endpoint = its index among unicast bindings. BINDING_RANK_NONE
 * means it isn't in the table (a stale report from an unbound device). */
static int binding_rank(chip::NodeId node, chip::EndpointId ep)
{
    int idx = 0;
    for (const auto &b : chip::app::Clusters::Binding::Table::GetInstance()) {
        if (b.type != chip::app::Clusters::Binding::MATTER_UNICAST_BINDING) continue;
        if (b.nodeId == node && b.remote == ep) return idx;
        idx++;
    }
    return BINDING_RANK_NONE;
}

/* True if (node, ep) is the source for metric m: adopted when no source is set yet or when it
 * ranks earlier than the current one. Stale (unbound) endpoints are always rejected. */
static bool indoor_accept(indoor_metric_t m, chip::NodeId node, chip::EndpointId ep)
{
    int rank = binding_rank(node, ep);
    if (rank == BINDING_RANK_NONE) return false;
    indoor_src_t &s = s_indoor_src[m];
    if (!s.have || rank < s.rank) { s.have = true; s.node = node; s.ep = ep; s.rank = rank; return true; }
    return s.node == node && s.ep == ep;
}

/* Flush a node's buffered temperature(s) to the Indoor climate card, once it's known to be a plain
 * sensor. Routed through the source-of-truth resolver. */
static void flush_node_temps_to_climate(chip::NodeId node)
{
    for (size_t i = 0; i < s_weather_ep_count; i++) {
        if (s_weather_eps[i].node == node && s_weather_eps[i].have_temp &&
            indoor_accept(IM_TEMP, node, s_weather_eps[i].ep)) {
            light_ui_set_temperature(s_weather_eps[i].celsius);
        }
    }
}

#define TEMP_CLASSIFY_DEBOUNCE_MS 2500   /* matches the boolean-sensor debounce; covers the aggregator's label burst */
static bool s_temp_debounce_armed = false;

// Debounce: any node still unclassified once the initial report burst has settled has shown no
// weather marker, so it is a plain sensor — classify it and flush its buffered temperature to Indoor.
static void temp_classify_timer_cb(chip::System::Layer *, void *)
{
    s_temp_debounce_armed = false;
    for (auto &c : s_node_cls) {
        if (c.in_use && c.cls == node_class_t::unknown) {
            c.cls = node_class_t::sensor;
            flush_node_temps_to_climate(c.node);
        }
    }
}

static void temp_arm_classify_debounce(void)
{
    if (s_temp_debounce_armed) return;
    if (chip::DeviceLayer::SystemLayer().StartTimer(
            chip::System::Clock::Milliseconds32(TEMP_CLASSIFY_DEBOUNCE_MS),
            temp_classify_timer_cb, nullptr) == CHIP_NO_ERROR) {
        s_temp_debounce_armed = true;
    }
}

// A weather marker arrived → this node is the weather aggregator. Flush any temps buffered while
// its class was unknown. A weather marker is unambiguous, so this also upgrades a node that was
// provisionally classed as a plain sensor.
static void mark_weather_node(chip::NodeId node)
{
    node_cls_t *c = node_cls_get(node);
    if (!c || c->cls == node_class_t::weather) return;
    c->cls = node_class_t::weather;
    for (size_t i = 0; i < s_weather_ep_count; i++) {
        if (s_weather_eps[i].node == node && s_weather_eps[i].have_temp) weather_ep_flush(&s_weather_eps[i]);
    }
}

// A non-weather sensor cluster (humidity/AQ/CO2/PM2.5) arrived → this node is a plain sensor;
// flush any buffered temperature to Indoor. Only promotes from 'unknown'.
static void mark_sensor_node(chip::NodeId node)
{
    node_cls_t *c = node_cls_get(node);
    if (!c || c->cls != node_class_t::unknown) return;
    c->cls = node_class_t::sensor;
    flush_node_temps_to_climate(node);
}

/* Contact sensors and water-leak detectors both report through BooleanState, but with opposite
 * alarming polarity (contact: false = open; leak: true = water present), so each endpoint's kind
 * must be known. Only the leak detector also exposes BooleanStateConfiguration, so that's the
 * discriminator, with a short debounce to let it arrive. Entries are keyed by (node, endpoint)
 * because different nodes may reuse the same endpoint number. */
enum class bool_kind_t : uint8_t { unknown, contact, leak };
#define MAX_BOOL_SENSORS 6
#define BOOL_CLASSIFY_DEBOUNCE_MS 2500
struct bool_sensor_t {
    chip::NodeId node_id;
    chip::EndpointId ep;
    bool_kind_t kind;
    bool have_state;
    bool state;          // raw BooleanState.StateValue
    bool alarming;       // last computed alarming condition
    bool have_alarming;
    bool in_use;
};
static bool_sensor_t s_bool_sensors[MAX_BOOL_SENSORS];
static bool s_bool_debounce_armed = false;

static bool_sensor_t *bool_sensor_get(chip::NodeId node, chip::EndpointId ep)
{
    for (auto &b : s_bool_sensors) {
        if (b.in_use && b.node_id == node && b.ep == ep) {
            return &b;
        }
    }
    for (auto &b : s_bool_sensors) {
        if (!b.in_use) {
            b = { node, ep, bool_kind_t::unknown, false, false, false, false, true };
            return &b;
        }
    }
    return nullptr;
}

// Compute the alarming condition for a classified boolean sensor and, on a transition INTO the
// alarming state, flash the alert overlay. Idempotent: repeated same-state reports do nothing.
static void bool_sensor_evaluate(bool_sensor_t *b)
{
    if (!b || b->kind == bool_kind_t::unknown || !b->have_state) {
        return;  // need both the device kind and a state before we can decide
    }
    bool alarming = (b->kind == bool_kind_t::leak) ? (b->state == true) : (b->state == false);
    bool was_alarming = b->have_alarming && b->alarming;
    b->alarming = alarming;
    b->have_alarming = true;
    if (alarming && !was_alarming) {
        if (b->kind == bool_kind_t::leak) {
            ESP_LOGW(TAG, "ALERT: water leak detected (node 0x" ChipLogFormatX64 " ep %u)",
                     ChipLogValueX64(b->node_id), b->ep);
            light_ui_show_alert("Water Leak", "Leak detected", /* critical */ true);
            audio_play_alert();   /* single-note chime for the critical (water-leak) alert */
        } else {
            ESP_LOGW(TAG, "ALERT: contact opened (node 0x" ChipLogFormatX64 " ep %u)",
                     ChipLogValueX64(b->node_id), b->ep);
            light_ui_show_alert("Contact Open", "A door or window is open", /* critical */ false);
            audio_play_alert();   /* single-note chime for the contact-open alert */
        }
    }
}

// Debounce timer: anything still unclassified once the initial report burst has settled has no
// BooleanStateConfiguration, so it is a contact sensor. Classify and evaluate it now. Runs on the
// Matter task (SystemLayer timer).
static void bool_classify_timer_cb(chip::System::Layer *, void *)
{
    s_bool_debounce_armed = false;
    for (auto &b : s_bool_sensors) {
        if (b.in_use && b.kind == bool_kind_t::unknown) {
            b.kind = bool_kind_t::contact;
            bool_sensor_evaluate(&b);
        }
    }
}

static void bool_arm_classify_debounce(void)
{
    if (s_bool_debounce_armed) {
        return;
    }
    if (chip::DeviceLayer::SystemLayer().StartTimer(
            chip::System::Clock::Milliseconds32(BOOL_CLASSIFY_DEBOUNCE_MS),
            bool_classify_timer_cb, nullptr) == CHIP_NO_ERROR) {
        s_bool_debounce_armed = true;
    }
}

/* ── Controllable devices (lights / smart plugs) ────────────────────────────────────────────────
 * Which bound nodes actually accept OnOff/Level/Color commands, learned from the reports they send:
 * a node that reports OnOff is controllable, and its Descriptor device type says whether it is a
 * light or a plug.
 *
 * This exists because esp_matter's client::cluster_update() fan-out cannot be trusted to target
 * lights. It defers to CHIP's BindingManager, which matches a binding when
 * `clusterId.value_or(cluster) == cluster` — so a WILDCARD binding (all a controller like Home
 * Assistant creates) matches every cluster, and a light command is invoked on every bound node:
 * the weather aggregator, the room sensor, and the sleepy contact / leak detectors. Those reject it
 * or never answer, which both logs "Failed to send command request" and churns CASE sessions and
 * exchanges enough to make the real light's command unreliable. Commands are therefore filtered
 * against this table before they go out. */
#define MAX_ONOFF_DEVICES 8
static struct onoff_dev_t {
    chip::FabricIndex fabric_index;
    chip::NodeId node_id;
    chip::EndpointId ep;
    bool is_plug;
    bool in_use;
} s_onoff_devs[MAX_ONOFF_DEVICES];

/* Remember a node/endpoint that reported OnOff, i.e. something the user can control. */
static void onoff_dev_note(chip::FabricIndex fabric, chip::NodeId node, chip::EndpointId ep)
{
    for (auto &d : s_onoff_devs) {
        if (d.in_use && d.node_id == node && d.ep == ep) { d.fabric_index = fabric; return; }
    }
    for (auto &d : s_onoff_devs) {
        if (!d.in_use) {
            d.fabric_index = fabric; d.node_id = node; d.ep = ep; d.is_plug = false; d.in_use = true;
            return;
        }
    }
}

static void onoff_dev_set_plug(chip::NodeId node, chip::EndpointId ep, bool is_plug)
{
    for (auto &d : s_onoff_devs) {
        if (d.in_use && d.node_id == node && d.ep == ep) { d.is_plug = is_plug; return; }
    }
}

/* Whether `node` may be sent `cluster`. Unknown nodes are never sent anything (they are sensors
 * swept in by a wildcard binding); plugs get OnOff only, since they have no Level/Color clusters. */
static bool onoff_dev_accepts(chip::NodeId node, chip::ClusterId cluster)
{
    for (auto &d : s_onoff_devs) {
        if (!d.in_use || d.node_id != node) continue;
        if (cluster == OnOff::Id) return true;
        return !d.is_plug;
    }
    return false;
}

/* Handles subscription reports from bound sensor nodes (temperature/humidity sensor, air-quality
 * monitor, weather aggregator, and/or contact / water-leak sensors) and pushes the decoded readings
 * onto the display. One instance backs each node's subscription (from a small pool), tagged with
 * weather_mode: on a weather node, temperatures are forecast readings routed by their FixedLabel
 * role and MUST NOT reach the climate page; on a plain sensor node, the temperature is the room
 * reading and goes to the climate page. auto_resubscribe keeps each subscription alive, so OnError
 * just logs; OnResubscriptionNeeded vetoes endless retries for a wildcard node that never returns
 * data (the bound light — see below). */
class SensorReadClientCallback : public chip::app::ReadClient::Callback {
public:
    bool weather_mode = false;
    bool received_data = false;  // set once this node has ever delivered an attribute report
    uint8_t resub_attempts = 0;  // consecutive resubscribe attempts with no data yet
    chip::NodeId node_id = chip::kUndefinedNodeId;  // the node this subscription belongs to
    chip::FabricIndex fabric_index = chip::kUndefinedFabricIndex;  // its fabric (for per-device toggles)

    void OnAttributeData(const chip::app::ConcreteDataAttributePath &aPath,
                         chip::TLV::TLVReader *aReader,
                         const chip::app::StatusIB &aStatus) override
    {
        if (!aReader) {
            return;
        }
        received_data = true;  // a real sensor: keep its subscription alive across drops
        if (aPath.mClusterId == TemperatureMeasurement::Id &&
            aPath.mAttributeId == TemperatureMeasurement::Attributes::MeasuredValue::Id) {
            // MeasuredValue is a nullable int16 in centi-degrees Celsius (0.01 °C).
            chip::app::DataModel::Nullable<int16_t> temp;
            if (chip::app::DataModel::Decode(*aReader, temp) == CHIP_NO_ERROR && !temp.IsNull()) {
                float celsius = temp.Value() / 100.0f;
                // Always buffer the latest reading per (node, endpoint); routing depends on the node's
                // class (weather aggregator vs plain sensor), which may still be resolving.
                weather_ep_t *e = weather_ep_get(node_id, aPath.mEndpointId);
                if (e) { e->have_temp = true; e->celsius = celsius; }
                node_cls_t *nc = node_cls_get(node_id);
                node_class_t cls = nc ? nc->cls : node_class_t::unknown;
                if (cls == node_class_t::weather) {
                    // Forecast temp → weather page, routed by the endpoint's role (may arrive later).
                    ESP_LOGI(TAG, "Weather temp: %.1f C (endpoint %u)", celsius, aPath.mEndpointId);
                    if (e) weather_ep_flush(e);
                } else if (cls == node_class_t::sensor) {
                    // Room temperature sensor → Indoor page (only the first temp source is shown).
                    ESP_LOGI(TAG, "Temperature report: %.2f C (endpoint %u)", celsius, aPath.mEndpointId);
                    if (indoor_accept(IM_TEMP, node_id, aPath.mEndpointId)) light_ui_set_temperature(celsius);
                } else {
                    // Unknown yet: buffered above — a weather marker, a sensor cluster, or the debounce
                    // will classify the node and then flush this reading to the right page.
                    temp_arm_classify_debounce();
                }
            }
        } else if (aPath.mClusterId == FixedLabel::Id &&
                   aPath.mAttributeId == FixedLabel::Attributes::LabelList::Id) {
            // Per-endpoint labels: "role"=min/max/current tags a weather temp endpoint;
            // "location" (on the aggregator) is the place name; "type"=weather is the marker.
            FixedLabel::Attributes::LabelList::TypeInfo::DecodableType list;
            if (chip::app::DataModel::Decode(*aReader, list) == CHIP_NO_ERROR) {
                auto it = list.begin();
                while (it.Next()) {
                    char label[16], value[24];
                    span_to_cstr(it.GetValue().label, label, sizeof(label));
                    span_to_cstr(it.GetValue().value, value, sizeof(value));
                    if (strcmp(label, "role") == 0) {
                        weather_role_t role = weather_role_t::none;
                        if (strcmp(value, "min") == 0) role = weather_role_t::min;
                        else if (strcmp(value, "max") == 0) role = weather_role_t::max;
                        else if (strcmp(value, "current") == 0) role = weather_role_t::current;
                        ESP_LOGI(TAG, "Weather role: endpoint %u = %s", aPath.mEndpointId, value);
                        mark_weather_node(node_id);   // a role marks this node as the weather aggregator
                        weather_ep_t *e = weather_ep_get(node_id, aPath.mEndpointId);
                        if (e) {
                            e->role = role;
                            weather_ep_flush(e);  // apply any temp buffered before the role arrived
                        }
                    } else if (strcmp(label, "location") == 0) {
                        ESP_LOGI(TAG, "Weather location: %s", value);
                        mark_weather_node(node_id);
                        light_ui_set_weather_location(value);
                    } else if (strcmp(label, "type") == 0 && strcmp(value, "weather") == 0) {
                        mark_weather_node(node_id);
                    }
                }
            }
        } else if (aPath.mClusterId == UserLabel::Id &&
                   aPath.mAttributeId == UserLabel::Attributes::LabelList::Id) {
            // The aggregator's UserLabel carries the short text forecast under "condition".
            UserLabel::Attributes::LabelList::TypeInfo::DecodableType list;
            if (chip::app::DataModel::Decode(*aReader, list) == CHIP_NO_ERROR) {
                auto it = list.begin();
                while (it.Next()) {
                    char label[16], value[40];
                    span_to_cstr(it.GetValue().label, label, sizeof(label));
                    span_to_cstr(it.GetValue().value, value, sizeof(value));
                    if (strcmp(label, "condition") == 0) {
                        ESP_LOGI(TAG, "Weather condition: %s", value);
                        mark_weather_node(node_id);   // a forecast condition marks the weather aggregator
                        light_ui_set_weather_condition(value);
                    }
                }
            }
        } else if (aPath.mClusterId == RelativeHumidityMeasurement::Id &&
                   aPath.mAttributeId == RelativeHumidityMeasurement::Attributes::MeasuredValue::Id) {
            // MeasuredValue is a nullable uint16 in centi-percent (0.01 %RH).
            chip::app::DataModel::Nullable<uint16_t> humidity;
            if (chip::app::DataModel::Decode(*aReader, humidity) == CHIP_NO_ERROR && !humidity.IsNull()) {
                float percent = humidity.Value() / 100.0f;
                ESP_LOGI(TAG, "Humidity report: %.2f %% (endpoint %u)", percent, aPath.mEndpointId);
                mark_sensor_node(node_id);   // a humidity cluster ⇒ a real sensor (not the aggregator)
                if (indoor_accept(IM_HUMIDITY, node_id, aPath.mEndpointId)) light_ui_set_humidity(percent);
            }
        } else if (aPath.mClusterId == AirQuality::Id &&
                   aPath.mAttributeId == AirQuality::Attributes::AirQuality::Id) {
            // Overall air-quality rating enum (0 Unknown .. 6 Extremely Poor).
            AirQuality::AirQualityEnum aq;
            if (chip::app::DataModel::Decode(*aReader, aq) == CHIP_NO_ERROR) {
                int level = static_cast<uint8_t>(aq);
                ESP_LOGI(TAG, "Air Quality report: level %d (endpoint %u)", level, aPath.mEndpointId);
                mark_sensor_node(node_id);
                if (indoor_accept(IM_AQ, node_id, aPath.mEndpointId)) light_ui_set_air_quality(level);
            }
        } else if (aPath.mClusterId == CarbonDioxideConcentrationMeasurement::Id &&
                   aPath.mAttributeId == CarbonDioxideConcentrationMeasurement::Attributes::MeasuredValue::Id) {
            // MeasuredValue is a nullable float in ppm.
            chip::app::DataModel::Nullable<float> co2;
            if (chip::app::DataModel::Decode(*aReader, co2) == CHIP_NO_ERROR && !co2.IsNull()) {
                ESP_LOGI(TAG, "CO2 report: %.0f ppm (endpoint %u)", co2.Value(), aPath.mEndpointId);
                mark_sensor_node(node_id);
                if (indoor_accept(IM_CO2, node_id, aPath.mEndpointId)) light_ui_set_co2(co2.Value());
            }
        } else if (aPath.mClusterId == Pm25ConcentrationMeasurement::Id &&
                   aPath.mAttributeId == Pm25ConcentrationMeasurement::Attributes::MeasuredValue::Id) {
            // MeasuredValue is a nullable float in µg/m³.
            chip::app::DataModel::Nullable<float> pm25;
            if (chip::app::DataModel::Decode(*aReader, pm25) == CHIP_NO_ERROR && !pm25.IsNull()) {
                ESP_LOGI(TAG, "PM2.5 report: %.1f ug/m3 (endpoint %u)", pm25.Value(), aPath.mEndpointId);
                mark_sensor_node(node_id);
                if (indoor_accept(IM_PM25, node_id, aPath.mEndpointId)) light_ui_set_pm25(pm25.Value());
            }
        } else if (aPath.mClusterId == BooleanState::Id &&
                   aPath.mAttributeId == BooleanState::Attributes::StateValue::Id) {
            // Contact sensor (StateValue false = open) or water-leak detector (true = water). We do
            // not yet know which — classified by whether this endpoint also reports
            // BooleanStateConfiguration (leak only), resolved via the debounce below.
            bool state;
            if (chip::app::DataModel::Decode(*aReader, state) == CHIP_NO_ERROR) {
                ESP_LOGI(TAG, "BooleanState node 0x" ChipLogFormatX64 " ep %u: %s",
                         ChipLogValueX64(node_id), aPath.mEndpointId, state ? "true" : "false");
                bool_sensor_t *b = bool_sensor_get(node_id, aPath.mEndpointId);
                if (b) {
                    b->have_state = true;
                    b->state = state;
                    if (b->kind == bool_kind_t::unknown) {
                        bool_arm_classify_debounce();  // wait for a possible BooleanStateConfiguration
                    } else {
                        bool_sensor_evaluate(b);
                    }
                }
            }
        } else if (aPath.mClusterId == BooleanStateConfiguration::Id) {
            // Only a water-leak detector exposes this cluster, so ANY report from it classifies the
            // endpoint as a leak detector; evaluate immediately in case a state is already buffered.
            bool_sensor_t *b = bool_sensor_get(node_id, aPath.mEndpointId);
            if (b) {
                if (b->kind != bool_kind_t::leak) {
                    ESP_LOGI(TAG, "Node 0x" ChipLogFormatX64 " ep %u is a water-leak detector",
                             ChipLogValueX64(node_id), aPath.mEndpointId);
                }
                b->kind = bool_kind_t::leak;
                bool_sensor_evaluate(b);
            }
            if (aPath.mAttributeId == BooleanStateConfiguration::Attributes::SensorFault::Id) {
                uint16_t fault;
                if (chip::app::DataModel::Decode(*aReader, fault) == CHIP_NO_ERROR && fault != 0) {
                    ESP_LOGW(TAG, "BooleanStateConfig ep %u SensorFault=0x%04x", aPath.mEndpointId, fault);
                }
            }
        } else if (aPath.mClusterId == OnOff::Id &&
                   aPath.mAttributeId == OnOff::Attributes::OnOff::Id) {
            // A bound light or smart plug reporting its on/off state → drives the per-device page.
            bool on;
            if (chip::app::DataModel::Decode(*aReader, on) == CHIP_NO_ERROR) {
                ESP_LOGI(TAG, "OnOff device node 0x" ChipLogFormatX64 " ep %u: %s",
                         ChipLogValueX64(node_id), aPath.mEndpointId, on ? "ON" : "OFF");
                onoff_dev_note(fabric_index, node_id, aPath.mEndpointId);
                light_ui_set_device_state(fabric_index, node_id, aPath.mEndpointId, on);
            }
        } else if (aPath.mClusterId == Descriptor::Id &&
                   aPath.mAttributeId == Descriptor::Attributes::DeviceTypeList::Id) {
            // Endpoint device type → pick the light vs smart-plug icon on the per-device page.
            Descriptor::Attributes::DeviceTypeList::TypeInfo::DecodableType list;
            if (chip::app::DataModel::Decode(*aReader, list) == CHIP_NO_ERROR) {
                bool is_light = false, is_plug = false;
                auto it = list.begin();
                while (it.Next()) {
                    uint32_t dt = it.GetValue().deviceType;
                    if (dt == 0x010A || dt == 0x010B || dt == 0x010F || dt == 0x0110) is_plug = true;   // plug-in / mounted
                    else if (dt == 0x0100 || dt == 0x0101 || dt == 0x010C || dt == 0x010D) is_light = true;  // on-off/dimmable/colour light
                }
                if (is_plug || is_light) {
                    onoff_dev_set_plug(node_id, aPath.mEndpointId, is_plug);
                    light_ui_set_device_kind(node_id, aPath.mEndpointId, is_plug ? 1 : 0);
                }
            }
        }
    }

    void OnEventData(const chip::app::EventHeader &aEventHeader, chip::TLV::TLVReader *apData,
                     const chip::app::StatusIB *aStatus) override {}

    void OnError(CHIP_ERROR aError) override
    {
        ESP_LOGE(TAG, "Sensor subscription error: %s", chip::ErrorStr(aError));
    }

    void OnDone(chip::app::ReadClient *apReadClient) override
    {
        ESP_LOGI(TAG, "Sensor subscription done");
    }

    // Called before each auto-resubscribe attempt. This always resubscribes.
    //
    // There used to be a veto here: a wildcard-bound light was assumed to have none of the clusters
    // we ask for, so after a few rejections its subscription was torn down to stop it retrying
    // forever. That reasoning stopped being true once OnOff joined the wildcard path list — the
    // light is now a wanted target of this subscription, because its OnOff reports are what
    // populate the Lights page and the controllable-device registry. The veto was killing them,
    // taking the master toggle and (when it caught the aggregator) the weather with it.
    //
    // CHIP's default policy already backs off between attempts, so a node that never matches costs
    // little. Losing a real device for the rest of the session costs a great deal.
    CHIP_ERROR OnResubscriptionNeeded(chip::app::ReadClient *apReadClient, CHIP_ERROR aTerminationCause) override
    {
        resub_attempts++;
        ESP_LOGW(TAG, "Sensor node 0x" ChipLogFormatX64 " subscription dropped after %s (attempt %u, "
                      "cause %" CHIP_ERROR_FORMAT ") — resubscribing",
                 ChipLogValueX64(node_id), received_data ? "delivering data" : "NO data",
                 resub_attempts, aTerminationCause.Format());
        return apReadClient->DefaultResubscribePolicy(aTerminationCause);
    }
};
// One callback per subscribed node: subscriptions are long-lived and auto-resubscribe, so each
// needs its own instance (which also carries that node's weather_mode).
/* Publisher-side lifetime of our subscriptions: also how long a ghost subscription survives our
 * reboot before the publisher frees its paths. */
#define SENSOR_SUBSCRIBE_MAX_INTERVAL_S 300

#define MAX_SENSOR_CALLBACKS 8
static SensorReadClientCallback s_sensor_cbs[MAX_SENSOR_CALLBACKS];
static size_t s_sensor_cb_count = 0;

// Build and send the attribute subscription for a freshly-connected sensor node. Called from our
// own per-node OnDeviceConnected callback (below), so it always runs on the Matter task.
static void do_sensor_subscribe(client::peer_device_t *peer_device)
{
    // Identify the peer so we only include paths for clusters on this device — mixing in paths
    // from other nodes can make a strict sensor reject the whole request.
    chip::NodeId targetNodeId = peer_device->GetDeviceId();
    chip::FabricIndex targetFabricIdx = chip::kUndefinedFabricIndex;
    {
        auto session = peer_device->GetSecureSession();
        if (session.HasValue() && session.Value()->IsSecureSession()) {
            targetFabricIdx = session.Value()->AsSecureSession()->GetFabricIndex();
        }
    }

    // A wildcard ("all clusters") binding — or an explicit UserLabel/FixedLabel one — marks a node
    // we treat as the weather aggregator: it spans several endpoints (aggregator + min/max/current
    // temp sensors), so we subscribe with wildcard-endpoint paths and route temps by their
    // FixedLabel "role".
    bool use_wildcard = false;
    for (const auto &binding : chip::app::Clusters::Binding::Table::GetInstance()) {
        if (binding.type == chip::app::Clusters::Binding::MATTER_UNICAST_BINDING &&
            binding.nodeId == targetNodeId &&
            (!binding.clusterId.has_value() ||
             binding.clusterId.value() == UserLabel::Id || binding.clusterId.value() == FixedLabel::Id)) {
            use_wildcard = true;
            break;
        }
    }

    // Build the attribute paths into a static array — this must complete and call
    // send_request before returning, because the CASESessionManager frees the
    // OperationalSessionSetup (peer_device) once the connection callbacks return.
    static chip::app::AttributePathParams paths[16];
    size_t path_count = 0;
    if (use_wildcard) {
        // Wildcard endpoint on each cluster we understand: weather temps, the FixedLabels carrying
        // role/type/location, the UserLabel carrying the forecast, and the BooleanState /
        // BooleanStateConfiguration of a contact sensor or leak detector. Deliberately not
        // Descriptor or "all clusters": a bound light has none of these, so its subscription is
        // rejected and self-terminates (see OnResubscriptionNeeded).
        const chip::ClusterId wildcard_clusters[] = {
            // Indoor sensors: any wildcard-bound device now delivers every reading it has (the Indoor
            // page picks the first source per cluster; see indoor_accept).
            TemperatureMeasurement::Id, RelativeHumidityMeasurement::Id, AirQuality::Id,
            CarbonDioxideConcentrationMeasurement::Id, Pm25ConcentrationMeasurement::Id,
            FixedLabel::Id, UserLabel::Id,               // weather aggregator: role/location/condition
            BooleanState::Id, BooleanStateConfiguration::Id,   // contact / water-leak sensors
            OnOff::Id,   // bound lights / smart plugs report on/off here (drives the per-device page)
        };
        for (chip::ClusterId cid : wildcard_clusters) {
            paths[path_count].mEndpointId  = chip::kInvalidEndpointId;  // wildcard: all endpoints
            paths[path_count].mClusterId   = cid;
            paths[path_count].mAttributeId = kInvalidAttributeId;       // wildcard: all attributes
            path_count++;
        }
        // Descriptor DeviceTypeList (specific attr, not the whole cluster) → light vs plug icon.
        paths[path_count].mEndpointId  = chip::kInvalidEndpointId;
        paths[path_count].mClusterId   = Descriptor::Id;
        paths[path_count].mAttributeId = Descriptor::Attributes::DeviceTypeList::Id;
        path_count++;
    } else {
        for (const auto &binding : chip::app::Clusters::Binding::Table::GetInstance()) {
            if (binding.type != chip::app::Clusters::Binding::MATTER_UNICAST_BINDING ||
                path_count >= 8) {
                continue;
            }
            if (binding.nodeId != targetNodeId) {
                continue;
            }
            if (targetFabricIdx != chip::kUndefinedFabricIndex && binding.fabricIndex != targetFabricIdx) {
                continue;
            }
            // Only subscribe to explicit sensor cluster bindings (skip the light's "all clusters"
            // wildcard binding so we don't over-subscribe).
            if (!binding.clusterId.has_value()) {
                continue;
            }
            chip::ClusterId cid = binding.clusterId.value();
            if (cid == TemperatureMeasurement::Id || cid == RelativeHumidityMeasurement::Id ||
                cid == AirQuality::Id || cid == CarbonDioxideConcentrationMeasurement::Id ||
                cid == Pm25ConcentrationMeasurement::Id ||
                cid == BooleanState::Id || cid == BooleanStateConfiguration::Id) {
                paths[path_count].mEndpointId  = binding.remote;
                paths[path_count].mClusterId   = cid;
                paths[path_count].mAttributeId = kInvalidAttributeId;
                path_count++;
            }
        }
    }
    if (path_count == 0) {
        ESP_LOGE(TAG, "No sensor paths found in binding table for node 0x" ChipLogFormatX64,
                 ChipLogValueX64(targetNodeId));
        return;
    }

    for (size_t i = 0; i < path_count; i++) {
        ESP_LOGI(TAG, "  subscribe path[%d]: ep=%u cluster=0x%04" PRIx32 " attr=0x%04" PRIx32,
                 (int)i, paths[i].mEndpointId, paths[i].mClusterId, paths[i].mAttributeId);
    }

    // Allocate a persistent callback for this node's subscription, tagged with the routing mode
    // (weather node → forecast temps routed by role; plain sensor → room temperature).
    if (s_sensor_cb_count >= MAX_SENSOR_CALLBACKS) {
        ESP_LOGE(TAG, "Out of sensor callbacks; not subscribing node 0x" ChipLogFormatX64,
                 ChipLogValueX64(targetNodeId));
        return;
    }
    SensorReadClientCallback &cb = s_sensor_cbs[s_sensor_cb_count++];
    cb.weather_mode = use_wildcard;
    cb.node_id = targetNodeId;
    cb.fabric_index = targetFabricIdx;

    // min=0 lets the device choose its own floor. The max interval is also how long a publisher
    // keeps this subscription alive after we vanish: we reboot far more often than a sensor does,
    // and until the old subscription times out the publisher is still holding its attribute paths
    // for us. At 3600 s that ghost could survive an hour, and the next boot's SubscribeRequest —
    // ten wildcard paths — came back PATHS_EXHAUSTED (0x5c8) with the weather and the lights
    // silently missing. SENSOR_SUBSCRIBE_MAX_INTERVAL_S bounds that recovery instead. Raise it if a
    // very sleepy publisher refuses to report this often.
    ESP_LOGI(TAG, "Sending sensor SubscribeRequest to node 0x" ChipLogFormatX64 ": %d path(s)%s",
             ChipLogValueX64(targetNodeId), (int)path_count, use_wildcard ? " [weather]" : "");
    esp_err_t err = client::interaction::subscribe::send_request(peer_device, paths, path_count,
                                                                 nullptr, 0,
                                                                 /* min_interval_s */ 0,
                                                                 SENSOR_SUBSCRIBE_MAX_INTERVAL_S,
                                                                 /* keep_subscription */ true,
                                                                 /* auto_resubscribe */ true,
                                                                 cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor subscribe send_request failed: 0x%x", err);
        s_sensor_cb_count--;  // reclaim the callback slot
    }
}

/* We establish the CASE session ourselves rather than using esp_matter's client::connect(), which
 * shares two static Callback objects — overlapping connects corrupt CHIP's intrusive callback list.
 * Here each node gets its own success/failure callback pair, so all nodes connect in parallel and
 * failures are reported back to app_main for retry. */
// Defined in app_main.cpp: drop the node from the handled set so a later trigger retries it.
void app_sensor_node_connect_failed(chip::NodeId node_id);

// CASE connects fail transiently when more nodes handshake at once than
// CHIP_CONFIG_DEVICE_MAX_ACTIVE_CASE_CLIENTS allows (CHIP_ERROR_NO_MEMORY), and a sleepy ICD can
// take ~120 s to wake and answer at all — so failed connects retry on a timer for ~150 s.
#define SENSOR_CONNECT_MAX_ATTEMPTS 15
#define SENSOR_CONNECT_RETRY_DELAY_S 10

static esp_err_t sensor_connect_node_attempt(chip::FabricIndex fabric_index, chip::NodeId node_id,
                                             uint8_t attempt);

static void sensor_on_connected(void *context, chip::Messaging::ExchangeManager &exchangeMgr,
                                const chip::SessionHandle &sessionHandle);
static void sensor_on_connect_failure(void *context, const chip::ScopedNodeId &peerId, CHIP_ERROR error);

// chip::Callback::Callback has no default constructor, so the handlers are bound in the member
// initializers and each instance's context is filled in at alloc time.
struct sensor_connect_ctx_t {
    chip::Callback::Callback<chip::OnDeviceConnected> on_connected{sensor_on_connected, nullptr};
    chip::Callback::Callback<chip::OnDeviceConnectionFailure> on_failure{sensor_on_connect_failure, nullptr};
    chip::NodeId node_id = chip::kUndefinedNodeId;
    chip::FabricIndex fabric_index = chip::kUndefinedFabricIndex;
    uint8_t attempt = 0;
    bool in_use = false;
};

static sensor_connect_ctx_t s_connect_ctxs[MAX_SENSOR_CALLBACKS];

// Pending delayed retries: the SystemLayer timer's state is a raw void*, so it points at one of
// these static slots rather than heap-allocated state.
struct sensor_retry_t {
    chip::NodeId node_id = chip::kUndefinedNodeId;
    chip::FabricIndex fabric_index = chip::kUndefinedFabricIndex;
    uint8_t attempt = 0;
    bool pending = false;
};
static sensor_retry_t s_retries[MAX_SENSOR_CALLBACKS];

static sensor_connect_ctx_t *sensor_connect_ctx_alloc(chip::NodeId node_id, chip::FabricIndex fabric_index,
                                                      uint8_t attempt)
{
    for (auto &ctx : s_connect_ctxs) {
        if (!ctx.in_use) {
            ctx.in_use = true;
            ctx.node_id = node_id;
            ctx.fabric_index = fabric_index;
            ctx.attempt = attempt;
            ctx.on_connected.mContext = &ctx;
            ctx.on_failure.mContext = &ctx;
            return &ctx;
        }
    }
    return nullptr;
}

static void sensor_retry_timer_cb(chip::System::Layer *, void *app_state)
{
    auto *r = static_cast<sensor_retry_t *>(app_state);
    r->pending = false;
    sensor_connect_node_attempt(r->fabric_index, r->node_id, r->attempt);
}

// Schedule a connect attempt for node_id after delay_ms — used for the staggered first attempt and
// for retry-after-failure. Runs on the Matter task (StartTimer is thread-affine to it).
static void sensor_schedule_retry(chip::FabricIndex fabric_index, chip::NodeId node_id, uint8_t attempt,
                                  uint32_t delay_ms)
{
    sensor_retry_t *slot = nullptr;
    for (auto &r : s_retries) {
        if (!r.pending) { slot = &r; break; }
    }
    if (!slot) {
        ESP_LOGE(TAG, "No retry slot for node 0x" ChipLogFormatX64, ChipLogValueX64(node_id));
        app_sensor_node_connect_failed(node_id);  // fall back to external-trigger retry
        return;
    }
    slot->node_id = node_id;
    slot->fabric_index = fabric_index;
    slot->attempt = attempt;
    slot->pending = true;
    CHIP_ERROR err = chip::DeviceLayer::SystemLayer().StartTimer(
        chip::System::Clock::Milliseconds32(delay_ms), sensor_retry_timer_cb, slot);
    if (err != CHIP_NO_ERROR) {
        slot->pending = false;
        app_sensor_node_connect_failed(node_id);
    }
}

static void sensor_on_connected(void *context, chip::Messaging::ExchangeManager &exchangeMgr,
                                const chip::SessionHandle &sessionHandle)
{
    auto *ctx = static_cast<sensor_connect_ctx_t *>(context);
    ESP_LOGI(TAG, "Sensor node 0x" ChipLogFormatX64 " connected; subscribing", ChipLogValueX64(ctx->node_id));
    chip::OperationalDeviceProxy device(&exchangeMgr, sessionHandle);
    do_sensor_subscribe(&device);
    ctx->in_use = false;  // CASE callback consumed; the ReadClient owns the subscription now
}

static void sensor_on_connect_failure(void *context, const chip::ScopedNodeId &peerId, CHIP_ERROR error)
{
    auto *ctx = static_cast<sensor_connect_ctx_t *>(context);
    chip::NodeId node_id = ctx->node_id;
    chip::FabricIndex fabric_index = ctx->fabric_index;
    uint8_t attempt = ctx->attempt;
    ctx->in_use = false;
    if (attempt + 1 < SENSOR_CONNECT_MAX_ATTEMPTS) {
        ESP_LOGW(TAG, "Sensor node 0x" ChipLogFormatX64 " connect failed (%s), attempt %u/%u — retrying in %ds",
                 ChipLogValueX64(node_id), chip::ErrorStr(error), attempt + 1, SENSOR_CONNECT_MAX_ATTEMPTS,
                 SENSOR_CONNECT_RETRY_DELAY_S);
        sensor_schedule_retry(fabric_index, node_id, attempt + 1, SENSOR_CONNECT_RETRY_DELAY_S * 1000);
    } else {
        ESP_LOGE(TAG, "Sensor node 0x" ChipLogFormatX64 " connect failed (%s) after %u attempts — giving up",
                 ChipLogValueX64(node_id), chip::ErrorStr(error), SENSOR_CONNECT_MAX_ATTEMPTS);
        app_sensor_node_connect_failed(node_id);  // un-mark so an external trigger can start over
    }
}

static esp_err_t sensor_connect_node_attempt(chip::FabricIndex fabric_index, chip::NodeId node_id,
                                             uint8_t attempt)
{
    sensor_connect_ctx_t *ctx = sensor_connect_ctx_alloc(node_id, fabric_index, attempt);
    if (!ctx) {
        ESP_LOGE(TAG, "No free connect context for node 0x" ChipLogFormatX64, ChipLogValueX64(node_id));
        // slots are transient; try again shortly
        sensor_schedule_retry(fabric_index, node_id, attempt, SENSOR_CONNECT_RETRY_DELAY_S * 1000);
        return ESP_ERR_NO_MEM;
    }
    auto *case_mgr = chip::Server::GetInstance().GetCASESessionManager();
    if (!case_mgr) {
        ctx->in_use = false;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Connecting to sensor node 0x" ChipLogFormatX64 " (fabric %u, attempt %u)",
             ChipLogValueX64(node_id), fabric_index, attempt + 1);
    case_mgr->FindOrEstablishSession(chip::ScopedNodeId(node_id, fabric_index),
                                     &ctx->on_connected, &ctx->on_failure);
    return ESP_OK;
}

// Begin a CASE session to a sensor node and subscribe on success. Called (once per node) by
// app_main; failed connects retry themselves on a timer. Runs on the Matter task.
esp_err_t app_sensor_connect_node(chip::FabricIndex fabric_index, chip::NodeId node_id)
{
    return sensor_connect_node_attempt(fabric_index, node_id, 0);
}

// Called from app_main on a binding-table change: clear the Indoor page's per-cluster source of
// truth so it's recomputed from subsequent reports. Runs on the Matter task.
void app_sensor_bindings_changed(void)
{
    for (int i = 0; i < IM_COUNT; i++) s_indoor_src[i].have = false;
    ESP_LOGI(TAG, "Binding table changed — Indoor source-of-truth reset");
}

#if CONFIG_ENABLE_CHIP_SHELL
static char console_buffer[101] = {0};
static esp_err_t app_driver_bound_console_handler(int argc, char **argv)
{
    if (argc == 1 && strncmp(argv[0], "help", sizeof("help")) == 0) {
        printf("Bound commands:\n"
               "\thelp: Print help\n"
               "\tinvoke: <local_endpoint_id> <cluster_id> <command_id> parameters ... \n"
               "\t\tExample: matter esp bound invoke 0x0001 0x0003 0x0000 0x78.\n");
    } else if (argc >= 4 && strncmp(argv[0], "invoke", sizeof("invoke")) == 0) {
        client::request_handle_t req_handle;
        req_handle.type = esp_matter::client::INVOKE_CMD;
        uint16_t local_endpoint_id = strtoul((const char *)&argv[1][2], NULL, 16);
        req_handle.command_path.mClusterId = strtoul((const char *)&argv[2][2], NULL, 16);
        req_handle.command_path.mCommandId = strtoul((const char *)&argv[3][2], NULL, 16);

        if (argc > 4) {
            console_buffer[0] = argc - 4;
            for (int i = 0; i < (argc - 4); i++) {
                if ((argv[4 + i][0] != '0') || (argv[4 + i][1] != 'x') ||
                        (strlen((const char *)&argv[4 + i][2]) > 10)) {
                    ESP_LOGE(TAG, "Incorrect arguments. Check help for more details.");
                    return ESP_ERR_INVALID_ARG;
                }
                strcpy((console_buffer + 1 + 10 * i), &argv[4 + i][2]);
            }

            req_handle.request_data = console_buffer;
        }

        client::cluster_update(local_endpoint_id, &req_handle);
    } else {
        ESP_LOGE(TAG, "Incorrect arguments. Check help for more details.");
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t app_driver_client_console_handler(int argc, char **argv)
{
    if (argc == 1 && strncmp(argv[0], "help", sizeof("help")) == 0) {
        printf("Client commands:\n"
               "\thelp: Print help\n"
               "\tinvoke: <fabric_index> <remote_node_id> <remote_endpoint_id> <cluster_id> <command_id> parameters "
               "... \n"
               "\t\tExample: matter esp client invoke 0x0001 0xBC5C01 0x0001 0x0003 0x0000 0x78.\n"
               "\tinvoke-group: <fabric_index> <group_id> <cluster_id> <command_id> parameters ... \n"
               "\t\tExample: matter esp client invoke-group 0x0001 0x257 0x0003 0x0000 0x78.\n");
    } else if (argc >= 6 && strncmp(argv[0], "invoke", sizeof("invoke")) == 0) {
        client::request_handle_t req_handle;
        req_handle.type = esp_matter::client::INVOKE_CMD;
        uint8_t fabric_index = strtoul((const char *)&argv[1][2], NULL, 16);
        uint64_t node_id = strtoull((const char *)&argv[2][2], NULL, 16);
        req_handle.command_path = {(chip::EndpointId)strtoul((const char *)&argv[3][2], NULL, 16) /* EndpointId */,
                                   0 /* GroupId */, strtoul((const char *)&argv[4][2], NULL, 16) /* ClusterId */,
                                   strtoul((const char *)&argv[5][2], NULL, 16) /* CommandId */,
                                   chip::app::CommandPathFlags::kEndpointIdValid
                                  };

        if (argc > 6) {
            console_buffer[0] = argc - 6;
            for (int i = 0; i < (argc - 6); i++) {
                if ((argv[6 + i][0] != '0') || (argv[6 + i][1] != 'x') ||
                        (strlen((const char *)&argv[6 + i][2]) > 10)) {
                    ESP_LOGE(TAG, "Incorrect arguments. Check help for more details.");
                    return ESP_ERR_INVALID_ARG;
                }
                strcpy((console_buffer + 1 + 10 * i), &argv[6 + i][2]);
            }

            req_handle.request_data = console_buffer;
        }
        auto &server = chip::Server::GetInstance();
        client::connect(server.GetCASESessionManager(), fabric_index, node_id, &req_handle);
    } else if (argc >= 5 && strncmp(argv[0], "invoke-group", sizeof("invoke-group")) == 0) {
        client::request_handle_t req_handle;
        req_handle.type = esp_matter::client::INVOKE_CMD;
        uint8_t fabric_index = strtoul((const char *)&argv[1][2], NULL, 16);
        req_handle.command_path.mGroupId = strtoul((const char *)&argv[2][2], NULL, 16);
        req_handle.command_path.mClusterId = strtoul((const char *)&argv[3][2], NULL, 16);
        req_handle.command_path.mCommandId = strtoul((const char *)&argv[4][2], NULL, 16);
        req_handle.command_path = {
            0 /* EndpointId */, (chip::GroupId)strtoul((const char *)&argv[2][2], NULL, 16) /* GroupId */,
            strtoul((const char *)&argv[3][2], NULL, 16) /* ClusterId */,
            strtoul((const char *)&argv[4][2], NULL, 16) /* CommandId */, chip::app::CommandPathFlags::kGroupIdValid
        };

        if (argc > 5) {
            console_buffer[0] = argc - 5;
            for (int i = 0; i < (argc - 5); i++) {
                if ((argv[5 + i][0] != '0') || (argv[5 + i][1] != 'x') ||
                        (strlen((const char *)&argv[5 + i][2]) > 10)) {
                    ESP_LOGE(TAG, "Incorrect arguments. Check help for more details.");
                    return ESP_ERR_INVALID_ARG;
                }
                strcpy((console_buffer + 1 + 10 * i), &argv[5 + i][2]);
            }

            req_handle.request_data = console_buffer;
        }

        client::group_request_send(fabric_index, &req_handle);
    } else {
        ESP_LOGE(TAG, "Incorrect arguments. Check help for more details.");
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static void app_driver_register_commands()
{
    /* Add console command for bound devices */
    static const esp_matter::console::command_t bound_command = {
        .name = "bound",
        .description = "This can be used to simulate on-device control for bound devices."
        "Usage: matter esp bound <bound_command>. "
        "Bound commands: help, invoke",
        .handler = app_driver_bound_console_handler,
    };
    esp_matter::console::add_commands(&bound_command, 1);

    /* Add console command for client to control non-bound devices */
    static const esp_matter::console::command_t client_command = {
        .name = "client",
        .description = "This can be used to simulate on-device control for client devices."
        "Usage: matter esp client <client_command>. "
        "Client commands: help, invoke",
        .handler = app_driver_client_console_handler,
    };
    esp_matter::console::add_commands(&client_command, 1);
}
#endif // CONFIG_ENABLE_CHIP_SHELL

static void send_command_success_callback(void *context, const ConcreteCommandPath &command_path,
                                          const chip::app::StatusIB &status, TLVReader *response_data)
{
    ESP_LOGI(TAG, "Send command success");
}

static void send_command_failure_callback(void *context, CHIP_ERROR error)
{
    ESP_LOGI(TAG, "Send command failure: err :%" CHIP_ERROR_FORMAT, error.Format());
}

void app_driver_client_invoke_command_callback(client::peer_device_t *peer_device, client::request_handle_t *req_handle,
                                               void *priv_data)
{
    if (req_handle->type == esp_matter::client::INVOKE_CMD) {
        char command_data_str[64];
        // A dimmer switch sends on_off, level_control and identify cluster commands.
        if (req_handle->command_path.mClusterId == OnOff::Id) {
            strcpy(command_data_str, "{}");
        } else if (req_handle->command_path.mClusterId == LevelControl::Id) {
            // MoveToLevelWithOnOff: {Level:U8, TransitionTime:U16, OptionsMask:U8, OptionsOverride:U8}
            // request_data points to a single uint8_t target level (1-254).
            if (req_handle->command_path.mCommandId == LevelControl::Commands::MoveToLevelWithOnOff::Id) {
                uint8_t level = ((const uint8_t *)req_handle->request_data)[0];
                snprintf(command_data_str, sizeof(command_data_str),
                         "{\"0:U8\": %u, \"1:U16\": 0, \"2:U8\": 0, \"3:U8\": 0}", level);
            } else {
                ESP_LOGE(TAG, "Unsupported command");
                return;
            }
        } else if (req_handle->command_path.mClusterId == ColorControl::Id) {
            // MoveToColorTemperature: {ColorTemperatureMireds:U16, TransitionTime:U16,
            //                          OptionsMask:U8, OptionsOverride:U8}
            // request_data points to a single uint16_t target color temperature in mireds.
            if (req_handle->command_path.mCommandId == ColorControl::Commands::MoveToColorTemperature::Id) {
                uint16_t mireds = ((const uint16_t *)req_handle->request_data)[0];
                snprintf(command_data_str, sizeof(command_data_str),
                         "{\"0:U16\": %u, \"1:U16\": 0, \"2:U8\": 0, \"3:U8\": 0}", mireds);
            } else if (req_handle->command_path.mCommandId == ColorControl::Commands::MoveToHueAndSaturation::Id) {
                // MoveToHueAndSaturation: {Hue:U8, Saturation:U8, TransitionTime:U16,
                //                          OptionsMask:U8, OptionsOverride:U8}
                // request_data points to two uint8_t: [hue, saturation].
                uint8_t hue = ((const uint8_t *)req_handle->request_data)[0];
                uint8_t sat = ((const uint8_t *)req_handle->request_data)[1];
                snprintf(command_data_str, sizeof(command_data_str),
                         "{\"0:U8\": %u, \"1:U8\": %u, \"2:U16\": 0, \"3:U8\": 0, \"4:U8\": 0}", hue, sat);
            } else {
                ESP_LOGE(TAG, "Unsupported command");
                return;
            }
        } else if (req_handle->command_path.mClusterId == Identify::Id) {
            if (req_handle->command_path.mCommandId == Identify::Commands::Identify::Id) {
                if (((char *)req_handle->request_data)[0] != 1) {
                    ESP_LOGE(TAG, "Number of parameters error");
                    return;
                }
                snprintf(command_data_str, sizeof(command_data_str), "{\"0:U16\": %ld}",
                         strtoul((const char *)(req_handle->request_data) + 1, NULL, 16));
            } else {
                ESP_LOGE(TAG, "Unsupported command");
                return;
            }
        } else {
            ESP_LOGE(TAG, "Unsupported cluster");
            return;
        }
        client::interaction::invoke::send_request(NULL, peer_device, req_handle->command_path, command_data_str,
                                                  send_command_success_callback, send_command_failure_callback,
                                                  chip::NullOptional);
    }
    return;
}

void app_driver_client_callback(client::peer_device_t *peer_device, client::request_handle_t *req_handle,
                                void *priv_data)
{
    if (req_handle->type == esp_matter::client::INVOKE_CMD) {
        // Drop commands aimed at nodes that only look like targets because of a wildcard binding
        // (sensors, the weather aggregator). Sending to them fails and, worse, the extra CASE
        // sessions and exchanges make the real light's command unreliable. See onoff_dev_accepts().
        if (!onoff_dev_accepts(peer_device->GetDeviceId(), req_handle->command_path.mClusterId)) {
            ESP_LOGD(TAG, "Skipping cluster 0x%04" PRIx32 " command to non-controllable node 0x"
                     ChipLogFormatX64, req_handle->command_path.mClusterId,
                     ChipLogValueX64(peer_device->GetDeviceId()));
            return;
        }
        app_driver_client_invoke_command_callback(peer_device, req_handle, priv_data);
    } else if (req_handle->type == esp_matter::client::SUBSCRIBE_ATTR) {
        // Only the bound light's state (on/off, level, color) flows through esp_matter's
        // client::connect() → this callback; sensor nodes use app_sensor_connect_node().
        chip::ClusterId cluster_id = req_handle->attribute_path.mClusterId;
        if (cluster_id == OnOff::Id || cluster_id == LevelControl::Id || cluster_id == ColorControl::Id) {
            app_client_subscribe_command_callback(peer_device, req_handle, priv_data);
        }
    }
    return;
}
void app_driver_client_group_invoke_command_callback(uint8_t fabric_index, client::request_handle_t *req_handle,
                                                     void *priv_data)
{
    if (req_handle->type != esp_matter::client::INVOKE_CMD) {
        return;
    }
    char command_data_str[32];
    // on_off light switch should support on_off cluster and identify cluster commands sending.
    if (req_handle->command_path.mClusterId == OnOff::Id) {
        strcpy(command_data_str, "{}");
    } else if (req_handle->command_path.mClusterId == Identify::Id) {
        if (req_handle->command_path.mCommandId == Identify::Commands::Identify::Id) {
            if (((char *)req_handle->request_data)[0] != 1) {
                ESP_LOGE(TAG, "Number of parameters error");
                return;
            }
            snprintf(command_data_str, sizeof(command_data_str), "{\"0:U16\": %ld}",
                     strtoul((const char *)(req_handle->request_data) + 1, NULL, 16));
        } else {
            ESP_LOGE(TAG, "Unsupported command");
            return;
        }
    } else {
        ESP_LOGE(TAG, "Unsupported cluster");
        return;
    }
    client::interaction::invoke::send_group_request(fabric_index, req_handle->command_path, command_data_str);
}

/* ── Sending a command to the bound lights ──────────────────────────────────────────────────────
 * Each controllable device is addressed directly — the same path the per-device Lights page uses,
 * which is reliable — rather than client::cluster_update()'s binding fan-out (see
 * onoff_dev_accepts() above for why that path cannot be used with wildcard bindings).
 *
 * Sends are serialized: esp_matter's client::connect() parks the request in two function-local
 * static Callback objects, so a second connect begun before the first completes takes over their
 * slot and the first command is silently dropped. When the session is already live the callback
 * runs synchronously and the next send follows immediately; the spacing only matters when a
 * session has to be re-established. */
#define CMD_SPACING_MS 120

static client::request_handle_t s_cmd_req;
static size_t s_cmd_idx      = 0;
static bool   s_cmd_active   = false;
static bool   s_cmd_sent_any = false;

static void cmd_step(chip::System::Layer *, void *)
{
    auto &server = chip::Server::GetInstance();
    while (s_cmd_idx < MAX_ONOFF_DEVICES) {
        onoff_dev_t &d = s_onoff_devs[s_cmd_idx++];
        if (!d.in_use || !onoff_dev_accepts(d.node_id, s_cmd_req.command_path.mClusterId)) {
            continue;
        }
        s_cmd_req.command_path.mEndpointId = d.ep;
        client::connect(server.GetCASESessionManager(), d.fabric_index, d.node_id, &s_cmd_req);
        s_cmd_sent_any = true;
        chip::DeviceLayer::SystemLayer().StartTimer(chip::System::Clock::Milliseconds32(CMD_SPACING_MS),
                                                    cmd_step, nullptr);
        return;
    }
    if (!s_cmd_sent_any) {
        // Nothing has reported OnOff yet, so there is no light to talk to (the binding may not have
        // been made, or its first report has not arrived).
        ESP_LOGW(TAG, "No controllable device known yet — cluster 0x%04" PRIx32 " command not sent",
                 s_cmd_req.command_path.mClusterId);
    }
    s_cmd_active = false;
}

static void cmd_start(intptr_t)
{
    cmd_step(nullptr, nullptr);
}

/* Deliver `req` to every controllable device that accepts its cluster. Callable from any task. */
static void send_to_lights(const client::request_handle_t &req)
{
    bool running   = s_cmd_active;
    s_cmd_req      = req;
    s_cmd_idx      = 0;
    s_cmd_sent_any = false;
    s_cmd_active   = true;
    if (!running) {
        chip::DeviceLayer::PlatformMgr().ScheduleWork(cmd_start, 0);
    }
}

void app_driver_notify_active(void)
{
#if CONFIG_ENABLE_ICD_SERVER
    /* Runs on the CHIP thread (ScheduleWork takes the stack lock for us). Each call keeps the ICD
     * in active mode for about ActiveModeThreshold, so repeated calls while the display is on keep
     * the device reachable and its subscriptions live. */
    chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t) {
        chip::app::ICDNotifier::GetInstance().NotifyNetworkActivityNotification();
    }, 0);
#endif
}

void app_driver_send_toggle()
{
    client::request_handle_t req_handle;
    req_handle.type = esp_matter::client::INVOKE_CMD;
    req_handle.command_path = { 0, 0, OnOff::Id, OnOff::Commands::Toggle::Id,
                                chip::app::CommandPathFlags::kEndpointIdValid };
    send_to_lights(req_handle);
}

void app_driver_send_onoff(bool on)
{
    client::request_handle_t req_handle;
    req_handle.type = esp_matter::client::INVOKE_CMD;
    req_handle.command_path = { 0, 0, OnOff::Id,
                                on ? OnOff::Commands::On::Id : OnOff::Commands::Off::Id,
                                chip::app::CommandPathFlags::kEndpointIdValid };
    send_to_lights(req_handle);
}

void app_driver_send_toggle_node(uint8_t fabric_index, uint64_t node_id, uint16_t endpoint)
{
    // Invoke OnOff::Toggle on ONE specific bound device (by fabric+node+endpoint), instead of the
    // group fan-out cluster_update() does. connect() establishes CASE then routes the command via
    // app_driver_client_callback → invoke_command_callback (which builds "{}" for OnOff).
    client::request_handle_t req_handle;
    req_handle.type = esp_matter::client::INVOKE_CMD;
    req_handle.command_path = { (chip::EndpointId)endpoint, 0 /* GroupId */, OnOff::Id,
                                OnOff::Commands::Toggle::Id, chip::app::CommandPathFlags::kEndpointIdValid };

    lock::ScopedChipStackLock lock(portMAX_DELAY);
    auto &server = chip::Server::GetInstance();
    client::connect(server.GetCASESessionManager(), fabric_index, node_id, &req_handle);
}

/* Target level for a pending MoveToLevelWithOnOff. Static so it stays valid
 * while the request is queued waiting for the CASE session to come up. */
static uint8_t s_level_arg = 0;

void app_driver_send_level(uint8_t level)
{
    s_level_arg = level;
    client::request_handle_t req_handle;
    req_handle.type = esp_matter::client::INVOKE_CMD;
    req_handle.command_path = { 0, 0, LevelControl::Id, LevelControl::Commands::MoveToLevelWithOnOff::Id,
                                chip::app::CommandPathFlags::kEndpointIdValid };
    req_handle.request_data = &s_level_arg;
    send_to_lights(req_handle);
}

/* Target color temperature (mireds) for a pending MoveToColorTemperature. Static
 * so it stays valid while the request is queued waiting for the CASE session. */
static uint16_t s_colortemp_arg = 0;

void app_driver_send_color_temp(uint16_t mireds)
{
    s_colortemp_arg = mireds;
    client::request_handle_t req_handle;
    req_handle.type = esp_matter::client::INVOKE_CMD;
    req_handle.command_path = { 0, 0, ColorControl::Id, ColorControl::Commands::MoveToColorTemperature::Id,
                                chip::app::CommandPathFlags::kEndpointIdValid };
    req_handle.request_data = &s_colortemp_arg;
    send_to_lights(req_handle);
}

/* Target [hue, saturation] for a pending MoveToHueAndSaturation. Static so it stays
 * valid while the request is queued waiting for the CASE session. */
static uint8_t s_huesat_arg[2] = {0, 254};

void app_driver_send_color_hue_sat(uint8_t hue, uint8_t sat)
{
    s_huesat_arg[0] = hue;
    s_huesat_arg[1] = sat;
    client::request_handle_t req_handle;
    req_handle.type = esp_matter::client::INVOKE_CMD;
    req_handle.command_path = { 0, 0, ColorControl::Id, ColorControl::Commands::MoveToHueAndSaturation::Id,
                                chip::app::CommandPathFlags::kEndpointIdValid };
    req_handle.request_data = s_huesat_arg;
    send_to_lights(req_handle);
}

void app_driver_factory_reset(void)
{
    ESP_LOGW(TAG, "Factory reset requested from Settings UI — erasing all data and restarting");
    esp_matter::factory_reset();
}

static void app_driver_button_toggle_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Toggle button pressed");
    app_driver_send_toggle();
}

app_driver_handle_t app_driver_switch_init()
{
    /* Initialize button */

    /* The physical button is non-essential (the touch UI controls everything), so a failure
     * here must NOT abort boot — degrade gracefully and return a NULL handle instead. */
    button_handle_t btns[BSP_BUTTON_NUM] = {};
    esp_err_t btn_err = bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM);
    if (btn_err != ESP_OK || btns[0] == NULL) {
        ESP_LOGW(TAG, "Button init failed (%s) — physical toggle/reset button disabled",
                 esp_err_to_name(btn_err));
        return NULL;
    }
    esp_err_t cb_err = iot_button_register_cb(btns[0], BUTTON_PRESS_DOWN, NULL, app_driver_button_toggle_cb, NULL);
    if (cb_err != ESP_OK) {
        ESP_LOGW(TAG, "Button callback registration failed (%s)", esp_err_to_name(cb_err));
    }

    /* Other initializations */
#if CONFIG_ENABLE_CHIP_SHELL
    app_driver_register_commands();
#endif // CONFIG_ENABLE_CHIP_SHELL
    client::set_request_callback(app_driver_client_callback,
                                 app_driver_client_group_invoke_command_callback, NULL);

    return (app_driver_handle_t)btns[0];
}
