# Mini Matter Display

A pocket-sized **Matter smart-home display** built on the Waveshare
ESP32-C6-Touch-AMOLED-1.8, plus the **Matter weather device** that feeds it the
forecast.

The display joins your Matter fabric over Thread and binds to devices you already
own: it controls your lights (on/off, brightness, colour temperature, colour),
shows readings from temperature / humidity / air-quality sensors, pops an alert
when a contact sensor opens or a leak detector trips, and shows a clock and the
weather. It talks only to your fabric — no cloud service, no Home Assistant
required, no vendor app.

Because Matter has no "weather" device type, the forecast comes from a companion
device in this repo: a screenless ESP32 that fetches the forecast from
[Open-Meteo](https://open-meteo.com) and publishes it as Matter attributes the
display binds to.

```
      Open-Meteo (HTTPS)
             │
    ┌────────┴─────────┐                       ┌──────────────────────┐
    │  weather device  │   Matter (one fabric) │    Matter display    │
    │  (S3 + H2 board) ├──────────────────────►│  ESP32-C6 + AMOLED   │
    └──────────────────┘                       └──────────┬───────────┘
                                                          │ Matter bindings
                                          lights · sensors · contact/leak
```

## What's in here

| Folder | Board | What it is |
| --- | --- | --- |
| [`matter_display/`](matter_display/) | Waveshare ESP32-C6-Touch-AMOLED-1.8 | The display firmware. Matter over Thread. |
| [`matter_weather_device/`](matter_weather_device/) | ESP Thread Border Router board (ESP32-S3 + ESP32-H2) | The weather device. The S3 fetches the forecast over Wi-Fi; the H2 publishes it over Matter/Thread. |

Each folder has its own README with build, flash and setup instructions.

### Why the weather device needs two chips

A single ESP32-C6 cannot reliably share its one 2.4 GHz radio between Wi-Fi and Thread, and a
Thread-only device has no route to the internet — so the weather device splits the job
across the two chips already wired together on the Thread Border Router board. That
also keeps it on the Thread mesh alongside the display, rather than binding across the
Wi-Fi↔Thread border.

It publishes this structure, which is what the display binds to:

```
Aggregator endpoint
  FixedLabel: type=weather, location=<place>
  UserLabel:  condition=<short forecast text>
  ├─ Temperature Sensor  FixedLabel role=min      MeasuredValue
  ├─ Temperature Sensor  FixedLabel role=max      MeasuredValue
  └─ Temperature Sensor  FixedLabel role=current  MeasuredValue
```

## Flash it from your browser

If you just want to build the thing, you don't need a toolchain at all — all
three firmwares can be flashed from a Chrome/Edge/Opera tab via ESP Launchpad:

**<https://espressif.github.io/esp-launchpad/?flashConfigURL=https://uamphome.github.io/mini_matter_display/config.toml>**

Plug a board in and Launchpad offers the firmware that matches its chip. The
weather device's two chips have a USB-C port each, so flash them one at a time
through their own port. The S3's Wi-Fi and location are then set from Launchpad's
Console tab — see [`launchpad/`](launchpad/) for how this is built and published.

Web Serial is desktop Chrome/Edge/Opera only: no Safari, no Firefox, no mobile.

## Prerequisites

All three firmwares are [ESP-IDF](https://github.com/espressif/esp-idf) projects;
`matter_display` and `h2_matter` also need
[esp-matter](https://github.com/espressif/esp-matter) (`s3_weather` does not).

```bash
# once per shell
source /path/to/esp-idf/export.sh
export ESP_MATTER_PATH=/path/to/esp-matter
source $ESP_MATTER_PATH/export.sh
```

Then, in any project folder:

```bash
idf.py set-target esp32c6     # esp32s3 / esp32h2 for the weather device
idf.py build flash monitor
```

You'll also need a **Thread Border Router** on your network for the display to
join over Thread — an Apple TV / HomePod, a Google Nest Hub, a Home Assistant
Thread border router, or Espressif's own TBR board.

## Hardware

- **Display:** [Waveshare ESP32-C6-Touch-AMOLED-1.8](https://s.click.aliexpress.com/e/_c3kYGeLd)
  — 368×448 SH8601 QSPI AMOLED, FT3168 capacitive touch, AXP2101 PMU, TCA9554 IO
  expander, ES8311 codec + speaker, 16 MB flash.
  ([wiki](https://docs.waveshare.com/ESP32-C6-Touch-AMOLED-1.8), schematic on the
  product page)
- **Weather device:** [ESP Thread Border Router board](https://s.click.aliexpress.com/e/_c33n4Qrz)
  (ESP32-S3 + ESP32-H2 on one PCB).

