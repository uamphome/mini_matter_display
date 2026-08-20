# Matter Display

Firmware for the **Waveshare ESP32-C6-Touch-AMOLED-1.8**: a Matter-over-Thread
touch display that controls your bound lights and shows readings from your bound
sensors, a clock, and the weather.

To Matter it's a **Colour Dimmer Switch** (endpoint 1) with client clusters for
everything it wants to listen to, so a controller can bind it to lights, sensors
and the weather device from this repo.

## The UI

Swipe horizontally between three pages, and up from Home for Settings.

```
        Controls  ←→  HOME  ←→  Indoor
                       ↕
                    Settings
```

**Home**
- **Clock card** — local time, weekday and date. Tap it to open **Time & Date**.
- **Weather card** — location, condition icon and text, current temperature, and
  L/H chips for the forecast min/max (from the bound weather device).
- **Light card** — tap to toggle every bound light/plug at once (the knob follows
  the aggregate state, so a change made from your phone moves it too). Tap the
  card's icon to open the per-device **Lights** page.

**Controls** — brightness slider for the bound light(s), plus a segmented
Warmth / Colour toggle: a Kelvin slider (2000–6000 K) or a rainbow hue slider.

**Indoor** — a stat grid of the bound sensors: temperature, humidity, CO₂, PM2.5,
and a wide air-quality card whose colour follows the rating. Cards show `--` until
a reading arrives.

**Settings** — display brightness, display-off timeout, and Reboot / Factory reset
(both behind a confirmation overlay).

**Modal pages and overlays**
- **Lights** — one card per individually-controllable bound light or smart plug;
  tap a card to toggle just that device.
- **Time & Date** — rollers to set the time and date by hand, plus an
  **AUTOMATIC** toggle (on by default) that decides whether a controller's time
  push may override what you set.
- **Alert** — a full-screen banner plus a chime when a bound contact sensor opens
  or a water-leak detector trips. It wakes the screen and auto-dismisses.

Before commissioning, the screen shows the onboarding **QR code** instead. After
~2 minutes of inactivity the AMOLED power rail is cut and the screen sleeps; a tap
wakes it (the timeout is configurable in Settings).

## Build and flash

```bash
source /path/to/esp-idf/export.sh
export ESP_MATTER_PATH=/path/to/esp-matter
source $ESP_MATTER_PATH/export.sh

idf.py set-target esp32c6
idf.py build flash monitor
```

The board has 16 MB of flash and uses `partitions_c6.csv` (4 MB OTA slots), set up
in `sdkconfig.defaults.esp32c6` — no manual menuconfig needed.

## Commission and bind

1. **Commission** by scanning the QR code on the display with Apple Home, Google
   Home, Alexa or Home Assistant. You need a **Thread border router** on the
   network. With the default test credentials the codes are:

   | Field | Value |
   | --- | --- |
   | Setup passcode | `20202021` |
   | Discriminator | `3840` |
   | QR payload | `MT:Y.K9042C00KA0648G00` |
   | Manual code | `34970112332` |

2. **Bind** the devices you want it to control and display. In Home Assistant this
   is the *Bind devices* action on the switch; with `chip-tool` write the binding
   table directly (and add the display to each target's ACL). The display reads
   whatever it finds:

   | Bind this | To get |
   | --- | --- |
   | A light or smart plug | Light card, Controls page, Lights page |
   | A temperature / humidity sensor | Indoor temp + humidity |
   | An air-quality monitor | Indoor air quality, CO₂, PM2.5 |
   | The weather device from this repo | Home weather card |
   | A contact sensor or leak detector | Alert overlay + chime |

   Wildcard ("all clusters") bindings are fine — that's all some controllers
   create — and the firmware works out what each bound node actually is from the
   reports it sends.

3. Long-press the **BOOT** button (GPIO9) to factory-reset, or use the Settings
   page.

## Layout

```
main/
  app_main.cpp        Matter node + clusters, event handling, time sync, startup
  app_driver.cpp      Matter client: commands out, sensor subscriptions in
  light_ui.cpp/.h     All LVGL screens + the LVGL task (display sleep, sends)
  audio.c/.h          ES8311 codec + the alert chime
  brutal_*.c          Generated Arial-Black display fonts
  weather_icons.c/.h  Generated weather glyphs
  CHIPProjectConfig.h Raises the concurrent-CASE-session limit
components/
  amoled_driver/      Display + touch + PMIC bring-up and the LVGL port (MIT,
                      vendored — see LICENSE.upstream)
```

## Notes

- **Time** comes from the Matter fabric (`TimeSynchronization`), not the internet:
  this is an IPv6-only Thread build with no route to an NTP server. Controllers
  are inconsistent about re-pushing the time after a reboot, which is why the
  Time & Date page exists.
- **Thread role** is a Minimal End Device (MTD) running as a Matter ICD
  (Intermittently Connected Device), so the 802.15.4 receiver and the CPU are
  duty-cycled instead of being on continuously. While the screen is on it stays
  in ICD active mode and light sleep is blocked, so the UI is unaffected; once
  the screen sleeps it polls its Thread parent every
  `CONFIG_ICD_SLOW_POLL_INTERVAL_MS` (5 s by default). Sending commands is
  unaffected either way — the trade-off is that an incoming report or alert can
  be delayed by up to one slow-poll interval while the screen is off. Lower that
  value in `sdkconfig.defaults.esp32c6` if latency matters more than power.
- The board's `LCD_TE` (tearing-effect) line goes to a test pad, not a GPIO, so
  frame-synced drawing isn't available.
