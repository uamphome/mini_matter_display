# Matter Weather Device

Matter has no weather device type, so the display in this repo gets its forecast
from a small companion device: it fetches the forecast from
[Open-Meteo](https://open-meteo.com) (free, keyless, global) and publishes it as
Matter attributes the display binds to.

It runs on the [ESP Thread Border Router board](https://github.com/espressif/esp-thread-br)
— an ESP32-S3 and an ESP32-H2 on one PCB, wired together over UART. Two chips, two
radios: the S3 does Wi-Fi and the H2 speaks **Matter over Thread**, so the display
binds to it on the Thread mesh.

```
   Open-Meteo (HTTPS over Wi-Fi)
            │
            ▼
   ┌──────────────┐  UART  ┌──────────────┐  Matter/Thread  ┌──────────────┐
   │  ESP32-S3    ├───────►│  ESP32-H2    ├────────────────►│  display     │
   │  s3_weather  │        │  h2_matter   │                 │  (ESP32-C6)  │
   └──────────────┘        └──────────────┘                 └──────────────┘
     Wi-Fi + fetch           Thread + Matter
     + setup TUI
```

## Why two chips

A single ESP32-C6 cannot share its one 2.4 GHz radio between Wi-Fi and Thread: with
Thread up, a Wi-Fi scan sees no APs, and time-division (pause Thread, fetch, resume)
drops mid-TLS. A Thread-only device, meanwhile, has no route to the internet. Two
radios on one board solves both — and the TBR board already has the topology plus the
inter-chip UART.

Putting the weather device on Thread alongside the display also avoids binding across
the Wi-Fi↔Thread border, which is the less reliable path.

## What it presents to Matter

```
Aggregator endpoint
  FixedLabel: type=weather, location=<place>
  UserLabel:  condition=<short forecast text>
  ├─ Temperature Sensor  FixedLabel role=min      MeasuredValue
  ├─ Temperature Sensor  FixedLabel role=max      MeasuredValue
  └─ Temperature Sensor  FixedLabel role=current  MeasuredValue
```

## The two projects

### [`s3_weather/`](s3_weather/) — the front end (plain ESP-IDF, no esp-matter)

Owns Wi-Fi, geocodes your location by name and fetches the forecast from Open-Meteo,
then streams readings to the H2. Wi-Fi credentials, location, units and refresh
interval are entered in the `setup` page over USB serial and saved to NVS. Re-sends on
every refresh and whenever the H2 asks.

### [`h2_matter/`](h2_matter/) — the Matter device (esp-matter)

An aggregator plus three bridged temperature-sensor endpoints with FixedLabel /
UserLabel, served on the Thread mesh — the structure the display binds to. Takes its
readings from the S3 over UART.

### UART line protocol

```
S3 → H2:  WX <min> <max> <cur> <condition…>    temps per unit, "-" = null/unknown
          LOC <location…>
H2 → S3:  READY                                 (H2 booted)
          NEED_WEATHER                           (until the first WX arrives,
                                                  then every 30 s if missing)
```

Pins, per the board schematic: the S3 uses `UART1` on GPIO18 (TX) / GPIO17 (RX); the
H2 uses `UART0` on its default GPIO24 (TX) / GPIO23 (RX). Both at 115200 baud. The
H2's console stays on USB-Serial-JTAG, which leaves UART0 free for this link.
**Check these against your board revision** before flashing — they're `#define`s at
the top of each `app_main`.

## Build and flash

Each chip is a separate project, flashed to its own port.

```bash
source /path/to/esp-idf/export.sh

# S3 front end (no esp-matter needed)
cd s3_weather
idf.py set-target esp32s3
idf.py -p <s3-port> build flash monitor

# H2 Matter device
export ESP_MATTER_PATH=/path/to/esp-matter
source $ESP_MATTER_PATH/export.sh
cd ../h2_matter
idf.py set-target esp32h2
idf.py -p <h2-port> build flash monitor
```

The H2 on this board has 2 MB of flash, so it uses a single factory app partition and
OTA is disabled.

## Setting it up

1. Flash both chips.
2. Open the **S3's** serial console and type `setup`:

   ```
      Weather Device — Setup
      =========================

      Wi-Fi SSID      : my-network
      Wi-Fi Password  : **********
      Location        : Melbourne
      Units           : Celsius
      Refresh         : 60 min

                      : [ Save ]
                      : [ Save & Exit ]

      ↑/↓ move    Enter select/edit    ←/→ toggle    Esc save & exit
   ```

   Set the Wi-Fi credentials first, then pick **Location** and type a place name — it
   geocodes it and lists matches to arrow-pick from, so there are no coordinates to
   look up. Saving connects Wi-Fi and fetches immediately. `poll` fetches on demand.

   The `setup` page needs a real terminal (`idf.py monitor`, `screen`, PuTTY).
   Browser serial consoles send whole lines and never forward arrow keys, so the
   same settings are also plain commands — use these if you flashed from a browser:

   ```
   wifi "my network" "my password"   quote values containing spaces
   location Melbourne                lists numbered matches
   pick 2                            choose one (unnecessary if only one matched)
   units c                           or f
   refresh 60                        minutes between fetches
   save                              connect, fetch, and print the result
   ```

   `show` prints the current configuration and `help` lists everything. `location`
   needs Wi-Fi up, so set `wifi` first. (If you open `setup` in a browser console
   by accident, `q` then Enter exits it.)
3. **Commission the H2** in your Matter app (default test credentials: passcode
   `20202021`, discriminator `3840`), then **bind** the display to it.

> **Unit note:** Matter's temperature field is nominally Celsius, but the paired
> display simply renders the number it receives, so choosing Fahrenheit publishes °F
> directly. A third-party controller reading the raw attribute would read it as °C.
