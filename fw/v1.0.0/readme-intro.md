# Mini Matter Display — browser flashing

Flash all three firmwares from this page, no toolchain to install.

**You need Chrome, Edge or Opera on a desktop.** Web Serial doesn't exist in
Safari or Firefox, or on iOS/Android.

| App | Board | Plug into |
| --- | --- | --- |
| `matter_display` | Waveshare ESP32-C6-Touch-AMOLED-1.8 | the board's only USB-C port |
| `s3_weather` | ESP Thread Border Router board | the **ESP32-S3** USB-C port |
| `h2_matter` | ESP Thread Border Router board | the **ESP32-H2** USB-C port |

The weather device is one PCB with two chips and **two USB-C ports** — one per
chip. Flash each half through its own port; there's nothing to jumper.

Pick your app, hit **Connect**, choose the port, then **Flash**. Each image is a
full flash image, so flashing also erases NVS — a freshly flashed board is
uncommissioned and ready to pair.

Suggested order: `h2_matter` → `s3_weather` (configure it in the Console tab) →
`matter_display`.

Source, schematics and the full build instructions:
<https://github.com/uamphome/mini_matter_display>
