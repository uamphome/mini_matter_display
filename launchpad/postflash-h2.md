# H2 Matter device — flashed

This chip is the Matter-over-Thread side of the weather device: an aggregator
endpoint with three bridged temperature sensors (min / max / current), fed by the
S3 over the inter-chip UART.

It has no screen, so pair it with the QR code shown on this page, or the manual
pairing code:

```
Pairing code : 34970112332
Passcode     : 20202021
Discriminator: 3840
```

These are Espressif's **test** commissioning credentials — fine for a hobby build,
not for a product. If pairing is rejected, open the Console tab, press **Reset
Device**, and read the payload the firmware prints at boot; that's authoritative.

## Steps

1. Commission it in your Matter app (Apple Home, Google Home, Home Assistant…).
   You need a **Thread border router** on your network already.
2. Flash `s3_weather` through the board's other USB-C port and configure Wi-Fi
   and your location there — until it does, this device reports no readings.
3. **Bind** the display to it in your controller, so the display can read the
   weather endpoints.

Nothing appears on the display until both the binding and the S3's first
successful fetch have happened.
