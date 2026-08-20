# Display — flashed

The board should reboot into the UI and show a pairing QR code on screen.

## Commission it

Scan the QR **on the display** with your Matter app. The code on this page is the
same default test payload, but the on-screen one always matches the firmware
that's actually running.

```
Pairing code : 34970112332
Passcode     : 20202021
Discriminator: 3840
```

The display joins over **Thread**, so you need a Thread border router on your
network — an Apple TV or HomePod, a Google Nest Hub, a Home Assistant border
router, or Espressif's own TBR board.

## Then bind it

The display is a Colour Dimmer Switch: it does nothing on its own until your
controller **binds** it to devices. Bind it to

- lights, for on/off, brightness and colour;
- temperature / humidity / air-quality sensors, for the Indoor page;
- contact sensors and leak detectors, for alerts;
- the weather device from this repo, for the Weather page.

Binding is done in your controller, not on the display.

## Notes

- Flashing erased NVS, so the board is uncommissioned and has no saved fabric.
- The clock can only be set from your Matter fabric, and controllers are
  unreliable about re-pushing the time after a reboot — tap the clock card to set
  it by hand.
- Swipe up from Home for settings: brightness, display timeout, reboot, factory
  reset.
- The display is a sleepy Thread device, so once the screen turns off it checks
  in with the network every few seconds rather than listening constantly. An
  alert can therefore arrive a few seconds late while the screen is off; tapping
  a light is unaffected.
