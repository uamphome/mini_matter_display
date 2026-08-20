# Browser flashing with ESP Launchpad

[ESP Launchpad](https://github.com/espressif/esp-launchpad) is Espressif's
browser flasher. You don't host the tool — you host three merged firmware images
and a `config.toml`, and link people to Espressif's copy of Launchpad pointed at
your config:

```
https://espressif.github.io/esp-launchpad/?flashConfigURL=https://<user>.github.io/<repo>/config.toml
```

Launchpad detects which chip is plugged in and offers only the matching app, so
one link covers the display (C6), the weather front end (S3) and the weather
Matter device (H2).

## What's here

| File | Purpose |
| --- | --- |
| `config.toml.in` | Launchpad config template; `@BASE_URL@` is substituted at build time |
| `readme-intro.md` | Shown at the top of the Launchpad page |
| `postflash-*.md` | Shown after each app is flashed |

`scripts/build_launchpad.sh` builds all three projects, runs `idf.py merge-bin`
on each, and stages everything under `dist/launchpad/`.

## Build

```bash
. /path/to/esp-idf/export.sh
export ESP_MATTER_PATH=/path/to/esp-matter && . $ESP_MATTER_PATH/export.sh

scripts/build_launchpad.sh v1.0.0
```

Images land in `dist/launchpad/fw/v1.0.0/`, and `dist/launchpad/config.toml` gets
the matching URLs. Images are versioned but `config.toml` sits at a stable path,
so a link in a video description keeps working across releases.

## Publish to GitHub Pages

First time only, create an empty branch for the site:

```bash
git switch --orphan gh-pages
git commit --allow-empty -m "Initialise GitHub Pages branch"
git push -u origin gh-pages
git switch main
```

Then enable Pages in the repo settings: **Settings → Pages → Source: Deploy from
a branch → `gh-pages` / `(root)`**.

To publish a build:

```bash
git worktree add ../mmd-pages gh-pages
rsync -a dist/launchpad/ ../mmd-pages/
cd ../mmd-pages && git add -A && git commit -m "Publish firmware v1.0.0" && git push
```

Keeping the binaries on `gh-pages` means they never enter `main`'s tree.

## Why GitHub Pages and not Releases

Launchpad runs on `espressif.github.io` and fetches your files cross-origin, so
they must be served with `Access-Control-Allow-Origin`. GitHub **Pages** sends
`access-control-allow-origin: *`; GitHub **release assets** send no CORS headers
at all — not on the `github.com/.../releases/download/...` redirect, and not on
the `release-assets.githubusercontent.com` response it redirects to. The browser
blocks the fetch before esptool ever sees it, so release assets can't be used
here. `raw.githubusercontent.com` does send `*` and would also work, but then the
binaries live in `main`.

## Notes

- **Chrome, Edge or Opera on desktop only.** Web Serial doesn't exist in Safari
  or Firefox, or on mobile.
- Each `.bin` is a whole flash image written at `0x0`. `merge-bin` pads the gaps
  with `0xFF`, so flashing also clears NVS: boards come up uncommissioned, and
  the S3 loses its saved Wi-Fi and location.
- The `setup_payload` values in `config.toml.in` are esp-matter's default test
  credentials. If you ever change the VID/PID, update them — the display's
  on-screen QR and the H2's boot log are authoritative.
- Launchpad's Console tab sends **whole lines** and never forwards arrow keys or
  Esc, which is why `s3_weather` has line commands (`wifi`, `location`, `pick`,
  `units`, `refresh`, `save`) alongside the arrow-key `setup` page. Don't remove
  them without also giving browser users another way in.
