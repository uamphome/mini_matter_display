#!/usr/bin/env bash
#
# Build all three firmwares as single merged flash images and stage them, with an
# ESP Launchpad config.toml, under dist/launchpad/ — ready to publish to GitHub
# Pages. See launchpad/README.md for the publishing step.
#
# Prerequisites, in this shell:
#     . /path/to/esp-idf/export.sh
#     export ESP_MATTER_PATH=/path/to/esp-matter && . $ESP_MATTER_PATH/export.sh
#
# Usage: scripts/build_launchpad.sh [version]
#   version defaults to `git describe`. Images are published under fw/<version>/
#   so older config.toml links keep working; config.toml itself is stable.
#
# Override the site URL if your Pages site isn't <user>.github.io/<repo>:
#     PAGES_URL=https://example.com/ scripts/build_launchpad.sh

set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_root"

version=${1:-$(git describe --tags --always 2>/dev/null || echo dev)}

command -v idf.py >/dev/null || {
    echo "idf.py not found — source esp-idf's export.sh first." >&2
    exit 1
}

if [ -z "${PAGES_URL:-}" ]; then
    slug=$(git remote get-url origin 2>/dev/null |
           sed -E 's#^(git@github\.com:|https://github\.com/)##; s#\.git$##') || slug=""
    [ -n "$slug" ] || {
        echo "No 'origin' remote to derive the URL from." >&2
        echo "Set PAGES_URL=https://<user>.github.io/<repo>/ and re-run." >&2
        exit 1
    }
    PAGES_URL="https://${slug%%/*}.github.io/${slug##*/}/"
fi
base_url="${PAGES_URL%/}/fw/${version}/"

out=dist/launchpad
fw="$out/fw/$version"
rm -rf "$fw"
mkdir -p "$fw"

# name : target : project dir : needs esp-matter
projects=(
    "matter_display:esp32c6:matter_display:yes"
    "s3_weather:esp32s3:matter_weather_device/s3_weather:no"
    "h2_matter:esp32h2:matter_weather_device/h2_matter:yes"
)

for spec in "${projects[@]}"; do
    IFS=: read -r name target dir needs_matter <<< "$spec"

    if [ "$needs_matter" = yes ] && [ -z "${ESP_MATTER_PATH:-}" ]; then
        echo "$name needs esp-matter: export ESP_MATTER_PATH and source its export.sh." >&2
        exit 1
    fi

    echo
    echo "=== $name ($target) ==="
    (
        cd "$dir"
        # set-target regenerates sdkconfig from the defaults and forces a full
        # rebuild, so only do it when there's nothing configured yet.
        [ -f sdkconfig ] || idf.py set-target "$target"
        idf.py build
        idf.py merge-bin
    )
    cp "$dir/build/merged-binary.bin" "$fw/$name.bin"
done

cp launchpad/readme-intro.md launchpad/postflash-*.md "$fw/"
sed "s#@BASE_URL@#${base_url}#g" launchpad/config.toml.in > "$out/config.toml"

echo
echo "Staged in $out:"
ls -l "$fw" | sed 's/^/  /'
echo
echo "Flash URL once published:"
echo "  https://espressif.github.io/esp-launchpad/?flashConfigURL=${PAGES_URL%/}/config.toml"
echo
echo "Publish with the steps in launchpad/README.md."
