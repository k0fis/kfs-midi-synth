#!/bin/sh
# Download latest midi_synth.bin from GitHub release and flash via ST-Link
set -e

REPO="k0fis/kfs-midi-synth"
BIN="midi_synth.bin"
TMP="/tmp/$BIN"

echo "Fetching latest release..."
URL=$(curl -s "https://api.github.com/repos/$REPO/releases/latest" \
    | grep "browser_download_url.*$BIN" \
    | cut -d '"' -f 4)

if [ -z "$URL" ]; then
    echo "Error: midi_synth.bin not found in latest release"
    exit 1
fi

echo "Downloading $URL"
curl -sL -o "$TMP" "$URL"
echo "Downloaded $(wc -c < "$TMP" | tr -d ' ') bytes"

echo "Flashing..."
st-flash --connect-under-reset write "$TMP" 0x08000000
st-flash reset

echo "Done."
