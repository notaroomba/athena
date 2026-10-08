#!/bin/sh
# Builds the ground station as a standalone app folder (dist/station/athena-station/athena-station[.exe]):
# the LoRa receiver, the local relay, the dashboard files (docs/) and the learned PHY conventions, no Python
# needed on the target. A folder rather than --onefile: that variant unpacked 32 MB on every launch (23 s to start).
# Needs Python 3.10+ and rtl_sdr on the PATH at run time (brew install librtlsdr / apt install rtl-sdr /
# the Windows release zip). Usage: tools/build_station.sh   then   dist/station/athena-station --app
set -e
cd "$(dirname "$0")/.."
python3 -m venv dist/venv
. dist/venv/bin/activate 2>/dev/null || . dist/venv/Scripts/activate
pip install -q -r tools/requirements-station.txt pyinstaller
pyinstaller --noconfirm --onedir --name athena-station \
  --distpath dist/station --workpath dist/build --specpath dist \
  --add-data "$(pwd)/tools/lora_conv.json:." --add-data "$(pwd)/docs:docs" \
  --collect-all webview --hidden-import websockets --hidden-import websocket --hidden-import serial \
  tools/lora_rx.py
echo "built dist/station/athena-station/athena-station (zip the folder to hand it out)"
