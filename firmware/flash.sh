#!/bin/sh
# Flash the three Athena MCUs over USB DFU (hold BOOT on each, then reset/power up).
#
#   ./flash.sh            flash all three (MCUs running the firmware are rebooted into DFU over USB first)
#   ./flash.sh mpu|tpu|spu
#   ./flash.sh dfu [mcu]  just reboot the running MCU(s) into DFU
#   ./flash.sh list       just show what dfu-util sees
#
# Which chip is which comes from the PCB: the TUSB2036 hub's downstream port 1 is the
# SPU, port 2 the MPU, port 3 the TPU, and dfu-util reports that port as the last number
# of the device path (e.g. path="2-1.3" -> hub port 3 -> TPU). The STM32 ROM bootloaders
# all enumerate as 0483:df11 and the two H7 parts even share a serial number, so the
# port is the only reliable key. Needs: brew install dfu-util.
set -e
cd "$(dirname "$0")"
CONFIG=${CONFIG:-debug}

need() { command -v "$1" >/dev/null 2>&1 || { echo "missing $1 (brew install $1)"; exit 1; }; }
need dfu-util

# "port -> path" table from dfu-util -l (one line per alt setting; keep unique paths)
devices() {
  dfu-util -l 2>/dev/null | grep '0483:df11' | sed -n 's/.*path="\([^"]*\)".*/\1/p' | sort -u
}
port_of() { echo "$1" | sed 's/.*[.-]\([0-9][0-9]*\)$/\1/'; }

mcu_for_port() {
  case "$1" in 1) echo spu;; 2) echo mpu;; 3) echo tpu;; *) echo "";; esac
}

list() {
  n=0
  for p in $(devices); do
    n=$((n+1)); port=$(port_of "$p"); echo "  path=$p  hub port $port  -> $(mcu_for_port "$port" | tr a-z A-Z)"
  done
  [ $n -eq 0 ] && echo "  no STM32 bootloader (0483:df11) on USB. Hold BOOT, press RESET, check the cable carries data."
  return 0
}

flash_one() {   # $1 = mcu (lower), $2 = dfu path
  mcu=$1; path=$2; up=$(echo "$mcu" | tr a-z A-Z)
  bin="$up/build/$CONFIG/${up}_Firmware.bin"
  [ -f "$bin" ] || { echo "$bin not found: run 'make $CONFIG' first"; exit 1; }
  echo "== $up  <- $bin  (dfu path $path)"
  # -a 0 = internal flash, :leave = run the application afterwards. dfu-util often exits
  # non-zero right after the leave request because the device has already rebooted, so
  # judge success by its own "File downloaded successfully" line instead of the exit code.
  out=$(dfu-util -p "$path" -a 0 -s 0x08000000:leave -D "$bin" 2>&1 | tr '\r' '\n' | grep -v "^\(Erase\|Download\)\s*\[" ) || true
  echo "$out" | grep -v "^dfu-util\|Copyright\|License\|Please report\|^$" | tail -6
  echo "$out" | grep -q "File downloaded successfully" || { echo "!! $up: download did not complete"; return 1; }
  sleep 1
}

# Map a USB console (/dev/cu.usbmodem<serial>1) to its MCU through the hub port in ioreg's locationID
# (last non-zero hex digit: 1 = SPU, 2 = MPU, 3 = TPU).
console_mcu() {
  serial=$(basename "$1" | sed 's/^cu\.usbmodem//; s/1$//')
  loc=$(ioreg -p IOUSB -l -w0 2>/dev/null | awk -v s="$serial" '
    /^[ |]*\+-o / { loc=""; ser="" }
    /"locationID"/ { gsub(/[^0-9]/, "", $0); loc=$0 }
    /"USB Serial Number"/ { split($0, a, "\""); ser=a[4] }
    ser == s && loc != "" { print loc; exit }')
  [ -z "$loc" ] && { echo ""; return; }
  hex=$(printf '%x' "$loc" | sed 's/0*$//'); port=${hex##*[!0-9]}; port=$(printf '%s' "$hex" | tail -c 1)
  mcu_for_port "$port"
}

# Ask the running MCU(s) (USB serial console, command 'B') to reboot into DFU, then wait for them.
enter_dfu() {   # $1 = all|mpu|tpu|spu   -- 'B': USB disconnect + magic word + reset, ROM bootloader from a clean chip; 'J' = in-place jump fallback
  n=0
  for p in /dev/cu.usbmodem*; do
    [ -e "$p" ] || continue
    m=$(console_mcu "$p")
    if [ "$1" = all ] || [ "$m" = "$1" ]; then
      stty -f "$p" raw -echo 2>/dev/null || true; printf 'B' > "$p" 2>/dev/null && { n=$((n+1)); echo "asking ${m:-?} ($p) to enter DFU"; }
    fi
  done
  [ $n -eq 0 ] && return 0
  i=0; while [ $i -lt 16 ]; do sleep 0.5; [ "$(devices | wc -l | tr -d ' ')" -ge "$n" ] && break; i=$((i+1)); done
  if [ "$(devices | wc -l | tr -d ' ')" -lt "$n" ]; then           # fall back to the in-place jump
    for p in /dev/cu.usbmodem*; do [ -e "$p" ] || continue; m=$(console_mcu "$p"); if [ "$1" = all ] || [ "$m" = "$1" ]; then stty -f "$p" raw -echo 2>/dev/null || true; printf 'J' > "$p" 2>/dev/null; fi; done
    i=0; while [ $i -lt 16 ]; do sleep 0.5; [ "$(devices | wc -l | tr -d ' ')" -ge "$n" ] && break; i=$((i+1)); done
  fi
  sleep 1
}

case "${1:-all}" in
  list) echo "DFU devices:"; list ;;
  dfu)  enter_dfu "${2:-all}"; echo "DFU devices:"; list ;;
  all|mpu|tpu|spu)
    want=$1; found=0
    enter_dfu "$want"
    echo "DFU devices:"; list
    for p in $(devices); do
      mcu=$(mcu_for_port "$(port_of "$p")")
      [ -z "$mcu" ] && { echo "skipping $p: not on a known hub port"; continue; }
      if [ "$want" = all ] || [ "$want" = "$mcu" ]; then flash_one "$mcu" "$p"; found=$((found+1)); fi
    done
    [ $found -eq 0 ] && { echo "nothing flashed"; exit 1; }
    echo "done: $found device(s). Identity LEDs for the first 3 s: MPU green, TPU red, SPU blue."
    echo "Serial consoles appear as /dev/cu.usbmodem*; each prints '=== Athena <MCU> ===' at boot." ;;
  *) echo "usage: $0 [all|mpu|tpu|spu|dfu|list]"; exit 1 ;;
esac
