#!/bin/sh
# Flash the three Athena MCUs over USB DFU (hold BOOT on each, then reset/power up).
#
#   ./flash.sh            flash all three
#   ./flash.sh mpu|tpu|spu
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

case "${1:-all}" in
  list) echo "DFU devices:"; list ;;
  all|mpu|tpu|spu)
    want=$1; found=0
    echo "DFU devices:"; list
    for p in $(devices); do
      mcu=$(mcu_for_port "$(port_of "$p")")
      [ -z "$mcu" ] && { echo "skipping $p: not on a known hub port"; continue; }
      if [ "$want" = all ] || [ "$want" = "$mcu" ]; then flash_one "$mcu" "$p"; found=$((found+1)); fi
    done
    [ $found -eq 0 ] && { echo "nothing flashed"; exit 1; }
    echo "done: $found device(s). Identity LEDs for the first 3 s: MPU green, TPU red, SPU blue."
    echo "Serial consoles appear as /dev/cu.usbmodem*; each prints '=== Athena <MCU> ===' at boot." ;;
  *) echo "usage: $0 [all|mpu|tpu|spu|list]"; exit 1 ;;
esac
