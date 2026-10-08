<h1 align="center">
  <br>
  <a href="https://notaroomba.dev"><img src="https://raw.githubusercontent.com/NotARoomba/Athena/main/assets/logo.png" alt="Athena" width="200"></a>
  <br>
  Athena
  <br>
</h1>

<h4 align="center">
Advanced Flight Computer with Triple MCU Architecture

</h4>

<div align="center">

![C](https://img.shields.io/badge/C-%2300599C.svg?style=for-the-badge&logo=c&logoColor=white)
![STM32](https://img.shields.io/badge/STM32-%23FFD200.svg?style=for-the-badge&logo=stmicroelectronics&logoColor=white)
![EasyEDA](https://img.shields.io/badge/EasyEDA-%230F66DC.svg?style=for-the-badge&logo=easyeda&logoColor=white)

</div>

<p align="center">
  <a href="#key-features">Key Features</a> •
  <a href="#board-overview">Board Overview</a> •
  <a href="#specifications">Specifications</a> •
  <a href="#components">Components</a> •
  <a href="#credits">Credits</a> •
  <a href="#license">License</a>
</p>
<img src="https://raw.githubusercontent.com/NotARoomba/Athena/main/assets/pcb_physical.jpg" alt="Athena Flight Computer Physical" width="500"/>

## Key Features

- **Triple MCU Architecture**: STM32H753VIT6 (MPU), STM32H743VIT6 (TPU), STM32G474RET6 (SPU)
- **6 Pyro Channels**: Direct 12V battery connection with fuse protection
- **6 PWM Channels**: 2 for TVC (Thrust Vector Control), 4 for fin control
- **Advanced Sensors**: Triple ICM-45686 IMUs, LIS2MDLTR magnetometer, ICP-20100 & BMP388 barometers
- **GNSS & Communication**: NEO-M8U-06B GPS, LoRa RA-02 telemetry, Bluetooth DA14531MOD
- **Storage**: SD Card + Winbond W25Q256JV flash memory
- **Power Management**: 7.4-12V LiPo battery with BQ25713 charger, TPS25751 USB-C PD controller
- **6-Layer PCB**: Dedicated power planes and signal routing

## Board Overview

Athena is a high-performance flight computer designed for advanced rocketry applications. The board features a sophisticated 6-layer PCB design with dedicated power planes and optimized signal routing.

### Board Images

<img src="https://raw.githubusercontent.com/NotARoomba/Athena/main/assets/board_front.png" alt="Athena Flight Computer Front" width="500"/>
<img src="https://raw.githubusercontent.com/NotARoomba/Athena/main/assets/board_back.png" alt="Athena Flight Computer Back" width="500"/>

### PCB Design Process

The board was designed in EasyEDA with careful attention to power distribution and signal integrity:

<img src="https://raw.githubusercontent.com/NotARoomba/Athena/main/assets/routing_finished.png" alt="Power Plane" width="500"/>
<img src="https://raw.githubusercontent.com/NotARoomba/Athena/main/assets/power_plane.png" alt="Power Plane" width="500"/>
<img src="https://raw.githubusercontent.com/NotARoomba/Athena/main/assets/signal_layer.png" alt="Signal Layer" width="500"/>
<img src="https://raw.githubusercontent.com/NotARoomba/Athena/main/assets/bottom_layer.png" alt="Bottom Layer" width="500"/>

## Specifications

### Physical Specifications

- **Board Size**: 80mm × 140mm
- **Layer Count**: 6 layers
- **Thickness**: Standard PCB thickness
- **Connectors**: USB-C, SD Card slot, various headers

### Power Specifications

- **Input Voltage**: 7.4V LiPo battery
- **Regulated Outputs**: 5V, 3.3V
- **Charging**: USB-C Power Delivery support

### Communication Interfaces

- **GNSS**: NEO-M8U-06B GPS module
- **LoRa**: RA-02 for long-range telemetry
- **Bluetooth**: DA14531MOD for local communication
- **CAN**: TCAN1057AVDRQ1 transceiver
- **USB**: TUSB2036 USB hub
- **UART**: Dedicated UART channels for each STM with ESD protection

## Components

### Microcontrollers

- **MPU (Main Processing Unit)**: STM32H753VIT6 - Handles sensors and Kalman filtering
- **TPU (Telemetry Processing Unit)**: STM32H743VIT6 - Manages LoRa, SD card, and flash memory
- **SPU (Servo Processing Unit)**: STM32G474RET6 - Controls pyro channels and PWM outputs

### Sensors

- **IMU**: 3× ICM-45686 (triple redundancy)
- **Magnetometer**: LIS2MDLTR
- **Barometers**: ICP-20100, BMP388 (dual redundancy)

### Power Management

- **Battery Charger**: BQ25713RSNR (on the TPS25751's I2C controller bus)
- **USB-C PD Controller**: TPS25751DREFR (patched over I2C by the SPU at power-up)
- **Buck Converters**: LM5145RGYR (servo), TPS5430 (3.3V)

### Storage & Communication

- **Flash Memory**: Winbond W25Q256JV
- **SD Card**: Standard microSD slot
- **GPS**: NEO-M8U-06B
- **LoRa**: RA-02
- **Bluetooth**: DA14531MOD-00F01002

## Dashboard

[docs/](docs/) is the built dashboard (source in [software/web](software/web): React 19 + Vite +
Tailwind + recharts + three.js, same stack as [cyberboard.notaroomba.dev](https://cyberboard.notaroomba.dev)),
served at [athena.notaroomba.dev](https://athena.notaroomba.dev) by GitHub Pages. It shows the 3D
attitude, accel/gyro/altitude charts, flight flags, GPS, a live **ground-track map** (filter
estimate, dead-reckoned stretches dashed, raw GPS, landing estimate from the current descent,
distance/bearing from the pad, apogee and max speed), and the SPU **recovery & power** panel
(flight phase, arming, the six pyro channels, battery/charger/USB-PD state) with ARM / DISARM /
FIRE / servo / main-altitude commands when a writable link is open, a **flight event timeline**
(launch, phases, pyro firings, arming) with a summary line (apogee, max speed, max g, flight time,
landing distance), per-type frame rates, a **REC** button that saves the raw link stream as a
replayable `.bin`, a **REPORT** button that downloads a JSON flight report (summary, events, tracks, last
frames, console), a **pre-flight checklist** (GO/NO-GO read from the live data: IMUs, baro, GPS fix,
pad origin, SPU link, main altitude, radio, arming, plus a hand-ticked list), **SOUND** alerts on
launch/apogee/pyro/landing, a **T-60 countdown** (beeps over the last 10 s, cancels itself at launch), a no-data
banner, the altitude and phase in the tab title, and keyboard shortcuts (`d` demo, `t` countdown, `c` checklist,
`s` sound, `l` login, `r` record). Data sources:

- **SERIAL**: any of the three USB ports (WebSerial, Chrome/Edge over https or localhost).
- **BLUETOOTH**: the DA14531 on the TPU over Web Bluetooth (DSPS serial-bridge firmware streams the
  telemetry; the factory CodeLess firmware only answers AT commands).
- **REPLAY**: an `ATHnnnnn.BIN` from the SD card, a flash dump, a dashboard recording or a ground-station
  log, paced by its own timestamps, with a speed selector and a seek bar.
- **DEMO**: a scripted flight (with a GPS dropout) through the real encoder/decoder.
- viewers: an admin's serial/Bluetooth bytes (or a ground station's) are relayed through the WebSocket
  server in [software/server](software/server) (axum, on Railway at `api.athena.notaroomba.dev`). A logged-in
  viewer can also send commands: the server hands them to the connected ground station, which writes them
  to its uplink port, and reports whether anyone was there to carry them.

Locally: `cd software/web && npm install && npm run dev` (or `python3 -m http.server 8787 --directory docs`).

## Firmware

Three CubeMX projects (`firmware/MPU`, `firmware/TPU`, `firmware/SPU`) share the pure-C
modules in `firmware/Athena`: the inter-MCU/LoRa framing (`athena_link`), the UBX parser
(`ubx`), the navigation filter (`fusion`: Mahony attitude + per-axis Kalman filters with
IMU dead reckoning, barometer and GPS corrections) and the SPU recovery logic (`recovery`:
flight phase from the fused state, drogue at apogee, main below a set altitude, servo outputs).
`cd firmware && make debug` builds all three (`brew install osx-cross/arm/arm-gcc-bin@14` for the
toolchain) and `make host-test` runs the PC self-check of the shared modules including a scripted
flight through the recovery logic.

`./firmware/flash.sh` flashes over USB DFU with `dfu-util` and tells the MCUs apart by USB hub
port. No buttons are needed: `B` on any MCU's USB console disconnects USB, leaves a magic word in
RAM and resets into the ST ROM bootloader (`J` jumps in place as a fallback); the script does this
itself. After reset each MCU shows its identity colour for 3 s: **MPU green, TPU red, SPU blue**.

Links: MPU -> TPU (UART4/UART8, state 20 Hz, GPS back), MPU -> SPU (UART8/UART5, state 20 Hz,
SPU status 2 Hz back), TPU -> MPU -> SPU for commands (LoRa uplink, USB, Bluetooth). The TPU
logs every link frame to the microSD card (one `ATHnnnnn.BIN` per boot, card hot-plug safe) and
to the W25Q256 flash, sends telemetry + SPU status over LoRa and over UART7 to the Bluetooth module;
`tools/athlog.py` dumps the flash log over USB and converts logs to CSV.

**Ground station on an RTL-SDR** (`tools/lora_rx.py`, macOS/Linux/Windows): a complete LoRa receiver for
the downlink (SF7, 125 kHz, sync 0x12) written in numpy, no GNU Radio, about 15% of one core at the
dongle's native 1 MS/s. It decodes the Athena frames straight from the IQ stream (the frames' own CRCs
verified 92% of packets on the bench), logs them to a replayable `athena-lora-*.bin`, and by default opens
the web dashboard fed live by this process (it serves `docs/` and speaks the relay protocol on
`ws://localhost:3001/ws`, including a `station` message with signal level, carrier offset and packet
counts that the dashboard shows in its footer, and its own log lines as TEXT frames so they appear in the
dashboard console; through `--relay` the public site gets both as well). `--app` opens it as a desktop window (pywebview) instead of
a browser tab, `--uplink /dev/tty...` adds a command path (the rocket's USB console on the bench, or a TPU
in ground-station mode: `G` on its console turns a second board into a LoRa<->USB relay), `--tui` gives a
terminal UI, `--relay wss://api.athena.notaroomba.dev/ws --password ...` also feeds the public site and
accepts commands from it, `--file cap.cu8` replays a capture. Needs `pip install numpy scipy websockets
websocket-client pyserial pywebview` and `rtl_sdr` (`brew install librtlsdr`). `tools/build_station.sh` packs
it into one standalone executable (`dist/station/athena-station`, PyInstaller, dashboard files included) so a
laptop at the range only needs `rtl_sdr` installed. `tools/lora_check.py` is the quick PHY check (burst period,
preamble, sync word).

USB console characters: all MCUs `B`/`J` (DFU), `L` LEDs off/on; TPU `D` dump flash log, `E` restart it,
`S` sync SD, `F` format SD, `G` ground-station mode (persistent); SPU `A` arm, `d` disarm, `1`-`6` fire a
channel (armed only), `s` sweep servo 1, `r` reset the MPU. Pyros only get power when the external ARM terminal is closed.

## Credits

This project uses:

- [EasyEDA](https://easyeda.com/) - PCB design and schematic capture
- [STM32 HAL](https://www.st.com/en/embedded-software/stm32cube-hal.html) - Hardware abstraction layer
- [JLCPCB](https://jlcpcb.com/) - PCB manufacturing and assembly
- [Figma](https://figma.com/) - Silkscreen design

## You may also like...

- [Niveles De Niveles](https://github.com/NotARoomba/NivelesDeNiveles) – Real-time flood alert app
- [Linea](https://github.com/NotARoomba/Linea) – An EMR tablet
- [Tamaki](https://github.com/NotARoomba/Tamaki) – A cute HackPad

## License

MIT

---

> [notaroomba.dev](https://notaroomba.dev) &nbsp;&middot;&nbsp;
> GitHub [@NotARoomba](https://github.com/NotARoomba)
