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

## Key Features

- **Triple MCU Architecture**: STM32H753VIT6 (MPU), STM32H743VIT6 (TPU), STM32G474RET6 (SPU)
- **6 Pyro Channels**: Direct 12V battery connection with fuse protection
- **6 PWM Channels**: 2 for TVC (Thrust Vector Control), 4 for fin control
- **Advanced Sensors**: Triple ICM-45686 IMUs, LIS2MDLTR magnetometer, ICP-20100 & BMP388 barometers
- **GNSS & Communication**: NEO-M8U-06B GPS, LoRa RA-02 telemetry, Bluetooth DA14531MOD
- **Storage**: SD Card + Winbond W25Q256JV flash memory
- **Power Management**: 7.4-12V LiPo battery with BQ25703ARSNR charger, USB-C PD support
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

- **Battery Charger**: BQ25703ARSNR
- **USB-C PD Controller**: TPS25750
- **Buck Converters**: LM5145RGYR (servo), TPS5430 (3.3V)

### Storage & Communication

- **Flash Memory**: Winbond W25Q256JV
- **SD Card**: Standard microSD slot
- **GPS**: NEO-M8U-06B
- **LoRa**: RA-02
- **Bluetooth**: DA14531MOD-00F01002

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
