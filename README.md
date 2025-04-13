# 🧪 STM32G431 – I²C Slave ADC Sampler with Trigger & Calibration

[![License: CC0-1.0](https://img.shields.io/badge/license-CC0--1.0-lightgrey.svg)](https://creativecommons.org/publicdomain/zero/1.0/)
[![STM32CubeIDE](https://img.shields.io/badge/STM32CubeIDE-✅-blue)](https://www.st.com/en/development-tools/stm32cubeide.html)
[![Platform](https://img.shields.io/badge/Platform-STM32G431-informational)](https://www.st.com/en/microcontrollers-microprocessors/stm32g4-series.html)

This project demonstrates how to configure an **STM32G431** as an **I²C slave** that samples **two ADC channels** at **10 kHz for 1 second** (10,000 samples per channel). The result is transmitted to a master (e.g., Raspberry Pi) over I²C. Sampling can be triggered either by I²C command or via **GPIO trigger input on PA8**.

## 📂 Project Structure

```
├── Core/                → STM32 application source
├── Drivers/             → STM32 HAL and CMSIS
├── Python/              → Raspberry Pi script to trigger and read
├── STM32G4_I2C_Slave.ioc → CubeMX config file
├── LICENSE              → CC0 License
└── README.md            → This file
```

## ✨ Features

- 🔌 **I²C Slave Interface**  
  Responds to master commands to control LED, trigger ADC, or transmit data

- 🧠 **Dual ADC Sampling @ 10 kHz**  
  ADC1 and ADC2 capture 8-bit values simultaneously every 100 µs

- 📦 **Data Buffer with CRC8**  
  20,000 bytes of ADC data plus 1 CRC8 byte (20001 total)

- 📶 **Hardware Trigger Support**  
  Optional sampling trigger via **GPIO PA8** for synchronized acquisition

- 🧮 **Onboard Calibration Support**  
  Offset calibration with EEPROM storage (ADC1 = `0x04`/`0x05`, ADC2 = `0x06`/`0x07`)

- 💡 **Status LED**  
  LED controlled by I²C and used as visual sampling indicator

- 🐍 **Python Example for Raspberry Pi**  
  Reads the buffer and checks CRC

## 🔄 I²C Protocol

| Command | Description                             |
|---------|-----------------------------------------|
| `0x00`  | Turn LED **ON**                         |
| `0x01`  | Turn LED **OFF**                        |
| `0x02`  | Start transmission of ADC buffer        |
| `0x03`  | Start sampling (1 s @ 10 kHz, 2 channels)|
| `0x04`  | Start calibration LED-blink for ADC1    |
| `0x05`  | Save offset of ADC1 to EEPROM           |
| `0x06`  | Start calibration LED-blink for ADC2    |
| `0x07`  | Save offset of ADC2 to EEPROM           |

## 💻 Requirements

- ✅ STM32G431K6 (e.g., custom or Nucleo board)
- 💡 LED on `PB5`
- 🧪 Analog inputs on `PA0` (ADC1) and `PA1` (ADC2)
- 🔌 Trigger input on `PA8`
- 🔗 I²C connection to master (e.g., Raspberry Pi)
- 🛠 [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)

## ⚙️ How to Build

```bash
git clone https://github.com/StyriaElectronics/STM32G4_I2C_Slave.git
# Open in STM32CubeIDE: File → Open Project from File System…
```

Then build & flash to your STM32G431.

## 🐍 Python Example (Raspberry Pi)

Inside the `Python/` folder:

```bash
python3 read_adc_data.py
```

This script sends command `0x03`, waits for 1 s, then reads 20001 bytes and checks the CRC.

## 📜 License

This project is released under  
[**Creative Commons Zero (CC0-1.0)**](https://creativecommons.org/publicdomain/zero/1.0/)  
> Use it freely – no restrictions!

## 🙌 Contributions Welcome

Feel free to fork, report issues or submit PRs (especially if adding DMA or improving robustness).

Made with ❤️ by **Styria Electronics**  
🌐 [styria-electronics.at](https://styria-electronics.at)
