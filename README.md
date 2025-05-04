# 🧪 STM32G431 – I²C Slave ADC Sampler with Trigger & Calibration

[![License: CC0-1.0](https://img.shields.io/badge/license-CC0--1.0-lightgrey.svg)](https://creativecommons.org/publicdomain/zero/1.0/)
![Made with KiCad](https://img.shields.io/badge/Made%20with-KiCad-005cad?logo=kicad)
[![STM32CubeIDE](https://img.shields.io/badge/STM32CubeIDE-✅-blue)](https://www.st.com/en/development-tools/stm32cubeide.html)
[![Platform](https://img.shields.io/badge/Platform-STM32G431-informational)](https://www.st.com/en/microcontrollers-microprocessors/stm32g4-series.html)

This project demonstrates how to configure an **STM32G431** as an **I²C slave** that samples **two ADC channels** at **10 kHz for 1 second** (10,000 samples per channel). The result is transmitted to a master (e.g., Raspberry Pi) over I²C. Sampling can be triggered either by I²C command or via **GPIO trigger input on PA8**.

## 📂 Project Structure

```
├── Core/                → STM32 application source
├── Drivers/             → STM32 HAL and CMSIS
├── Hardware/            → KiCad PCB design files
├── Python3_Script/      → Raspberry Pi script to trigger and read
├── docs/                → Documentation and visuals
├── .gitignore           → Git exclusions
├── I2C_ADC.ioc          → CubeMX configuration file
├── LICENSE              → CC0 license
└── README.md            → This file
```

## ✨ Features

- 🔌 **I²C Slave Interface**
- 📈 **Dual 10 kHz ADC Sampling**
- ⏱ **Timer-based precision using TIM6**
- 🔦 **Status LED controlled via I²C**
- 🧮 **CRC8 for data integrity**
- 🐍 **Python script for Raspberry Pi**
- 🛠️ **KiCad hardware project included**

## 🔄 I²C Protocol

| Command | Description                             |
|---------|-----------------------------------------|
| `0x00`  | Turn LED **ON**                         |
| `0x01`  | Turn LED **OFF**                        |
| `0x02`  | Start ADC buffer transmission (20001 B) |
| `0x03`  | Start 1 s sampling                      |
| `0x04`  | Start calibration (ADC1 blink)          |
| `0x05`  | Save offset of ADC1                     |
| `0x06`  | Start calibration (ADC2 blink)          |
| `0x07`  | Save offset of ADC2                     |

## 🛠 Hardware

The `Hardware/` folder contains the KiCad files for the PCB design:

- Schematic (`.kicad_sch`)
- Layout (`.kicad_pcb`)
- Gerber files
- BOM (Bill of Materials)

These files allow you to fabricate or modify the board as needed.

## 🐍 Python Example (Raspberry Pi)

A script using `smbus2` to trigger and read the data from the STM32:

```bash
cd Python3_Script/
python3 read_adc_data.py
```

Output will display the values of both ADC channels and CRC validation.

## 📄 License

This project is licensed under **CC0 1.0 Universal (Public Domain Dedication)**. You can use it freely without attribution – though a ⭐ on GitHub is always appreciated :)

⚠️ **Disclaimer**:  
This project is provided "as is" without warranty of any kind, either expressed or implied.  
In no event shall the author(s) be liable for any claim, damages, or other liability arising from, out of, or in connection with the use or other dealings in this project.

## 🙌 Contributing

Feel free to fork the repository, open issues, or submit pull requests.  
PRs improving timing, robustness or adding DMA support are welcome!

## ⭐️ If You Like It...

...consider giving this repo a ⭐️ to support the project!

Made with ❤️ by **Styria Electronics**  
🌐 [styria-electronics.at](https://styria-electronics.at)
