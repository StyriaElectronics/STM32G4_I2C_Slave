# 🧪 STM32G431 – I²C Slave ADC Sampling Example

[![License: CC0-1.0](https://img.shields.io/badge/license-CC0--1.0-lightgrey.svg)](https://creativecommons.org/publicdomain/zero/1.0/)
[![STM32CubeIDE](https://img.shields.io/badge/STM32CubeIDE-✅-blue)](https://www.st.com/en/development-tools/stm32cubeide.html)
[![Platform](https://img.shields.io/badge/Platform-STM32G431-informational)](https://www.st.com/en/microcontrollers-microprocessors/stm32g4-series.html)

This project demonstrates how to configure an **STM32G431** as an **I²C slave** that continuously samples **two analog input channels** via the ADC at **1 kHz** each. The sampled data is made available to an I²C master device (e.g., Raspberry Pi) upon request.

---

## 📂 Project Contents

```
├── Core/                → Application source code
├── Drivers/             → STM32 HAL and CMSIS drivers
├── Python/              → Python example script to read ADC values via I²C
├── .gitignore           → Git exclusions
├── STM32G4_I2C_Slave.ioc → CubeMX config file
├── LICENSE              → CC0 License file
└── README.md            → This file
```

---

## ✨ Features

- 🔌 **I²C Slave Communication**  
  Responds to I²C master read requests and transmits buffered ADC data

- 📈 **Dual ADC Sampling**  
  Two analog channels sampled at 1 kHz using STM32's ADC1 and ADC2

- ⏱ **Timer-Based Sampling**  
  TIM6 is used to trigger samples precisely every 1 ms

- 🔦 **Status LED**  
  On-board LED toggled via I²C command (`0x00` = ON, `0x01` = OFF) for quick connectivity check

- 🐍 **Python Script for Raspberry Pi**  
  Included Python3 script to request sampling and read the 2×1000 8-bit values plus CRC via I²C

- 🧮 **CRC8 Check**  
  Ensures data integrity with a CRC byte appended to each transmission

- 🧰 **Fully STM32CubeIDE Compatible**  
  Ready to build and flash via STM32CubeIDE (tested on G431K6)

---

## 🧱 Requirements

- ✅ STM32G431 development board (e.g., Nucleo-G431KB or custom board)
- 💻 [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)  
- 🔌 USB or external 3.3 V power supply
- 🔗 I²C Master (e.g. Raspberry Pi)

---

## ⚙️ How to Build

```bash
# Clone the repository
git clone https://github.com/StyriaElectronics/STM32G4_I2C_Slave.git

# Open with STM32CubeIDE
STM32CubeIDE → File → Open Projects from File System...
```

Then build and flash the project to your STM32G431.

---

## 🔄 I²C Protocol

| Command | Description                      |
|---------|----------------------------------|
| `0x00`  | Turn LED **on**                  |
| `0x01`  | Turn LED **off**                 |
| `0x03`  | Start 1-second ADC sampling      |
| `0x02`  | Request ADC buffer (2001 bytes)  |

---

## 🐍 Example Python Script (Raspberry Pi)

A sample script using `smbus2` to control the STM32 and read ADC data:

```bash
cd Python/
python3 read_adc_data.py
```

Output will display ADC values of both channels as hex data, including CRC check result.

---

## 📜 License

This project is licensed under the  
[**Creative Commons Zero v1.0 (CC0-1.0)**](https://creativecommons.org/publicdomain/zero/1.0/).  
> No restrictions – use, modify, and distribute freely!

---

## 🙌 Contributing

Feel free to fork the repository, open issues, or submit pull requests.  
PRs improving timing, robustness or adding DMA support are welcome!

---

## ⭐️ If You Like It...

...consider giving this repo a ⭐️ to support the project!

---

Made with ❤️ by **Styria Electronics**  
🌐 [styria-electronics.at](https://styria-electronics.at)

