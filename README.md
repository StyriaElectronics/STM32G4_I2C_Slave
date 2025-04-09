# 🧪 STM32G431 – I²C Slave ADC Sampling Example

[![License: CC0-1.0](https://img.shields.io/badge/license-CC0--1.0-lightgrey.svg)](https://creativecommons.org/publicdomain/zero/1.0/)
[![STM32CubeIDE](https://img.shields.io/badge/STM32CubeIDE-✅-blue)](https://www.st.com/en/development-tools/stm32cubeide.html)
[![Platform](https://img.shields.io/badge/Platform-STM32G431-informational)](https://www.st.com/en/microcontrollers-microprocessors/stm32g4-series.html)

This project demonstrates how to configure an **STM32G431** as an **I²C slave** that continuously samples **two analog input channels** via the ADC at **1 kHz** each. The sampled data is made available to an I²C master device upon request.

---

## 📂 Project Contents

```
├── Core/                → Application source code
├── Drivers/             → STM32 HAL and CMSIS drivers
├── .gitignore           → Git exclusions
├── STM32G4_I2C_Slave.ioc → CubeMX config file
├── LICENSE              → CC0 License file
└── README.md            → This file
```

---

## ✨ Features

- 🔌 **I²C Slave Communication**  
  Responds to I²C master read requests and transmits ADC data

- 📈 **Dual ADC Sampling**  
  Two analog channels sampled at 1 kHz using STM32's ADC

- ⏱ **Timer or DMA-Based Sampling** *(configurable)*  
  Ensures consistent and precise acquisition timing

- 🧰 **Fully STM32CubeIDE Compatible**  
  Ready to build and flash via STM32CubeIDE (tested on G431K6)

---

## 🧱 Requirements

- ✅ STM32G431 development board (e.g., Nucleo-G431KB)
- 💻 [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)  
- 🔌 USB or external 3.3 V power supply
- 🔗 I²C Master (e.g. Raspberry Pi, Arduino, STM32)

---

## ⚙️ How to Build

```bash
# Clone the repository
git clone https://github.com/StyriaElectronics/STM32G4_I2C_Slave.git

# Open with STM32CubeIDE
STM32CubeIDE → File → Open Projects from File System...
```

---

## 🔄 I²C Interface

| Parameter         | Value            |
|------------------|------------------|
| Slave Address     | `0x42` (default) |
| Response Payload  | 4 bytes (2× 16-bit ADC values) |
| Sampling Rate     | 1 kHz per channel |
| Byte Order        | MSB first        |

---

## 📜 License

This project is licensed under the  
[**Creative Commons Zero v1.0 (CC0-1.0)**](https://creativecommons.org/publicdomain/zero/1.0/).  
> No restrictions – use, modify, and distribute freely!

---

## 🙌 Contributing

Feel free to fork the repository, open issues, or submit pull requests.  
PRs improving structure, timing accuracy, or DMA handling are welcome!

---

## ⭐️ If You Like It...

...consider giving this repo a ⭐️ to support the project!

---

Made with ❤️ by **Styria Electronics**  
🌐 [styria-electronics.at](https://styria-electronics.at)
