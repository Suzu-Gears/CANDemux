English | [日本語](./README.ja.md)

# CANDemux - Virtual CAN bus demultiplexer for Arduino

An Arduino library that demultiplexes a physical CAN bus into multiple virtual CAN interfaces by filtering message IDs. VirtualCAN implements arduino::HardwareCAN and can be passed directly to code or APIs that accept arduino::HardwareCAN. Each virtual interface supports ID lists or ranges and configurable receive-queue behavior.

## Compatible Hardware

`CANDemux` works with any CAN controller or board that provides the `arduino::HardwareCAN` interface. Examples:

*   **Arduino (Uno R4 WiFi / Uno R4 Minima  / Nano R4):**
    *   Library: `<Arduino_CAN.h>` (Built-in)
    *   Tutorial: https://docs.arduino.cc/tutorials/uno-r4-minima/can

*   **ESP32:**
    *   Library: `<ESP32_TWAI.h>` (Arduino IDE Library Manager - search: "`ESP32_TWAI`")
    *   GitHub: [eyr1n/ESP32_TWAI](https://github.com/eyr1n/ESP32_TWAI)

*   **Raspberry Pi Pico (RP2040/RP2350):**
    *   Library: `<RP2040PIO_CAN.h>` (Arduino IDE Library Manager - search: "`RP2040PIO_CAN`")
    *   GitHub: [eyr1n/RP2040PIO_CAN](https://github.com/eyr1n/RP2040PIO_CAN)

## Installation

<!--
### Arduino IDE Library Manager

1.  Open the Arduino IDE.
2.  Go to `Sketch > Include Library > Manage Libraries...`.
3.  Search for "CANDemux" and install the latest version.
-->

### Manual Installation

#### Using Arduino IDE's "Add .ZIP Library"

1.  Download the latest release from the [GitHub repository](https://github.com/Suzu-Gears/CANDemux/releases/latest) as a `.zip` file.
2.  In the Arduino IDE, go to `Sketch > Include Library > Add .ZIP Library...`.
3.  Navigate to the downloaded `.zip` file and select it.
4.  Restart the Arduino IDE.

#### Direct Placement

1.  Download the latest release from the [GitHub repository](https://github.com/Suzu-Gears/CANDemux/releases/latest).
2.  Unzip the downloaded file and place the `CANDemux` folder into your Arduino libraries directory (e.g., `~/Documents/Arduino/libraries/`).
3.  Restart the Arduino IDE.