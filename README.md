# TF-Luna Arduino Project

This project provides Arduino sketches for interfacing with the TF-Luna LiDAR sensor using the I2C communication protocol. The TF-Luna sensor is a compact and cost-effective LiDAR module capable of measuring distances with high accuracy.

## Features
- **I2C Communication**: Read data from the TF-Luna sensor using the I2C protocol.

## File Structure
- `TFLuna_I2C.ino`: Arduino sketch for I2C communication with the TF-Luna sensor.

## Requirements
- **Hardware**:
  - Arduino Uno (or compatible board)
  - TF-Luna LiDAR sensor
  - Jumper wires for connections
- **Software**:
  - [Arduino IDE](https://www.arduino.cc/en/software) or [arduino-cli](https://arduino.github.io/arduino-cli/)

## Setup Instructions
1. **Hardware Connections**:
  - **SCL / SDA**: Connect the TF-Luna's `SCL` and `SDA` pins to the Arduino's I2C pins. On an Arduino Uno these are `A5` (`SCL`) and `A4` (`SDA`). On Uno R3 and other boards there are dedicated `SCL`/`SDA` pins next to `AREF` — prefer the dedicated pins when available.
  - **VCC / GND**: Connect `VCC` to `5V` (or `3.3V` if your TF-Luna module requires it) and `GND` to the Arduino's `GND`.
  - **MODE / SEL**: Connect the TF-Luna's `MODE` (or `SEL`) pin to the level required to enable I2C on your specific module. Different TF-Luna module versions use different MODE/SEL polarity — consult the TF-Luna datasheet or your module's documentation for whether `MODE` must be pulled `LOW` (GND) or `HIGH` (VCC) for I2C. Use a jumper or solder bridge to set the pin accordingly.
  - **DATA READY (DR / DOUT / INT)**: If your module exposes a data-ready or interrupt pin (often labeled `DR`, `DOUT`, or `INT`), connect it to an Arduino digital input (for example `D2`) so your sketch can detect when a new measurement is available. `D2` is interrupt-capable on the Uno and is a common choice.

2. **Software Setup**:
   - Install the Arduino IDE or arduino-cli.
   - Open the `TFLuna_I2C.ino` sketch in the Arduino IDE.
   - Select the correct board and port from the Tools menu.
   - Compile and upload the sketch to the Arduino.

3. **Testing**:
   - Open the Serial Monitor in the Arduino IDE.
   - Set the baud rate to `115200`.
   - Observe the distance measurements output.

## Usage
- **I2C Sketch**:
  - Reads distance data from the TF-Luna sensor using the I2C protocol.

## Notes
- Ensure the TF-Luna sensor is powered correctly (5V).
- Use a logic level shifter if interfacing with a 3.3V Arduino board.
- Refer to the TF-Luna datasheet for detailed specifications and command sets.

## License
This project is licensed under the MIT License. See the LICENSE file for details.