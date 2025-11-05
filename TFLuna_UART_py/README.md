# TF-Luna UART Python Utility

This Python program provides a utility for communicating with the TF-Luna LiDAR sensor over UART. It supports configuring the sensor, reading measurement data, and sending commands to adjust settings such as output frequency and data format.

## Features
- **Continuous Data Reading**: Reads and parses 9-byte data frames from the TF-Luna sensor.
- **Configurable Units**: Supports both `cm` and `mm` output formats.
- **Set Output Frequency**: Adjust the sensor's output frequency using the `--hz` option.
- **Command Support**: Send various commands to the sensor, such as setting the baud rate, enabling/disabling output, and restoring defaults.
- **Human-Readable Output**: Displays parsed distance, strength, and temperature values.

## Requirements
- Python 3.6+
- `pyserial` library

Install dependencies:
```bash
pip install -r requirements.txt
```

## Usage
Run the program with the following options:

### Basic Example
```bash
python3 tfluna_serial.py --port /dev/ttyUSB0 --units mm
```
- Reads data from the TF-Luna sensor on `/dev/ttyUSB0`.
- Configures the sensor to output distances in millimeters.

### Set Output Frequency
```bash
python3 tfluna_serial.py --port /dev/ttyUSB0 --hz 100
```
- Reads data from the TF-Luna sensor on `/dev/ttyUSB0`.
- Sets the sensor's output frequency to 100 Hz.

### Send Custom Command
```bash
python3 tfluna_serial.py --port /dev/ttyUSB0 --cmd set-freq 50
```
- Sends a command to set the output frequency to 50 Hz.

## Command-Line Options
- `--port`: Serial port to which the TF-Luna sensor is connected (e.g., `/dev/ttyUSB0`).
- `--baud`: Baud rate for the serial connection (default: 115200).
- `--units`: Output units (`cm` or `mm`, default: `cm`).
- `--hz`: Set the sensor's output frequency (0 for trigger mode).
- `--cmd`: Send a specific command to the sensor (e.g., `set-freq`, `set-format`).
- `--raw`: Print raw frame data in hexadecimal format.

## Notes
- Ensure the TF-Luna sensor is properly connected to the serial port.
- Use a logic level shifter if interfacing with a 3.3V device.
- Refer to the TF-Luna datasheet for detailed command descriptions.

## License
This project is licensed under the MIT License. See the LICENSE file for details.