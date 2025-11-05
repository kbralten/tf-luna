**TF-Luna LiDAR Sensor: Serial (UART) Development Guide**

This guide provides the essential information for developing software to interface with the Benewake TF-Luna LiDAR module using the Serial (UART) communication protocol.

**1\. Sensor Overview**

The TF-Luna is a single-point ranging LiDAR module that operates on the Time of Flight (TOF) principle. It emits a modulated near-infrared wave, measures the phase difference of the reflected wave, and calculates the distance to an object.

This guide focuses _only_ on the Serial (UART) protocol.

**2\. Enabling Serial (UART) Mode**

To use the Serial/UART protocol, you must configure the sensor's hardware:

- **Pin 5 (Configuration Input):** Must be **disconnected (floating)** or connected to **3.3V**.
- **Pin 2 (RXD/SDA):** Functions as the UART **RXD (Receiving)** line (Host TX to Sensor RX).
- **Pin 3 (TXD/SCL):** Functions as the UART **TXD (Transmitting)** line (Host RX from Sensor TX).

**Serial Port Settings**

- **Default Baud Rate:** 115200 bps
- **Data Bits:** 8
- **Stop Bits:** 1
- **Parity:** None
- **Signal Level:** LVTTL (3.3V)

The baud rate is configurable via a serial command (see Section 5).

**3\. Communication Protocol**

Communication consists of two types of messages:

- **Data Frames:** Data actively sent _from_ the sensor (e.g., distance, amp). The format of this data is specified by a configuration setting.
- **Command Frames:** Messages sent _to_ the sensor to change settings or request information.

**Command Frame Format (Host to Sensor)**

To send a command to the sensor, you must use the following byte format:

| **Byte** | **0** | **1** | **2** | **3 to (Len-2)** | **(Len-1)** |
| --- | --- | --- | --- | --- | --- |
| **Description** | Head | Len | ID  | Payload | Checksum |
| **Value** | 0x5A | Length | Command ID | Command Data | Checksum |

- **Head:** Always 0x5A.
- **Len:** The total length of the frame, from Head to Checksum (inclusive).
- **ID:** The specific ID for the command you want to execute (e.g., 0x03 for setting frequency).
- **Payload:** The data payload for the command, if any. This field is optional and its length depends on the command.
- **Checksum:** The lower 8 bits of the sum of all bytes from Head to Payload (i.e., bytes 0 to Len-2).
  - **Note:** By default, the sensor _does not_ check this checksum. The feature must be explicitly enabled using command 0x08.

**Response Frame Format (Sensor to Host)**

When you send a command, the sensor will often reply with a response frame in the same format.

**4\. Data Output Formats (Sensor to Host)**

The sensor can be configured to output measurement data in several different formats. The default is 9-byte/cm. This is set via the "Output Format" command (0x05).

**Default Format: 9-byte/cm (Format 0x01)**

This is the default output format. The sensor continuously sends 9-byte packets.

| **Byte** | **0** | **1** | **2** | **3** | **4** | **5** | **6** | **7** | **8** |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **Value** | 0x59 | 0x59 | Dist_L | Dist_H | Amp_L | Amp_H | Temp_L | Temp_H | Checksum |

- **Header:** 0x59 0x59 (Note: This is different from the command header 0x5A).
- **Distance (cm):** (Dist_H << 8) | Dist_L
- **Signal Strength (Amp):** (Amp_H << 8) | Amp_L (Unreliable if < 100 or 65535)
- **Temperature (°C):** ((Temp_H << 8) | Temp_L) / 8 - 256
- **Checksum:** Lower 8 bits of the sum of bytes 0-7.

**Other Formats**

Other available formats include:

- **PIX (Format 0x02):** An ASCII string (e.g., "1.23\\r\\n").
- **9-byte/mm (Format 0x06):** Same as default, but distance is in millimeters.

**5\. Key Serial Commands (Host to Sensor)**

Below are some of the most common commands. All commands must be sent using the frame format from Section 3.

| **ID** | **Command Name** | **Payload (Hex)** | **Example Full Command (Hex)** | **Description** |
| --- | --- | --- | --- | --- |
| 0x03 | **Set Output Frequency** | \[Freq_L\] \[Freq_H\] | 5A 06 03 64 00 00 | Sets output freq. (e.g., 0x0064 = 100 Hz). Set to 0x0000 to enter Trigger Mode. |
| 0x04 | **Trigger Measurement** | (None) | 5A 04 04 00 | In Trigger Mode, sends this command to get a single measurement. |
| 0x05 | **Set Output Format** | \[Format_ID\] | 5A 05 05 01 00 | Sets the data output format. 0x01 = 9-byte/cm (default), 0x02 = PIX, 0x06 = 9-byte/mm. |
| 0x06 | **Set Baud Rate** | \[B0\] \[B1\] \[B2\] \[B3\] | 5A 08 06 00 C2 01 00 00 | Sets baud rate (e.g., 0x0001C200 = 115200). Little-endian. |
| 0x07 | **Enable/Disable Output** | \[Enable\] | 5A 05 07 00 00 | 0x01 = Enable output (default). 0x00 = Disable output. |
| 0x10 | **Restore Defaults** | (None) | 5A 04 10 00 | Restores all settings to factory defaults. **This is not saved** until you send command 0x11. |
| 0x11 | **Save Settings** | (None) | 5A 04 11 00 | **Critical:** Saves all current settings to non-volatile memory. |
| 0x35 | **Set Power Saving** | \[Sample_Rate\] | 5A 06 35 0A 00 00 | Enable power saving (ECO) mode. 0x00 = Disable. 0x0A = 10Hz, etc. |
| 0x3A | **Set Distance Limit** | \[Min_L\] \[Min_H\] \[Max_L\] \[Max_H\] \[Silence\] | 5A 09 3A 14 00 F4 01 00 00 | Sets min/max output range. e.g., 20cm (0x0014) to 500cm (0x01F4). |

**6\. Saving Settings (Critical!)**

When you send a command to change a setting (like baud rate or output format), the change takes effect immediately but **will be lost when the sensor loses power.**

To make your changes permanent, you must send the Save Settings command (0x11) after making your changes.

Example: 5A 04 11 00