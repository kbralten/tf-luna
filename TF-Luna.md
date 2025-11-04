**TF-Luna LiDAR Sensor: I2C Development Guide**

This guide provides the essential information for developing software to interface with the Benewake TF-Luna LiDAR module using the I2C (IIC) communication protocol.

**1\. Sensor Overview**

The TF-Luna is a single-point ranging LiDAR module that operates on the Time of Flight (TOF) principle. It emits a modulated near-infrared wave, measures the phase difference of the reflected wave, and calculates the distance to an object.

This guide focuses _only_ on the I2C protocol.

**2\. Enabling I2C Mode**

To use the I2C protocol, you must configure the sensor's hardware:

- **Pin 5 (Configuration Input):** Must be connected to **GND (Ground)**.
- **Pin 2 (RXD/SDA):** Functions as the I2C **SDA (Data)** line.
- **Pin 3 (TXD/SCL):** Functions as the I2C **SCL (Clock)** line.
- **Pin 6 (Multiplexing output):** Functions as a **Data Availability** signal.

The default I2C slave address is **0x10**. The sensor supports a clock speed of up to 400kps.

**3\. I2C Message Format**

The sensor acts as an I2C slave device. Communication follows standard I2C timing for writing to and reading from registers.

**Write Register Timing**

To write data (e.g., to a configuration register):

- START
- Slave Address + W (Write bit)
- ACK (from sensor)
- Register Address (to write to)
- ACK (from sensor)
- Data Byte 1
- ACK (from sensor)
- ...
- Data Byte N
- ACK (from sensor)
- STOP

**Read Register Timing**

To read data (e.g., the measured distance):

- **Set the register address to read from:**
  - START
  - Slave Address + W (Write bit)
  - ACK (from sensor)
  - Register Address (to read from, e.g., 0x00 for distance)
  - ACK (from sensor)
  - STOP (Note: The manual states this STOP can be omitted)
- **Read the data from the previously set address:**
  - START
  - Slave Address + R (Read bit)
  - ACK (from sensor)
  - Data Byte 1 (from sensor)
  - ACK (from host)
  - ...
  - Data Byte N (from sensor)
  - NACK (from host, to signal end of read)
  - STOP

**4\. Synchronizing Data Reads (Important!)**

The sensor updates its data registers (like distance) internally. Reading a register _while_ it is being updated can result in corrupt data.

To prevent this, you **must** use Pin 6 (Data Availability).

- The sensor updates its internal data registers (Distance, AMP, etc.).
- After the update is complete, the sensor sets **Pin 6 to a HIGH level**.
- Your host controller should detect this HIGH signal. This is your cue that the data is stable and ready to be read.
- The host should now perform the I2C read operation (as described in "Read Register Timing").
- As soon as the host performs _any_ register read, the sensor will automatically set **Pin 6 back to a LOW level**.
- The host waits for the next HIGH signal on Pin 6 before reading again.

**Failure to follow this synchronization method may lead to unreliable readings.**

**5\. Key I2C Registers (Commands)**

You control the sensor by reading from and writing to its registers.

**Data Registers (Read-Only)**

These registers are updated by the sensor. Read them _only_ after Pin 6 goes HIGH.

| **Address** | **Name** | **Description** |
| --- | --- | --- |
| 0x00 | DIST_LOW | Low byte of the distance value. |
| 0x01 | DIST_HIGH | High byte of the distance value. \*\*Distance (cm) = (DIST_HIGH << 8) |
| 0x02 | AMP_LOW | Low byte of the signal strength. |
| 0x03 | AMP_HIGH | High byte of the signal strength. \*\*Strength = (AMP_HIGH << 8) |
| 0x04 | TEMP_LOW | Low byte of the chip temperature. |
| 0x05 | TEMP_HIGH | High byte of the chip temperature. \*\*Temperature (°C) = ((TEMP_HIGH << 8) |
| 0x06 | TICK_LOW | Low byte of the system timestamp (in ms). |
| 0x07 | TICK_HIGH | High byte of the system timestamp. |

**Configuration & Control Registers (Read/Write)**

Writing to these registers changes the sensor's configuration.

| **Address** | **Name** | **R/W** | **Default** | **Description** |
| --- | --- | --- | --- | --- |
| 0x20 | SAVE | W   | \-  | **Save Settings.** Write 0x01 to this register to save all current register values to non-volatile memory. Otherwise, changes will be lost on reboot. |
| 0x21 | SHUTDOWN/REBOOT | W   | \-  | Write 0x02 to this register to reboot the sensor. |
| 0x22 | SLAVE_ADDR | R/W | 0x10 | Read/Write the I2C slave address. Range: 0x08 to 0x77. **Remember to save settings (0x20) after changing.** |
| 0x23 | MODE | R/W | 0x00 | 0x00: Continuous ranging mode.<br><br>0x01: Trigger mode. |
| 0x24 | TRIG_ONE_SHOT | W   | \-  | **Trigger Measurement.** When in Trigger mode (0x23=0x01), write 0x01 to this register to trigger a single measurement. |
| 0x25 | ENABLE | R/W | 0x00 | 0x00: Turn on LIDAR.<br><br>0x01: Turn off LIDAR (standby). |
| 0x26 | FPS_LOW | R/W | 0x64 | Low byte of the frame rate (Hz). Default is 0x0064 = 100 Hz. |
| 0x27 | FPS_HIGH | R/W | 0x00 | High byte of the frame rate (Hz). \*\*Frame Rate = (FPS_HIGH << 8) |
| 0x29 | RESTORE_DEFAULTS | W   | \-  | **Restore Factory Defaults.** Write 0x01 to this register to restore factory settings. |
| 0x2A | AMP_THR_LOW | R/W | 0x64 | Low byte of Amp threshold (default 100). |
| 0x2B | AMP_THR_HIGH | R/W | 0x00 | High byte of Amp threshold. |
| 0x2E | MIN_DIST_LOW | R/W | 0x00 | Low byte of minimum distance output (cm). |
| 0x2F | MIN_DIST_HIGH | R/W | 0x00 | High byte of minimum distance output (cm). |
| 0x30 | MAX_DIST_LOW | R/W | 0x20 | Low byte of maximum distance output (cm). Default 0x0320 = 800 cm. |
| 0x31 | MAX_DIST_HIGH | R/W | 0x03 | High byte of maximum distance output (cm). |

**6\. Saving Settings (Critical!)**

When you write to a configuration register (e.g., 0x22 to change the slave address), the change takes effect immediately but **will be lost when the sensor loses power.**

To make your changes permanent, you **must** write the value 0x01 to the SAVE register (0x20) after making your changes. This will save the current settings to the sensor's non-volatile memory.