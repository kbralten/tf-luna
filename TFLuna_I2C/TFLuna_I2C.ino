/*
 * TF-Luna LiDAR I2C Reader
 *
 * This sketch reads distance measurements from a Benewake TF-Luna LiDAR sensor
 * using the I2C protocol with proper data synchronization and periodic
 * diagnostics when the Data-Ready pin isn't asserted.
 *
 * Hardware Connections:
 * - TF-Luna Pin 2 (SDA) -> Arduino SDA (A4 on Uno/Nano)
 * - TF-Luna Pin 3 (SCL) -> Arduino SCL (A5 on Uno/Nano)
 * - TF-Luna Pin 5 -> GND (to enable I2C mode)
 * - TF-Luna Pin 6 -> Arduino Digital Pin 2 (Data Ready interrupt)
 * - TF-Luna VCC -> 5V
 * - TF-Luna GND -> GND
 *
 * Default I2C Address: 0x10
 */

#include <Wire.h>

// TF-Luna I2C Configuration
#define TFLUNA_I2C_ADDR    0x10  // Default I2C slave address
#define DATA_READY_PIN     2     // Pin 6 from TF-Luna (Data Availability signal)

// Register Addresses
#define REG_DIST_LOW       0x00  // Distance low byte
#define REG_DIST_HIGH      0x01  // Distance high byte
#define REG_AMP_LOW        0x02  // Signal strength low byte
#define REG_AMP_HIGH       0x03  // Signal strength high byte
#define REG_TEMP_LOW       0x04  // Temperature low byte
#define REG_TEMP_HIGH      0x05  // Temperature high byte

// Diagnostics registers
#define REG_MODE           0x23
#define REG_FPS_LOW        0x26
#define REG_FPS_HIGH       0x27

// Global variables
volatile bool dataReady = false;  // Flag set by interrupt when data is ready
const unsigned long DIAG_INTERVAL_MS = 2000; // print diagnostics every 2s when no data-ready
unsigned long lastDiagMillis = 0;

// Function prototypes
void dataReadyISR();
bool readRegister(uint8_t regAddr, uint8_t &outValue);
bool readMultipleRegisters(uint8_t startAddr, uint8_t* buffer, uint8_t length);
uint16_t readDistance();
uint16_t readSignalStrength();
int16_t readTemperature();
void runDiagnostics();
void writeRegister(uint8_t regAddr, uint8_t value);

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // wait for Serial (on native USB boards)

  Serial.println("TF-Luna LiDAR I2C Reader");
  Serial.println("========================");

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);  // Set I2C clock to 400kHz

  // Set to 10hz frame rate
  Serial.println("Setting TF-Luna to 10Hz frame rate (write 0x0A to 0x26)");
  writeRegister(REG_FPS_LOW, 10);
  delay(50);

  // Configure Data Ready pin (Pin 6 from TF-Luna)
  pinMode(DATA_READY_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(DATA_READY_PIN), dataReadyISR, RISING);

  Serial.println("Initialization complete.");
  Serial.println("Waiting for sensor data...\n");

  delay(100);

  runDiagnostics();
}

void loop() {
  // If data is ready (Pin 6 went HIGH), read main measurement set
  if (dataReady) {
    dataReady = false; // reset flag

    uint16_t distance = readDistance();
    uint16_t strength = readSignalStrength();
    int16_t temperature_raw = readTemperature();

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm | Strength: ");
    Serial.print(strength);
    Serial.print(" | Temp: ");
    // temperature_raw is raw integer; per doc caller divides by 100.0
    Serial.print(temperature_raw / 100.0, 2);
    Serial.println(" °C");
  }
}

// ISR for Data Ready
void dataReadyISR() {
  dataReady = true;
}

// Read a single register from the sensor. Returns true on success and stores value in outValue.
bool readRegister(uint8_t regAddr, uint8_t &outValue) {
  Wire.beginTransmission(TFLUNA_I2C_ADDR);
  Wire.write(regAddr);
  uint8_t err = Wire.endTransmission(false); // send restart
  if (err != 0) return false;

  uint8_t req = Wire.requestFrom(TFLUNA_I2C_ADDR, (uint8_t)1);
  if (req == 0) return false;

  if (Wire.available()) {
    outValue = Wire.read();
    return true;
  }
  return false;
}

// Read multiple bytes starting from a register address
bool readMultipleRegisters(uint8_t startAddr, uint8_t* buffer, uint8_t length) {
  Wire.beginTransmission(TFLUNA_I2C_ADDR);
  Wire.write(startAddr);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) return false;

  uint8_t req = Wire.requestFrom(TFLUNA_I2C_ADDR, length);
  if (req == 0) return false;

  uint8_t i = 0;
  while (Wire.available() && i < length) {
    buffer[i++] = Wire.read();
  }
  return (i == length);
}

uint16_t readDistance() {
  uint8_t low, high;
  if (!readRegister(REG_DIST_LOW, low) || !readRegister(REG_DIST_HIGH, high)) {
    Serial.println("Failed to read distance registers");
    return 0;
  }
  return ((uint16_t)high << 8) | low;
}

uint16_t readSignalStrength() {
  uint8_t low, high;
  if (!readRegister(REG_AMP_LOW, low) || !readRegister(REG_AMP_HIGH, high)) {
    Serial.println("Failed to read signal strength registers");
    return 0;
  }
  return ((uint16_t)high << 8) | low;
}

int16_t readTemperature() {
  uint8_t low, high;
  if (!readRegister(REG_TEMP_LOW, low) || !readRegister(REG_TEMP_HIGH, high)) {
    Serial.println("Failed to read temperature registers");
    return 0;
  }
  return ((int16_t)high << 8) | low; // raw value; divide by 100.0 for degC
}

// Diagnostics: read MODE (0x23), FPS_LOW (0x26), FPS_HIGH (0x27)
void runDiagnostics() {
  uint8_t modeVal, fpsLow, fpsHigh;
  Serial.println("--- TF-Luna Diagnostics ---");

  if (!readRegister(REG_MODE, modeVal)) {
    Serial.println("MODE (0x23): <no response>");
  } else {
    Serial.print("MODE (0x23): 0x");
    if (modeVal < 16) Serial.print('0');
    Serial.print(modeVal, HEX);
    Serial.print(" (");
    if (modeVal == 0x00) Serial.print("Continuous");
    else if (modeVal == 0x01) Serial.print("Trigger");
    else Serial.print("Unknown");
    Serial.println(")");
  }

  bool gotFpsLow = readRegister(REG_FPS_LOW, fpsLow);
  bool gotFpsHigh = readRegister(REG_FPS_HIGH, fpsHigh);
  if (!gotFpsLow && !gotFpsHigh) {
    Serial.println("FPS (0x26/0x27): <no response>");
  } else {
    uint16_t fps = 0;
    if (gotFpsLow && gotFpsHigh) fps = ((uint16_t)fpsHigh << 8) | fpsLow;
    else if (gotFpsLow) fps = fpsLow;
    else if (gotFpsHigh) fps = ((uint16_t)fpsHigh << 8);

    Serial.print("FPS_LOW (0x26): 0x");
    if (gotFpsLow) {
      if (fpsLow < 16) Serial.print('0');
      Serial.print(fpsLow, HEX);
    } else Serial.print("--");
    Serial.println();

    Serial.print("FPS_HIGH (0x27): 0x");
    if (gotFpsHigh) {
      if (fpsHigh < 16) Serial.print('0');
      Serial.print(fpsHigh, HEX);
    } else Serial.print("--");
    Serial.println();

    if (gotFpsLow || gotFpsHigh) {
      Serial.print("Frame Rate (Hz): ");
      Serial.println(fps);
    }
  }

  Serial.println("---------------------------\\n");
}

// Write to a register (for configuration changes)
void writeRegister(uint8_t regAddr, uint8_t value) {
  Wire.beginTransmission(TFLUNA_I2C_ADDR);
  Wire.write(regAddr);
  Wire.write(value);
  Wire.endTransmission();
}

