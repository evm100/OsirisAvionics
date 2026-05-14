/***************************************************************************
  ESP32 + Arduino IDE
  Combined I2C diagnostic + LSM6DSOX + BMP388 sketch

  Pins:
    SDA = 7
    SCL = 8

  Features:
    - I2C bus scan at startup
    - LSM6DSOX WHO_AM_I check
    - BMP388 init and reading
    - LSM6DSOX accel + gyro raw data reading
    - Prints both sensors continuously

  Libraries needed:
    - Adafruit BMP3XX Library
    - Adafruit Unified Sensor
    - Adafruit BusIO
***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

// =========================
// I2C configuration
// =========================
#define I2C_SDA 7
#define I2C_SCL 8
#define I2C_FREQ 100000  // 100kHz for reliability during diagnostics

// =========================
// BMP388 configuration
// =========================
#define BMP388_ADDR_1 0x76
#define BMP388_ADDR_2 0x77
#define BMP388_CHIP_ID_REG 0x00
#define BMP388_EXPECTED_ID 0x50
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;
uint8_t bmpAddress = 0;

// =========================
// LSM6DSOX configuration
// =========================
#define LSM6DSOX_ADDR_1 0x6A
#define LSM6DSOX_ADDR_2 0x6B
#define LSM6DSOX_WHO_AM_I_REG 0x0F
#define LSM6DSOX_EXPECTED_ID 0x6C

#define LSM6DSOX_CTRL1_XL  0x10
#define LSM6DSOX_CTRL2_G   0x11
#define LSM6DSOX_OUTX_L_G  0x22
#define LSM6DSOX_OUTX_L_A  0x28

uint8_t lsmAddress = 0;

// =========================
// Low-level I2C helpers
// =========================
bool writeRegister(uint8_t devAddr, uint8_t regAddr, uint8_t value) {
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool readRegisters(uint8_t devAddr, uint8_t regAddr, uint8_t *buffer, size_t len) {
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  if (Wire.endTransmission(false) != 0) {  // repeated start
    return false;
  }

  size_t received = Wire.requestFrom((int)devAddr, (int)len);
  if (received != len) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool readRegister(uint8_t devAddr, uint8_t regAddr, uint8_t &value) {
  return readRegisters(devAddr, regAddr, &value, 1);
}

// =========================
// Diagnostic helpers
// =========================
void runI2CScanner() {
  Serial.println();
  Serial.println("=== Starting I2C Bus Scan ===");

  int devicesFound = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at address: 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      devicesFound++;
    }
  }

  if (devicesFound == 0) {
    Serial.println("NO DEVICES FOUND. Check wiring/pullups/power.");
  } else {
    Serial.print("Scan complete. Found ");
    Serial.print(devicesFound);
    Serial.println(" device(s).");
  }
  Serial.println("=============================");
  Serial.println();
}

void dumpRegisters(uint8_t addr, const char *name) {
  Serial.println();
  Serial.print("[Hex Dump for ");
  Serial.print(name);
  Serial.print(" (0x");
  if (addr < 16) Serial.print("0");
  Serial.print(addr, HEX);
  Serial.println(")]");

  Serial.println("Reg:  00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F");
  Serial.print("Val:  ");

  for (uint8_t reg = 0; reg < 16; reg++) {
    uint8_t val = 0;
    if (readRegister(addr, reg, val)) {
      if (val < 16) Serial.print("0");
      Serial.print(val, HEX);
      Serial.print(" ");
    } else {
      Serial.print("XX ");
    }
  }
  Serial.println();
}

void checkSensorID(uint8_t addr, uint8_t regID, uint8_t expectedID, const char *name) {
  uint8_t id = 0;
  if (readRegister(addr, regID, id)) {
    if (id == expectedID) {
      Serial.print("[OK] ");
      Serial.print(name);
      Serial.print(" verified at 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      Serial.print(", ID=0x");
      if (id < 16) Serial.print("0");
      Serial.println(id, HEX);
    } else {
      Serial.print("[FAIL] ");
      Serial.print(name);
      Serial.print(" ID mismatch at 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      Serial.print(". Read 0x");
      if (id < 16) Serial.print("0");
      Serial.print(id, HEX);
      Serial.print(", expected 0x");
      if (expectedID < 16) Serial.print("0");
      Serial.println(expectedID, HEX);
    }
    dumpRegisters(addr, name);
  } else {
    Serial.print("[FAIL] Could not communicate with ");
    Serial.print(name);
    Serial.print(" at 0x");
    if (addr < 16) Serial.print("0");
    Serial.println(addr, HEX);
  }
}

// =========================
// LSM6DSOX helpers
// =========================
bool findLSM6DSOX() {
  uint8_t who = 0;

  if (readRegister(LSM6DSOX_ADDR_1, LSM6DSOX_WHO_AM_I_REG, who) && who == LSM6DSOX_EXPECTED_ID) {
    lsmAddress = LSM6DSOX_ADDR_1;
    return true;
  }

  if (readRegister(LSM6DSOX_ADDR_2, LSM6DSOX_WHO_AM_I_REG, who) && who == LSM6DSOX_EXPECTED_ID) {
    lsmAddress = LSM6DSOX_ADDR_2;
    return true;
  }

  return false;
}

bool initLSM6DSOX() {
  if (!findLSM6DSOX()) {
    return false;
  }

  // Accel: 104 Hz, ±2g
  if (!writeRegister(lsmAddress, LSM6DSOX_CTRL1_XL, 0x40)) {
    return false;
  }

  // Gyro: 104 Hz, 250 dps
  if (!writeRegister(lsmAddress, LSM6DSOX_CTRL2_G, 0x40)) {
    return false;
  }

  return true;
}

bool readLSM6DSOX(int16_t &gx, int16_t &gy, int16_t &gz,
                  int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t gyroData[6];
  uint8_t accelData[6];

  if (!readRegisters(lsmAddress, LSM6DSOX_OUTX_L_G, gyroData, 6)) {
    return false;
  }

  if (!readRegisters(lsmAddress, LSM6DSOX_OUTX_L_A, accelData, 6)) {
    return false;
  }

  gx = (int16_t)((gyroData[1] << 8) | gyroData[0]);
  gy = (int16_t)((gyroData[3] << 8) | gyroData[2]);
  gz = (int16_t)((gyroData[5] << 8) | gyroData[4]);

  ax = (int16_t)((accelData[1] << 8) | accelData[0]);
  ay = (int16_t)((accelData[3] << 8) | accelData[2]);
  az = (int16_t)((accelData[5] << 8) | accelData[4]);

  return true;
}

// =========================
// BMP388 helpers
// =========================
bool initBMP388() {
  if (bmp.begin_I2C(BMP388_ADDR_1, &Wire)) {
    bmpAddress = BMP388_ADDR_1;
  } else if (bmp.begin_I2C(BMP388_ADDR_2, &Wire)) {
    bmpAddress = BMP388_ADDR_2;
  } else {
    return false;
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  return true;
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("========================================");
  Serial.println("ESP32 I2C Diagnostic + LSM6DSOX + BMP388");
  Serial.println("========================================");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);

  Serial.print("I2C initialized on SDA=");
  Serial.print(I2C_SDA);
  Serial.print(", SCL=");
  Serial.println(I2C_SCL);

  // Full scan
  runI2CScanner();

  // Explicit ID checks like your diagnostic code
  Serial.println("Checking LSM6DSOX...");
  checkSensorID(LSM6DSOX_ADDR_1, LSM6DSOX_WHO_AM_I_REG, LSM6DSOX_EXPECTED_ID, "LSM6DSOX (Primary)");
  checkSensorID(LSM6DSOX_ADDR_2, LSM6DSOX_WHO_AM_I_REG, LSM6DSOX_EXPECTED_ID, "LSM6DSOX (Alt)");

  Serial.println();
  Serial.println("Checking BMP388...");
  checkSensorID(BMP388_ADDR_1, BMP388_CHIP_ID_REG, BMP388_EXPECTED_ID, "BMP388 (Primary)");
  checkSensorID(BMP388_ADDR_2, BMP388_CHIP_ID_REG, BMP388_EXPECTED_ID, "BMP388 (Alt)");

  Serial.println();

  // Initialize LSM6DSOX
  if (initLSM6DSOX()) {
    Serial.print("LSM6DSOX initialized successfully at 0x");
    if (lsmAddress < 16) Serial.print("0");
    Serial.println(lsmAddress, HEX);
    Serial.println("Configured for 104 Hz, ±2g accel, 250 dps gyro");
  } else {
    Serial.println("Failed to initialize LSM6DSOX");
  }

  // Initialize BMP388
  if (initBMP388()) {
    Serial.print("BMP388 initialized successfully at 0x");
    if (bmpAddress < 16) Serial.print("0");
    Serial.println(bmpAddress, HEX);
  } else {
    Serial.println("Failed to initialize BMP388");
  }

  Serial.println();
  Serial.println("Starting continuous sensor readout...");
  Serial.println();
}

// =========================
// Loop
// =========================
void loop() {
  // ---- LSM6DSOX ----
  if (lsmAddress != 0) {
    int16_t gx, gy, gz, ax, ay, az;

    if (readLSM6DSOX(gx, gy, gz, ax, ay, az)) {
      // Approximate scaling:
      // accel ±2g => 0.061 mg/LSB
      // gyro 250 dps => 8.75 mdps/LSB
      int accel_mg_x = (int)(ax * 0.061f);
      int accel_mg_y = (int)(ay * 0.061f);
      int accel_mg_z = (int)(az * 0.061f);

      int gyro_mdps_x = (int)(gx * 8.75f);
      int gyro_mdps_y = (int)(gy * 8.75f);
      int gyro_mdps_z = (int)(gz * 8.75f);

      Serial.print("LSM6DSOX | ACCEL [mg] X=");
      Serial.print(accel_mg_x);
      Serial.print(" Y=");
      Serial.print(accel_mg_y);
      Serial.print(" Z=");
      Serial.print(accel_mg_z);

      Serial.print(" | GYRO [mdps] X=");
      Serial.print(gyro_mdps_x);
      Serial.print(" Y=");
      Serial.print(gyro_mdps_y);
      Serial.print(" Z=");
      Serial.println(gyro_mdps_z);
    } else {
      Serial.println("LSM6DSOX read failed");
    }
  }

  // ---- BMP388 ----
  if (bmpAddress != 0) {
    if (bmp.performReading()) {
      Serial.print("BMP388   | Temp = ");
      Serial.print(bmp.temperature);
      Serial.print(" C | Pressure = ");
      Serial.print(bmp.pressure / 100.0);
      Serial.print(" hPa | Altitude = ");
      Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
      Serial.println(" m");
    } else {
      Serial.println("BMP388 reading failed");
    }
  }

  Serial.println();
  delay(500);
}