/*
  ChangeBaudTo9600.ino
  ----------------------------------------------------------------
  ONE-TIME UTILITY: switches the SEN0676 sensor's Modbus baud rate
  from 115200 to 9600 and verifies the change.

  Why: SoftwareSerial on ESP8266 is bit-banged in software, and at
  115200 baud it is very sensitive to interrupt jitter from WiFi/MQTT,
  causing frequent CRC/timeout errors. Running the sensor at 9600
  gives much more margin and drastically reduces those errors.

  What this sketch does:
    1. Opens the serial port to the sensor at 115200 (its current baud).
    2. Writes register 0x03F6 (baud rate) with value 96
       (96 * 100 = 9600, per the datasheet: "actual baud rate / 100").
    3. Re-opens the serial port at 9600.
    4. Reads register 0x03F6 back to confirm the sensor now responds
       at 9600 and reports the expected value.

  Run this sketch ONCE, then go back to your main sketch and configure
  its serial port (and the DFRobot_SEN0676 library, if you use one) to
  use 9600 baud from now on -- the sensor will remember this setting.

  Wiring: direct TTL UART, no RS485.
    - ESP32:   Serial2, RX=17, TX=16
    - ESP8266: SoftwareSerial, RX=13, TX=12
  Cross TX/RX: sensor TX -> microcontroller RX, and vice versa.
*/

#include <Arduino.h>

#if defined(ESP32)
  #define SENSOR_RX_PIN  17
  #define SENSOR_TX_PIN  16
  #define SENSOR_SERIAL  Serial2

#elif defined(ESP8266)
  #include <SoftwareSerial.h>
  #define SENSOR_RX_PIN  13
  #define SENSOR_TX_PIN  12
  SoftwareSerial SENSOR_SERIAL(SENSOR_RX_PIN, SENSOR_TX_PIN); // RX, TX

#else
  #error "This sketch targets ESP8266 or ESP32"
#endif

#define SLAVE_ADDR         0x01   // Sensor's current Modbus address
#define BAUD_RATE_REG      0x03F6 // Register that stores the baud rate (per datasheet)
#define OLD_BAUD           115200
#define NEW_BAUD           9600
#define NEW_BAUD_REG_VALUE 96     // 9600 / 100, per datasheet format
#define RESPONSE_TIMEOUT   800     // ms
#define RETRIES            3

uint8_t rxBuffer[16];

// ---------- MODBUS CRC16 (polynomial 0xA001) ----------
uint16_t modbusCRC16(const uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

bool checkCRC(uint8_t *data, uint8_t len) {
  if (len < 3) return false;
  uint16_t calcCrc = modbusCRC16(data, len - 2);
  uint16_t recvCrc = data[len - 2] | (data[len - 1] << 8);
  return calcCrc == recvCrc;
}

void sendFrame(uint8_t *frame, uint8_t len) {
  uint16_t crc = modbusCRC16(frame, len);
  frame[len]     = crc & 0xFF;
  frame[len + 1] = (crc >> 8) & 0xFF;

  delay(2); // small bus-settle delay
  while (SENSOR_SERIAL.available()) SENSOR_SERIAL.read();

  SENSOR_SERIAL.write(frame, len + 2);
  SENSOR_SERIAL.flush();
}

uint8_t readResponse(uint16_t timeoutMs) {
  uint8_t idx = 0;
  unsigned long start = millis();
  unsigned long lastByteTime = millis();

  while (millis() - start < timeoutMs) {
    if (SENSOR_SERIAL.available()) {
      rxBuffer[idx++] = SENSOR_SERIAL.read();
      lastByteTime = millis();
      if (idx >= sizeof(rxBuffer)) break;
    } else if (idx > 0 && (millis() - lastByteTime > 20)) {
      break;
    }
  }
  return idx;
}

bool writeRegister(uint16_t regAddr, uint16_t value, uint8_t retries) {
  uint8_t frame[8];
  frame[0] = SLAVE_ADDR;
  frame[1] = 0x06;
  frame[2] = (regAddr >> 8) & 0xFF;
  frame[3] = regAddr & 0xFF;
  frame[4] = (value >> 8) & 0xFF;
  frame[5] = value & 0xFF;

  for (uint8_t attempt = 0; attempt <= retries; attempt++) {
    sendFrame(frame, 6);
    uint8_t len = readResponse(RESPONSE_TIMEOUT);
    if (len >= 8 && rxBuffer[0] == SLAVE_ADDR && rxBuffer[1] == 0x06 && checkCRC(rxBuffer, len)) {
      return true;
    }
    delay(100);
  }
  return false;
}

bool readRegister(uint16_t regAddr, uint16_t &value, uint8_t retries) {
  uint8_t frame[8];
  frame[0] = SLAVE_ADDR;
  frame[1] = 0x03;
  frame[2] = (regAddr >> 8) & 0xFF;
  frame[3] = regAddr & 0xFF;
  frame[4] = 0x00;
  frame[5] = 0x01;

  for (uint8_t attempt = 0; attempt <= retries; attempt++) {
    sendFrame(frame, 6);
    uint8_t len = readResponse(RESPONSE_TIMEOUT);
    if (len >= 7 && rxBuffer[0] == SLAVE_ADDR && rxBuffer[1] == 0x03 && checkCRC(rxBuffer, len)) {
      value = (rxBuffer[3] << 8) | rxBuffer[4];
      return true;
    }
    delay(100);
  }
  return false;
}

void openSerial(uint32_t baud) {
#if defined(ESP32)
  SENSOR_SERIAL.begin(baud, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
#elif defined(ESP8266)
  SENSOR_SERIAL.begin(baud);
#endif
  delay(50);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println(F("==================================================="));
  Serial.println(F(" SEN0676 - One-time baud rate change to 9600"));
  Serial.println(F("==================================================="));

  // Step 1: open at the sensor's current baud rate (115200)
  Serial.print(F("Opening sensor port at "));
  Serial.print(OLD_BAUD);
  Serial.println(F(" baud..."));
  openSerial(OLD_BAUD);

  // Step 2: write the new baud rate value into register 0x03F6
  Serial.println(F("Writing new baud rate (9600) to register 0x03F6..."));
  bool writeOk = writeRegister(BAUD_RATE_REG, NEW_BAUD_REG_VALUE, RETRIES);

  if (writeOk) {
    Serial.println(F("-> Sensor confirmed the write at 115200 baud."));
  } else {
    Serial.println(F("-> No confirmation received at 115200 (this can be normal:"));
    Serial.println(F("   some units switch baud rate before echoing back)."));
  }

  // Step 3: re-open the port at the new baud rate and verify
  delay(300);
  Serial.print(F("Re-opening sensor port at "));
  Serial.print(NEW_BAUD);
  Serial.println(F(" baud to verify..."));
  openSerial(NEW_BAUD);

  uint16_t currentBaudReg = 0;
  bool verifyOk = false;

  // A few attempts, since the sensor may need a moment after switching
  for (uint8_t i = 0; i < 5 && !verifyOk; i++) {
    if (readRegister(BAUD_RATE_REG, currentBaudReg, 1)) {
      verifyOk = true;
    } else {
      delay(300);
    }
  }

  Serial.println(F("---------------------------------------------------"));
  if (verifyOk) {
    Serial.print(F("SUCCESS: sensor now responds at 9600 baud. "));
    Serial.print(F("Register value read: "));
    Serial.print(currentBaudReg);
    Serial.print(F(" (means "));
    Serial.print((uint32_t)currentBaudReg * 100);
    Serial.println(F(" bps)."));
    Serial.println(F(""));
    Serial.println(F("You can now update your main sketch to use 9600 baud"));
    Serial.println(F("for this sensor. The setting is stored in the sensor"));
    Serial.println(F("itself, so this only needs to be done once."));
  } else {
    Serial.println(F("FAILED: could not confirm the sensor at 9600 baud."));
    Serial.println(F("Possible causes:"));
    Serial.println(F(" - Wiring issue (check RX/TX are crossed correctly)."));
    Serial.println(F(" - The write in step 2 did not actually take effect."));
    Serial.println(F(" - The sensor is still using 115200 (try running this"));
    Serial.println(F("   sketch again, or check with a serial debugging tool"));
    Serial.println(F("   like the one shown in the datasheet)."));
  }
  Serial.println(F("==================================================="));
}

void loop() {
  // Nothing to do here; this is a one-time utility.
}
