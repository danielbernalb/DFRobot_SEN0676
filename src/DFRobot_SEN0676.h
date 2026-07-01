/*
  DFRobot_SEN0676.h
  Library for the SEN0676 80GHz radar water level sensor (Modbus-RTU)

  Manual implementation of the Modbus-RTU protocol (no external dependencies),
  works with any Stream object (HardwareSerial or SoftwareSerial), so it is
  compatible with ESP32, ESP8266, AVR, etc.

  Typical usage:

    #include <DFRobot_SEN0676.h>

    DFRobot_SEN0676 sensor(Serial2); // or SoftwareSerial, etc.

    void setup() {
      Serial2.begin(115200);
      sensor.setInstallationHeight(100); // cm, adjust to your installation
    }

    void loop() {
      uint16_t level;
      if (sensor.readWaterLevel(level)) {
        // level in mm
      }
    }
*/

#ifndef DFRobot_SEN0676_H
#define DFRobot_SEN0676_H

#include <Arduino.h>

// ---------- Register addresses (per datasheet) ----------
#define SEN0676_REG_EMPTY_HEIGHT     0x0001  // R   - empty height, mm
#define SEN0676_REG_WATER_LEVEL      0x0003  // R   - water level, mm
#define SEN0676_REG_INSTALL_HEIGHT   0x0005  // R/W - installation height, cm
#define SEN0676_REG_DEVICE_ADDRESS   0x03F4  // R/W - device address
#define SEN0676_REG_BAUD_RATE        0x03F6  // R/W - baud rate (actual value / 100)
#define SEN0676_REG_RANGE            0x07D4  // R/W - maximum range, m

class DFRobot_SEN0676 {
  public:
    // serial: an already-initialized Stream object (Serial2.begin(), SoftwareSerial.begin(), etc.)
    // slaveAddr: Modbus address of the sensor (default 1)
    DFRobot_SEN0676(Stream &serial, uint8_t slaveAddr = 0x01);

    // Initializes the library's internal state. Does not open the serial port
    // (you must do that yourself beforehand, e.g. Serial2.begin(115200)).
    bool begin();

    // Changes the response timeout (ms). Default is 300 ms.
    void setTimeout(uint16_t timeoutMs);

    // Configures the installation height (register 0x0005), in cm.
    // Must be called before the water level can be read correctly.
    // Internally uses a longer timeout and retries, since some sensors
    // take longer to confirm when the written value actually changes
    // (they commit to non-volatile memory before responding).
    bool setInstallationHeight(uint16_t heightCm);

    // Reads the installation height currently configured in the sensor (cm).
    bool getInstallationHeight(uint16_t &heightCm);

    // Reads the empty height (detected clear distance), in mm.
    bool readEmptyHeight(uint16_t &valueMm);

    // Reads the current water level, in mm.
    bool readWaterLevel(uint16_t &valueMm);

    // Generic read/write of any register (Modbus function 03 / 06)
    // retries: additional retries on timeout/CRC failure. Useful on ESP8266
    // SoftwareSerial, where WiFi/MQTT activity can occasionally corrupt a
    // byte mid-frame and cause a transient CRC or timeout failure.
    bool readRegister(uint16_t regAddr, uint16_t &value, uint8_t retries = 2);

    // writeTimeoutMs: timeout specific to this write (longer by default than
    // the read timeout, because some real writes -value different from the
    // stored one- involve a commit to the sensor's non-volatile memory).
    // retries: additional retries if the first response arrives too late.
    bool writeRegister(uint16_t regAddr, uint16_t value,
                        uint16_t writeTimeoutMs = 800, uint8_t retries = 2);

  private:
    Stream &_serial;
    uint8_t _slaveAddr;
    uint16_t _timeoutMs;
    uint8_t _rxBuffer[32];

    uint16_t modbusCRC16(const uint8_t *buf, uint8_t len);
    void sendFrame(uint8_t *frame, uint8_t len);
    uint8_t readResponse(uint16_t timeoutMs);
    bool checkCRC(uint8_t *data, uint8_t len);
};

#endif
