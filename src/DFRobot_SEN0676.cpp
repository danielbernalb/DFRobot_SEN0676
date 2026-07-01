/*
  DFRobot_SEN0676.cpp
  Implementation of the library for the SEN0676 80GHz radar sensor (Modbus-RTU)
*/

#include "DFRobot_SEN0676.h"

DFRobot_SEN0676::DFRobot_SEN0676(Stream &serial, uint8_t slaveAddr)
  : _serial(serial), _slaveAddr(slaveAddr), _timeoutMs(300) {
}

bool DFRobot_SEN0676::begin() {
  // We don't open the port here: the user controls baud rate/pins according
  // to their board (HardwareSerial, SoftwareSerial, etc.) and must call
  // .begin() on that object before passing it to the constructor.
  return true;
}

void DFRobot_SEN0676::setTimeout(uint16_t timeoutMs) {
  _timeoutMs = timeoutMs;
}

// ---------- MODBUS CRC16 (polynomial 0xA001) ----------
uint16_t DFRobot_SEN0676::modbusCRC16(const uint8_t *buf, uint8_t len) {
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
  return crc; // Low byte first, high byte second when sending
}

// ---------- SEND RAW FRAME ----------
void DFRobot_SEN0676::sendFrame(uint8_t *frame, uint8_t len) {
  // Append CRC at the end of the frame
  uint16_t crc = modbusCRC16(frame, len);
  frame[len]     = crc & 0xFF;        // CRC low
  frame[len + 1] = (crc >> 8) & 0xFF; // CRC high

  // Small bus-settle delay before flushing the RX buffer. This helps avoid
  // starting a new transaction while a stray/late byte from the previous
  // one (or WiFi-induced jitter on SoftwareSerial) is still in flight.
  delay(2);

  // Clear any leftover bytes pending in the receive buffer
  while (_serial.available()) _serial.read();

  _serial.write(frame, len + 2);
  _serial.flush(); // Wait for transmission to finish before continuing
}

// ---------- READ RESPONSE ----------
// Returns the number of bytes read, or 0 on timeout
uint8_t DFRobot_SEN0676::readResponse(uint16_t timeoutMs) {
  uint8_t idx = 0;
  unsigned long start = millis();
  unsigned long lastByteTime = millis();

  while (millis() - start < timeoutMs) {
    if (_serial.available()) {
      _rxBuffer[idx++] = _serial.read();
      lastByteTime = millis();
      if (idx >= sizeof(_rxBuffer)) break;
    } else if (idx > 0 && (millis() - lastByteTime > 20)) {
      // If we already started receiving and 20ms passed without new bytes,
      // we assume the frame has ended (end of Modbus-RTU packet)
      break;
    }
  }
  return idx;
}

// Verifies that the response CRC is correct
bool DFRobot_SEN0676::checkCRC(uint8_t *data, uint8_t len) {
  if (len < 3) return false;
  uint16_t calcCrc = modbusCRC16(data, len - 2);
  uint16_t recvCrc = data[len - 2] | (data[len - 1] << 8);
  return calcCrc == recvCrc;
}

// ---------- FUNCTION 03: READ A REGISTER ----------
// Retries on failure by default. This matters most on ESP8266 SoftwareSerial,
// where WiFi/MQTT activity can briefly disable interrupts and corrupt or
// drop a byte mid-frame, causing an occasional CRC mismatch or timeout even
// though the wiring and timing are otherwise correct.
bool DFRobot_SEN0676::readRegister(uint16_t regAddr, uint16_t &value, uint8_t retries) {
  uint8_t frame[8];
  frame[0] = _slaveAddr;
  frame[1] = 0x03;                  // Function: read registers
  frame[2] = (regAddr >> 8) & 0xFF;
  frame[3] = regAddr & 0xFF;
  frame[4] = 0x00;                  // Number of registers (high)
  frame[5] = 0x01;                  // Number of registers (low) -> 1 register

  for (uint8_t attempt = 0; attempt <= retries; attempt++) {
    sendFrame(frame, 6);

    uint8_t len = readResponse(_timeoutMs);
    if (len >= 7 &&
        _rxBuffer[0] == _slaveAddr &&
        _rxBuffer[1] == 0x03 &&
        checkCRC(_rxBuffer, len)) {
      value = (_rxBuffer[3] << 8) | _rxBuffer[4];
      return true;
    }
    // Short pause before retrying, so any in-flight interrupt storm
    // (WiFi/MQTT) has time to settle before the next attempt.
    delay(15);
  }

  return false;
}

// ---------- FUNCTION 06: WRITE A REGISTER ----------
// Uses a longer timeout than reads and retries, because some sensors take
// longer to respond when the written value actually changes (they commit
// to non-volatile memory before sending the echo back).
bool DFRobot_SEN0676::writeRegister(uint16_t regAddr, uint16_t value,
                             uint16_t writeTimeoutMs, uint8_t retries) {
  uint8_t frame[8];
  frame[0] = _slaveAddr;
  frame[1] = 0x06;                  // Function: write a single register
  frame[2] = (regAddr >> 8) & 0xFF;
  frame[3] = regAddr & 0xFF;
  frame[4] = (value >> 8) & 0xFF;
  frame[5] = value & 0xFF;

  for (uint8_t attempt = 0; attempt <= retries; attempt++) {
    sendFrame(frame, 6);

    uint8_t len = readResponse(writeTimeoutMs);
    if (len >= 8 &&
        _rxBuffer[0] == _slaveAddr &&
        _rxBuffer[1] == 0x06 &&
        checkCRC(_rxBuffer, len)) {
      return true;
    }
    // If no valid response arrived, wait a bit and retry
    // (give the sensor time to finish any internal commit).
    delay(100);
  }

  return false;
}

// ---------- CONVENIENCE FUNCTIONS ----------
bool DFRobot_SEN0676::setInstallationHeight(uint16_t heightCm) {
  return writeRegister(SEN0676_REG_INSTALL_HEIGHT, heightCm);
}

bool DFRobot_SEN0676::getInstallationHeight(uint16_t &heightCm) {
  return readRegister(SEN0676_REG_INSTALL_HEIGHT, heightCm);
}

bool DFRobot_SEN0676::readEmptyHeight(uint16_t &valueMm) {
  return readRegister(SEN0676_REG_EMPTY_HEIGHT, valueMm);
}

bool DFRobot_SEN0676::readWaterLevel(uint16_t &valueMm) {
  return readRegister(SEN0676_REG_WATER_LEVEL, valueMm);
}
