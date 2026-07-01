# DFRobot_SEN0676 - Arduino Library (Modbus-RTU)

Library to read the SEN0676 80GHz radar water level sensor over Modbus-RTU,
implemented manually (no external dependencies like ModbusMaster) for better
reliability. Compatible with ESP32, ESP8266, and any board that supports a
`Stream` object (HardwareSerial or SoftwareSerial).

## Installation

1. Copy the whole `DFRobot_SEN0676` folder into your Arduino libraries folder:
   - Windows: `Documents/Arduino/libraries/`
   - macOS:   `~/Documents/Arduino/libraries/`
   - Linux:   `~/Arduino/libraries/`
2. Restart the Arduino IDE.
3. Open the example: `File > Examples > DFRobot_SEN0676 > BasicReading`

## Basic usage

```cpp
#include <DFRobot_SEN0676.h>

DFRobot_SEN0676 sensor(Serial2); // or your SoftwareSerial object

void setup() {
  Serial2.begin(115200, SERIAL_8N1, 17, 16); // RX, TX (ESP32 example)
  sensor.begin();
  sensor.setInstallationHeight(1000); // cm, adjust to your actual installation
}

void loop() {
  uint16_t level;
  if (sensor.readWaterLevel(level)) {
    Serial.println(level); // mm
  }
}
```

## Wiring notes

- Direct TTL UART connection.
- ESP8266 must be configured to 9600 baud for TX/RX serial communication; 115200 could result in numerous errors. Use the `speed_change` directory to change the SEN0676 speed to 9600.
- Cross TX/RX: sensor TX -> microcontroller RX, sensor RX -> microcontroller TX.
- You must call `setInstallationHeight()` before the water level can be
  read correctly (per the sensor's datasheet).

## API

| Method | Description |
|---|---|
| `begin()` | Initializes internal state (does not open the serial port) |
| `setTimeout(ms)` | Changes the response timeout (300ms by default) |
| `setInstallationHeight(cm)` | Sets the installation height (register 0x0005) |
| `getInstallationHeight(cm&)` | Reads the currently configured installation height |
| `readEmptyHeight(mm&)` | Reads the detected empty height (register 0x0001) |
| `readWaterLevel(mm&)` | Reads the current water level (register 0x0003) |
| `readRegister(addr, value&)` | Generic read of any Modbus register |
| `writeRegister(addr, value, writeTimeoutMs, retries)` | Generic write of any Modbus register, with configurable timeout and retries |
