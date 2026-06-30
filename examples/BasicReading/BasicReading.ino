/*
  BasicReading.ino
  Example usage of the DFRobot_SEN0676 library (80GHz radar sensor, Modbus-RTU)

  Wiring: direct TTL UART.
    - ESP32:   Serial2, RX=17, TX=16
    - ESP8266: SoftwareSerial, RX=13, TX=12

  Remember to cross TX/RX: sensor TX -> microcontroller RX, and vice versa.
*/

#include <DFRobot_SEN0676.h>

// Installation height in cm (distance from radar to reference bottom)
// IMPORTANT: adjust this value to your actual installation.
#define INSTALL_HEIGHT_CM  1000

#if defined(ESP32)
  #define SENSOR_RX_PIN  17
  #define SENSOR_TX_PIN  16
  DFRobot_SEN0676 sensor(Serial2);

#elif defined(ESP8266)
  #include <SoftwareSerial.h>
  #define SENSOR_RX_PIN  13
  #define SENSOR_TX_PIN  12
  SoftwareSerial sensorSerial(SENSOR_RX_PIN, SENSOR_TX_PIN); // RX, TX
  DFRobot_SEN0676 sensor(sensorSerial);

#else
  #error "This example targets ESP8266 or ESP32"
#endif

void setup() {
  Serial.begin(115200);   // USB serial monitor
  delay(200);

#if defined(ESP32)
  Serial2.begin(115200, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
#elif defined(ESP8266)
  sensorSerial.begin(115200);
#endif

  sensor.begin();

  Serial.println(F("=== SEN0676 - Modbus RTU Reader ==="));
  Serial.print(F("Setting installation height to "));
  Serial.print(INSTALL_HEIGHT_CM);
  Serial.println(F(" cm..."));

  if (sensor.setInstallationHeight(INSTALL_HEIGHT_CM)) {
    Serial.println(F("Installation height set OK."));
  } else {
    Serial.println(F("ERROR: could not set installation height."));
    Serial.println(F("Check wiring (RX/TX swapped), sensor address and baud rate."));
  }

  delay(200);
}

void loop() {
  uint16_t emptyHeight = 0;
  uint16_t waterLevel  = 0;

  bool okEmpty = sensor.readEmptyHeight(emptyHeight);
  delay(50); // Small pause between frames
  bool okLevel = sensor.readWaterLevel(waterLevel);

  if (okEmpty) {
    Serial.print(F("Empty height: "));
    Serial.print(emptyHeight);
    Serial.println(F(" mm"));
  } else {
    Serial.println(F("Error reading empty height (timeout/CRC)."));
  }

  if (okLevel) {
    Serial.print(F("Water level: "));
    Serial.print(waterLevel);
    Serial.println(F(" mm"));
  } else {
    Serial.println(F("Error reading water level (timeout/CRC)."));
  }

  Serial.println(F("------------------------------------"));
  delay(1000);
}
