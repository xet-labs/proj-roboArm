#include <AccelStepper.h>
#include <Adafruit_VL53L0X.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <Wire.h>
#include "src/lib/lib.h"

#define DEBUG
#define ENABLE_VL53L0X 0
Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();

void setup()
{
  Serial.begin(115200);
  conf_pins();        Serial.println();
  init_all_motors();  Serial.println();

  WiFi.mode(WIFI_MODE_APSTA);
  net::ap::init();    Serial.println();
  net::sta::init();   Serial.println();
  // net::tcp::init();
  net::udp::init();

#if ENABLE_VL53L0X
  // Initialize default I2c bus
  Wire.begin(21, 22);
  // VL53L0X
  if (!sensor1.begin(&Wire))
  {
    Serial.println(F("[VL53L0X] ERR Sensor not detected"));
    while (true)
      ; // Halt if sensor fails
  }
#endif
}

void loop()
{
  handle::serial();
  lgc::core.call();
}
