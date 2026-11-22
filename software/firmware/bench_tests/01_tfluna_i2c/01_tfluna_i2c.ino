// Bench test 01 — TF-Luna LiDAR over I2C on the Arduino UNO Q
// Wiring (TF-Luna 6-pin):  1 VCC→5V  2 SDA→SDA  3 SCL→SCL  4 GND→GND  5 CFG→GND (selects I2C)  6 n/c
// TF-Luna I2C lines are 3.3 V LVTTL, so they match the UNO Q's 3.3 V I2C directly.
// Pass: distance tracks a tape measure within ±6 cm from 0.2–3 m; strength > 100.
#include <Wire.h>
#include <Arduino_RouterBridge.h>   // UNO Q: print with Monitor, not Serial

const uint8_t TFLUNA_ADDR = 0x10;

bool readTfLuna(uint16_t &distCm, uint16_t &strength, float &tempC) {
  Wire.beginTransmission(TFLUNA_ADDR);
  Wire.write(0x00);                         // DIST_LOW register
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(TFLUNA_ADDR, (uint8_t)6) != 6) return false;
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  distCm   = b[0] | (b[1] << 8);
  strength = b[2] | (b[3] << 8);
  tempC    = (int16_t)(b[4] | (b[5] << 8)) / 100.0f;
  return true;
}

void setup() {
  Monitor.begin();
  Wire.begin();
  delay(500);
  Monitor.println("dist_cm,strength,temp_c,valid");
}

void loop() {
  uint16_t d, s; float t;
  if (readTfLuna(d, s, t)) {
    bool valid = (s >= 100 && s != 65535);  // weak or saturated returns are unreliable
    Monitor.print(d); Monitor.print(","); Monitor.print(s); Monitor.print(",");
    Monitor.print(t); Monitor.print(","); Monitor.println(valid ? 1 : 0);
  } else {
    Monitor.println("I2C read failed - check CFG pin 5 is tied to GND and SDA/SCL wiring");
  }
  delay(50);   // 20 Hz
}
