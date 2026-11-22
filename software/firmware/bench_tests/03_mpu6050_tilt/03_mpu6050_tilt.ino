// Bench test 03 — MPU6050 (GY-521) pitch + yaw-rate with a complementary filter
// Wiring: VCC→3.3V, GND→GND, SDA→SDA, SCL→SCL (shares the bus with TF-Luna 0x10; MPU is 0x68)
// Library: Adafruit MPU6050 (Library Manager)
// Mounting assumption: board X axis along the cane, pointing to the tip. Change AXIS_* if mounted differently.
// Pass: pitch matches a phone inclinometer within ±2° when still, and settles in < 1 s after a swing.
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Arduino_RouterBridge.h>

Adafruit_MPU6050 mpu;
float pitchDeg = 0;
unsigned long lastUs;
const float ALPHA = 0.98f;   // gyro weight in the complementary filter

void setup() {
  Monitor.begin();
  Wire.begin();
  if (!mpu.begin()) { Monitor.println("MPU6050 not found"); while (true) delay(10); }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  lastUs = micros();
  Monitor.println("pitch_deg,yaw_rate_dps");
}

void loop() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  float dt = (micros() - lastUs) / 1e6f; lastUs = micros();

  float accPitch = atan2f(a.acceleration.x,
                          sqrtf(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 57.2958f;
  float gyroRateY = g.gyro.y * 57.2958f;          // deg/s
  pitchDeg = ALPHA * (pitchDeg + gyroRateY * dt) + (1 - ALPHA) * accPitch;

  Monitor.print(pitchDeg); Monitor.print(","); Monitor.println(g.gyro.z * 57.2958f);
  delay(10);   // ~100 Hz
}
