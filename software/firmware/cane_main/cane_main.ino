// GuideSense — main cane firmware (Arduino UNO Q, STM32U585 MCU side)
// Pipeline: read IMU → read LiDAR + ultrasonic → tilt-compensate → filter → classify
//           → prioritise (drop > head/chest > body, crowd mode) → haptic belt pattern
// Prototype v1: belt motors wired to D3/D5/D6. Final: send 3 intensities over ESP-NOW (see belt_esp32c3/).
//
// STATUS: integration skeleton. Run bench_tests/01–04 first and copy calibrated values into CONFIG.
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Arduino_RouterBridge.h>
#include "guidesense_logic.h"
using namespace gs;

// ---------------- CONFIG ----------------
const uint8_t TFLUNA_ADDR = 0x10;
const int TRIG_PIN = 7, ECHO_PIN = 8, CAL_BTN = 2;
const int MOTOR[3] = {3, 5, 6};               // Left, Centre, Right

// Geometry (measure on the real cane)
float LIDAR_POS_FROM_TIP_CM = 30.0f;          // along the shaft
float US_POS_FROM_TIP_CM    = 30.0f;
float LIDAR_OFFSET_DEG      = 45.0f;          // beam angle relative to the shaft (adjustable mount slot)
float US_OFFSET_DEG         = 0.0f;           // ultrasonic follows the shaft (~45° down)

// Calibration (set by the one-button routine)
float calibPitchDeg  = -45.0f;                // normal holding angle
float halfSwingDeg   = 30.0f;                 // half the user's sweep width

Thresholds TH;
// ---------------------------------------

Adafruit_MPU6050 mpu;
MedianFilter<5> lidarMed, usMed;
Debounce headDeb(3), bodyDeb(3);
DropDetector drop(0.05f, 3, 10.0f);
CrowdMode crowd;
Ema sweepCentre(0.02f);

float pitchDeg = -45, yawDeg = 0;
unsigned long lastUs, lastUsPing = 0, lastLog = 0;

// ---------- sensors ----------
bool readTfLuna(float &cm) {
  Wire.beginTransmission(TFLUNA_ADDR); Wire.write(0x00);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(TFLUNA_ADDR, (uint8_t)4) != 4) return false;
  uint16_t d = Wire.read() | (Wire.read() << 8);
  uint16_t s = Wire.read() | (Wire.read() << 8);
  if (s < 100 || s == 65535) return false;     // unreliable return (e.g. strong sunlight)
  cm = d; return true;
}

float readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(20);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000UL);
  return us ? us / 58.0f : -1;
}

void updateImu() {
  sensors_event_t a, g, t; mpu.getEvent(&a, &g, &t);
  float dt = (micros() - lastUs) / 1e6f; lastUs = micros();
  float accPitch = atan2f(a.acceleration.x, sqrtf(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 57.2958f;
  pitchDeg = 0.98f * (pitchDeg + g.gyro.y * 57.2958f * dt) + 0.02f * accPitch;
  yawDeg  += g.gyro.z * 57.2958f * dt;         // drifts, so we only use it relative to the sweep centre
  sweepCentre.push(yawDeg);
}

// ---------- haptics (non-blocking) ----------
struct MotorState { Pulse p; unsigned long t0; } ms[3];
Hazard activeHazard = Hazard::None;

void setMotor(int i, Pulse p) { if (p.periodMs != ms[i].p.periodMs || p.duty != ms[i].p.duty) { ms[i].p = p; ms[i].t0 = millis(); } }

void serviceMotors() {
  unsigned long now = millis();
  for (int i = 0; i < 3; i++) {
    if (ms[i].p.duty == 0) { analogWrite(MOTOR[i], 0); continue; }
    unsigned long phase = (now - ms[i].t0) % ms[i].p.periodMs;
    analogWrite(MOTOR[i], phase < ms[i].p.periodMs / 2 ? ms[i].p.duty : 0);
  }
}

void dropAlert() {                              // distinct double pulse on all motors (blocking ~450 ms, rare)
  for (int r = 0; r < 2; r++) {
    for (int i = 0; i < 3; i++) analogWrite(MOTOR[i], 255);
    delay(120);
    for (int i = 0; i < 3; i++) analogWrite(MOTOR[i], 0);
    delay(100);
  }
}

// ---------- calibration ----------
void calibrate() {
  // 1) hold still for 2 s → holding angle; 2) sweep normally for 5 s → swing width; 3) double buzz to confirm
  Monitor.println("CAL: hold still");
  float sum = 0; int n = 0; unsigned long t = millis();
  while (millis() - t < 2000) { updateImu(); sum += pitchDeg; n++; delay(10); }
  calibPitchDeg = sum / n;
  Monitor.println("CAL: sweep now");
  float lo = 1e9, hi = -1e9; t = millis();
  while (millis() - t < 5000) { updateImu(); float o = yawDeg - sweepCentre.value; lo = min(lo, o); hi = max(hi, o); delay(10); }
  halfSwingDeg = max(10.0f, (hi - lo) / 2);
  dropAlert();
  Monitor.print("CAL done pitch="); Monitor.print(calibPitchDeg); Monitor.print(" halfSwing="); Monitor.println(halfSwingDeg);
}

void setup() {
  Monitor.begin();
  Wire.begin();
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT); pinMode(CAL_BTN, INPUT_PULLUP);
  for (int i = 0; i < 3; i++) { pinMode(MOTOR[i], OUTPUT); analogWrite(MOTOR[i], 0); }
  if (!mpu.begin()) Monitor.println("MPU6050 not found");
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  lastUs = micros();
  Monitor.println("t_ms,pitch,yaw_off,lidar_cm,us_cm,hazard,side,crowd");
}

void loop() {
  if (digitalRead(CAL_BTN) == LOW) { delay(30); if (digitalRead(CAL_BTN) == LOW) calibrate(); }
  updateImu();
  if (tiltOutOfRange(pitchDeg, calibPitchDeg)) { for (int i = 0; i < 3; i++) setMotor(i, {0, 0}); serviceMotors(); return; }

  float canePitch = pitchDeg;
  float lidarBeam = canePitch + LIDAR_OFFSET_DEG, usBeam = canePitch + US_OFFSET_DEG;
  float lidarH = LIDAR_POS_FROM_TIP_CM * sinf(-canePitch * DEG2RAD);   // sensor height above ground

  // Forward LiDAR (every loop, ~100 Hz)
  float lcm = -1, lraw;
  if (readTfLuna(lraw)) lcm = lidarMed.push(lraw);
  Hazard fwd = classifyForward(lcm, lidarBeam, lidarH, TH);
  bool head = headDeb.push(fwd == Hazard::HeadChest);
  bool body = bodyDeb.push(fwd == Hazard::Body);
  Hazard confirmed = head ? Hazard::HeadChest : (body ? Hazard::Body : Hazard::None);
  bool crowdOn = crowd.update(fwd, horizontalCm(lcm, lidarBeam));

  // Downward ultrasonic (staggered ping, every 70 ms)
  float ucm = -1; bool isDrop = false;
  if (millis() - lastUsPing >= 70) {
    lastUsPing = millis();
    float r = readUltrasonicCm();
    if (r > 0) { ucm = usMed.push(r); isDrop = drop.update(ucm, usBeam); }
  }

  Hazard h = prioritise(confirmed, isDrop, crowdOn);
  Side side = sideFromSweep(yawDeg - sweepCentre.value, halfSwingDeg);

  if (h == Hazard::Drop) dropAlert();
  for (int i = 0; i < 3; i++) setMotor(i, {0, 0});
  if (h == Hazard::HeadChest || h == Hazard::Body) setMotor((int)side, pulseForDistance(horizontalCm(lcm, lidarBeam), TH.alertRangeCm));
  serviceMotors();

  if (millis() - lastLog >= 100) {             // 10 Hz CSV for test logging
    lastLog = millis();
    Monitor.print(millis()); Monitor.print(","); Monitor.print(pitchDeg); Monitor.print(",");
    Monitor.print(yawDeg - sweepCentre.value); Monitor.print(","); Monitor.print(lcm); Monitor.print(",");
    Monitor.print(ucm); Monitor.print(","); Monitor.print((int)h); Monitor.print(",");
    Monitor.print((int)side); Monitor.print(","); Monitor.println(crowdOn ? 1 : 0);
  }
  delay(10);
}
