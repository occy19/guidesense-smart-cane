// Bench test 02 — JSN-SR04T v3 waterproof ultrasonic (mode 0, HC-SR04 compatible)
// Wiring: 5V→5V, GND→GND, TRIG→D7, ECHO→D8
// ECHO is a 5 V signal: D8 is 5 V-tolerant on the UNO Q. Never use A0/A1 for it.
// Pass: readings stable within ±2 cm on a flat floor; minimum range ~20–25 cm.
#include <Arduino_RouterBridge.h>

const int TRIG_PIN = 7;
const int ECHO_PIN = 8;

float readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(20);   // v3 wants >= 10 µs; 20 µs is safer
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000UL);   // 30 ms timeout ≈ 5 m
  if (us == 0) return -1;                                // no echo
  return us / 58.0f;
}

void setup() {
  Monitor.begin();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Monitor.println("dist_cm");
}

void loop() {
  Monitor.println(readUltrasonicCm());
  delay(70);   // >= 60 ms between pings so old echoes die out
}
