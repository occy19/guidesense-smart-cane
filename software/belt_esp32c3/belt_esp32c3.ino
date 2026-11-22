// GuideSense belt receiver — ESP32-C3 (final wireless version, ESP-NOW)
// Receives 3 bytes {left, centre, right} (PWM 0–255) + 1 byte pattern ~20×/s from the cane.
// If no packet arrives for 500 ms → "link lost" buzz so the user knows feedback is unavailable.
// STATUS: stub, to be completed after prototype v1 (wired) works.
#include <WiFi.h>
#include <esp_now.h>

const int MOTOR[3] = {2, 3, 4};      // ESP32-C3 SuperMini GPIOs (PWM via LEDC)
struct Packet { uint8_t duty[3]; uint8_t pattern; } pkt;
volatile unsigned long lastRx = 0;

void onRecv(const esp_now_recv_info_t*, const uint8_t* data, int len) {
  if (len == sizeof(Packet)) { memcpy(&pkt, data, len); lastRx = millis(); }
}

void setup() {
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onRecv);
  for (int i = 0; i < 3; i++) { pinMode(MOTOR[i], OUTPUT); analogWrite(MOTOR[i], 0); }
}

void loop() {
  if (millis() - lastRx > 500) {                 // link lost: slow triple buzz on centre
    for (int i = 0; i < 3; i++) { analogWrite(MOTOR[1], 200); delay(80); analogWrite(MOTOR[1], 0); delay(80); }
    delay(1000);
    return;
  }
  for (int i = 0; i < 3; i++) analogWrite(MOTOR[i], pkt.duty[i]);
  delay(10);
}
