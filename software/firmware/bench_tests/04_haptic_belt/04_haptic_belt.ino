// Bench test 04 — three coin vibration motors through NPN drivers
// Per motor: GPIO → 1 kΩ → 2N2222 base; emitter → GND; collector → motor(−);
//            motor(+) → battery rail (TP4056 OUT+, 3.0–4.2 V), not the UNO Q 3.3 V pin
//            (3 motors can draw ~250 mA together); 1N4148 across each motor (cathode to +) as flyback diode.
//            For a USB-only bench test, the 5 V pin plus a 22 Ω series resistor per motor is OK.
// Pins: LEFT D3, CENTRE D5, RIGHT D6 (PWM-capable)
// Pass: each motor is clearly felt through a shirt at every step; the user can tell L/C/R apart blind.
#include <Arduino_RouterBridge.h>

const int MOTOR[3] = {3, 5, 6};
const char* NAME[3] = {"LEFT", "CENTRE", "RIGHT"};

void pulse(int pin, uint8_t duty, int onMs, int offMs, int times) {
  for (int i = 0; i < times; i++) {
    analogWrite(pin, duty); delay(onMs);
    analogWrite(pin, 0);    delay(offMs);
  }
}

void setup() {
  Monitor.begin();
  for (int m = 0; m < 3; m++) { pinMode(MOTOR[m], OUTPUT); analogWrite(MOTOR[m], 0); }
}

void loop() {
  for (int m = 0; m < 3; m++) {
    Monitor.print("Testing "); Monitor.println(NAME[m]);
    for (int duty = 120; duty <= 255; duty += 45) pulse(MOTOR[m], duty, 150, 150, 2);  // strength sweep
    pulse(MOTOR[m], 255, 60, 60, 6);                                                  // fast = close
    delay(600);
  }
  Monitor.println("DROP pattern (double pulse, all motors)");
  for (int r = 0; r < 2; r++) {
    for (int m = 0; m < 3; m++) analogWrite(MOTOR[m], 255);
    delay(120);
    for (int m = 0; m < 3; m++) analogWrite(MOTOR[m], 0);
    delay(100);
  }
  delay(2000);
}
