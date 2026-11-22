# Software

| Path | What it is |
|---|---|
| `firmware/bench_tests/01_tfluna_i2c` | TF-Luna distance/strength over I2C |
| `firmware/bench_tests/02_jsnsr04t` | JSN-SR04T ultrasonic distance |
| `firmware/bench_tests/03_mpu6050_tilt` | MPU6050 pitch (complementary filter) + yaw rate |
| `firmware/bench_tests/04_haptic_belt` | 3 motors: strength sweep, fast pulse, drop pattern |
| `firmware/cane_main` | Main sketch + `guidesense_logic.h` (shared, hardware-independent algorithms) |
| `belt_esp32c3` | ESP-NOW belt receiver (final wireless version) |
| `tests` | Host unit tests for `guidesense_logic.h` |

## Algorithm summary
1. **Tilt compensation**: complementary filter (98% gyro / 2% accel) gives the cane pitch θ. Beam angle = θ + mount offset. Horizontal distance = d·cos(beam), hit height = sensor height + d·sin(beam).
2. **Forward hazards**: the LiDAR hit point is classed as *head/chest* (≥100 cm), *body*, or *ground* (<25 cm, left to the cane tip). Only hits within 200 cm horizontally count.
3. **Drop detection**: the ultrasonic depth below the sensor is tracked with a slow EMA ("ground reference"). A jump of ≥10 cm for 3 consecutive readings is a drop. Gradual change is a slope.
4. **Noise rejection**: median-of-5 filters, 3-reading debounce, staggered sensor timing, readings ignored when tilt is >25° from the calibrated angle.
5. **Direction**: yaw relative to the running sweep centre → left / centre / right third of the calibrated swing.
6. **Feedback**: closer = higher PWM duty (120–255) and faster pulse (700→100 ms). Drop = double pulse on all motors.
7. **Priority / crowd mode**: drop > head/chest > body. When many close body-level hits occur, body alerts are muted.

## UNO Q notes
- Print with `Monitor` (library `Arduino_RouterBridge`); `Serial` output does not appear in the App Lab/IDE console.
- MCU pins are 3.3 V logic and 5 V-tolerant except A0/A1.
