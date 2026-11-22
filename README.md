# GuideSense — Smart Cane with Directional Haptic Belt for the Visually Impaired

**EIRP 26/27** (Engineering Innovation Recognition Programme, UNM Robotics Society × Dept. of EEE, supported by Syqnal)

**Team:** LC Team
| Role | Name |
|---|---|
| Leader · firmware (sensing & signal processing) | Lee Lit Hang |
| Firmware (feedback, wireless, test tooling) · repo, Syqnal & BOM | Cheng Chu Young |
| Hardware & 3D design | Chong Khin Hyen |
| Hardware & 3D design | Lau Kai Ren |

**Syqnal project page:** [insert link once published]

---

## The problem
A white cane only finds obstacles at ground level, about 1 m ahead. It misses head- and chest-height hazards (signboards, branches, truck tailgates) and finds drops (open drains, kerbs, stairs) only when the tip falls in. Audio aids compete with the traffic sounds that blind pedestrians rely on. Commercial smart canes cost well above RM1,000.

## Our solution
A low-cost (< RM300) smart cane that combines:

- **TF-Luna ToF LiDAR**: forward obstacles, including head height (0.2–8 m, 2° beam)
- **JSN-SR04T waterproof ultrasonic**: ground and drop detection, aimed ~45° down
- **MPU6050 IMU**: tilt and swing compensation, plus sweep direction
- **Haptic belt**: 3 coin vibration motors (left / centre / right). The closer the hazard, the faster and stronger the pulse; drops get a distinct double pulse.
- **Arduino UNO Q**: real-time sensing on the STM32U585 MCU, with test logging on the Linux side
- Cane-to-belt link: wired in prototype v1, **ESP32-C3 + ESP-NOW** in the final version

## System architecture
```
 TF-Luna LiDAR ──(I2C 0x10)──┐
 MPU6050 IMU ────(I2C 0x68)──┤
 JSN-SR04T ──(TRIG D7/ECHO D8)┼──> Arduino UNO Q (STM32U585 MCU)
 Calibrate button ──(D2)─────┘        │  tilt-compensate → filter → classify → prioritise
                                      │
                         PWM D3/D5/D6 ▼ (v1 wired)   or   ESP-NOW (final)
                      NPN drivers → Belt motors L | C | R
 Power: 2× 18650 (1S2P) → TP4056 (USB-C charge + protection) → TPS61088 boost @ 5 V → UNO Q 5V
```

## Repository layout
```
hardware/
  bom/            BOM.csv — parts, quantities, prices, purchase channel
  cad/            STL/STEP files for handle enclosure, sensor mount, belt pods
  schematics/     KiCad project + PDF exports
  datasheets/     links to component datasheets
software/
  firmware/
    cane_main/        main UNO Q sketch (sensing → haptic output)
    bench_tests/      one sketch per component for bench testing
  belt_esp32c3/       ESP-NOW receiver for the wireless belt (final version)
  tests/              host-side unit tests for the signal-processing logic
docs/
  build-logs/     weekly engineering logs (mirrored on Syqnal)
  testing/        test plan, obstacle-course and rain/spray results
  media/          photos and short clips
```

## Getting started (firmware)
1. Install **Arduino IDE 2.3.x** (or Arduino App Lab) and the **Arduino UNO Q** board package.
2. Install the libraries **Arduino_RouterBridge** (for `Monitor` output) and **Adafruit MPU6050**.
3. Run the bench tests in `software/firmware/bench_tests/` in order (01 → 04) before `cane_main`.
4. The UNO Q MCU pins use 3.3 V logic but are 5 V-tolerant **except A0/A1**. Do not wire 5 V sensor outputs to A0/A1.

Host unit tests for the shared logic (no hardware needed):
```bash
cd software/tests && g++ -std=c++17 -I../firmware/cane_main test_logic.cpp -o test_logic && ./test_logic
```

## Status
- [x] Proposal and architecture
- [ ] Sensor bench tests
- [ ] Tilt compensation and drop detection
- [ ] Haptic belt v1 (wired)
- [ ] Integrated prototype v1
- [ ] Waterproofing and spray test
- [ ] Obstacle-course and blindfolded user testing
- [ ] Wireless belt (ESP-NOW)

## Licence
Software: MIT (see `LICENSE`). Hardware design files: CC BY-SA 4.0.
