# Build Log #1: Planning, problem statement and system architecture
**Date:** 22 Nov 2026 · **Stage:** Proposal (EIRP Phase 2)

## Problem
The white cane detects obstacles only at ground level, about 1 m ahead. Three gaps remain:
1. **Head/chest-level obstacles** (signboards, branches, truck tailgates) are missed.
2. **Drops** (open drains, kerbs, stairs) are found only when the tip falls in.
3. **Audio aids** compete with the traffic sounds blind pedestrians depend on.

Malaysian walkways make this worse. In Dec 2023 a visually impaired man fell into an uncovered drain on the tactile path near Dataran Merdeka. In 2024 The Star reported that over 30% of Brickfields' tactile paving was broken or blocked. Commercial smart canes cost well above RM1,000.

## Goal
A low-cost (< RM300) smart cane that tells the user **where** a hazard is (left / centre / right) and **how close** it is, through a vibration belt, so their ears stay free.

## Architecture
- **Sensors:** TF-Luna ToF LiDAR (forward + head height), JSN-SR04T waterproof ultrasonic (ground/drops), MPU6050 IMU (tilt + sweep direction)
- **Controller:** Arduino UNO Q. The STM32U585 MCU handles real-time sensing; the Linux side handles test-data logging.
- **Feedback:** 3 coin motors on a waist belt, PWM intensity + pulse rate ∝ proximity, distinct double pulse for drops
- **Link:** wired (prototype v1), then ESP32-C3 ESP-NOW (final)
- **Power:** 2× 18650 (1S2P) → TP4056 USB-C charger with protection → TPS61088 boost to 5 V

```
TF-Luna (I2C) ─┐
MPU6050 (I2C) ─┼─> UNO Q ─PWM─> NPN drivers ─> Belt L | C | R
JSN-SR04T ─────┘      │
                 18650 ×2 → TP4056 → TPS61088 (5 V)
```

## Key design decisions
| Decision | Why | Alternative rejected |
|---|---|---|
| LiDAR + ultrasonic fusion | LiDAR: narrow beam, long range; ultrasonic: wide beam, waterproof, sunlight-immune | HC-SR04 (not waterproof), VL53L1X (short range, weak in sunlight) |
| Software tilt compensation | No moving parts, easier to waterproof | Servo gimbal (kept as a V2 comparison) |
| TF-Luna on I2C | Shares the bus with the IMU and keeps the UNO Q UART free | UART mode |
| Haptic belt, not buzzer | Directional and silent, so ears stay free for traffic | Audio alerts |
| TPS61088 boost | The UNO Q needs a stable 5 V source rated ≥3 A; works from a single 3.0–4.2 V cell | MT3608 (~1 A max at 3 V in), XL6009 (needs ≥5 V input) |

## Repo
Folder structure (`/hardware`, `/software`, `/docs`), BOM, bench-test sketches and the core signal-processing library with unit tests are in the GitHub repo.

## Next
Order parts in Week 1 (Cytron + Lazada/Shopee), then bench-test each sensor on the UNO Q.
