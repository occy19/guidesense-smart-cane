# Test plan

| # | Test | Method | Pass criterion |
|---|---|---|---|
| T1 | LiDAR accuracy | Wall at 0.5/1/2/3 m, indoor and outdoor midday | ±6 cm up to 3 m; valid outdoor reading rate ≥ 90% |
| T2 | Ultrasonic accuracy | Floor at known angles/heights | ±2 cm, stable median |
| T3 | Tilt compensation | Swing cane ±30°, compare corrected vs tape-measured distance | Error < 10 cm across the swing |
| T4 | Drop detection | Step down 10/15/20 cm, kerb, drain mock-up; also a 5° ramp | Drop detected ≥ 90%; ramp gives no false drop |
| T5 | Head-height detection | Bar at 120–170 cm, signboard mock-up | Detected ≥ 90% at ≥ 1.0 m |
| T6 | Latency | Log timestamp of obstacle crossing vs motor ON (video at 240 fps) | < 150 ms end-to-end |
| T7 | Direction | Obstacle left/centre/right; blindfolded user names the side | ≥ 85% correct |
| T8 | Crowd mode | Walk through a group of 5+ people | Body alerts muted, drops/head-height still alert |
| T9 | Rain/spray | Garden spray, 5 min, all angles, then function test | No water inside; all sensors work |
| T10 | Battery life | Continuous run from full charge, log current | ≥ 3 h |
| T11 | User trial | Blindfolded volunteers: plain cane vs GuideSense on the same course | Fewer collisions; record walking time and confidence (1–5) |

All results are logged as CSV from the 10 Hz `Monitor` output, plotted, and posted in the weekly Syqnal log.
