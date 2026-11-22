# Hardware

| Folder | Contents |
|---|---|
| `bom/` | `BOM.csv`: every part with quantity, price, link and funding channel |
| `cad/` | 3D-printed parts (STL for printing + STEP/source files) |
| `schematics/` | Circuit schematic (KiCad) and PDF export |
| `datasheets/` | Links to component datasheets |

## Pin map (Arduino UNO Q, MCU side, 3.3 V logic)
| Signal | UNO Q pin | Notes |
|---|---|---|
| TF-Luna SDA / SCL | SDA / SCL | I2C address 0x10; TF-Luna pin 5 (CFG) tied to GND to select I2C |
| MPU6050 SDA / SCL | SDA / SCL | I2C address 0x68; powered from 3.3 V |
| JSN-SR04T TRIG | D7 | |
| JSN-SR04T ECHO | D8 | 5 V signal, OK because D8 is 5 V-tolerant (A0/A1 are **not**) |
| Calibrate button | D2 | INPUT_PULLUP, button to GND |
| Motor LEFT / CENTRE / RIGHT | D3 / D5 / D6 | PWM → 1 kΩ → 2N2222 base; 1N4148 flyback per motor |

## Power
2× 18650 in parallel (1S2P, ~4400 mAh) → TP4056 USB-C charger with protection (take power from **OUT+/OUT−**, never bypass the protection) → TPS61088 boost (2.7–12 V in, 10 A switch) at **5 V** → UNO Q 5V pin.
Why TPS61088: the UNO Q asks for a stable 5 V source rated ≥3 A; peaks that current-limit the supply cause resets. An MT3608 gives only ~0.7–1 A at 3 V in, and the XL6009 is rated for ≥5 V input, so neither suits a single cell.
Use a fixed-5 V TPS61088 board, not a QC 3.0 fast-charge version (those can switch to 9–12 V).
Motors run from the battery rail (TP4056 OUT+), not the UNO Q 3.3 V pin.
Check the boost output is 5.0–5.2 V with a multimeter **before** connecting the UNO Q.
