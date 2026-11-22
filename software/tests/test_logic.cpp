// Host-side tests for guidesense_logic.h
//   g++ -std=c++17 -I../firmware/cane_main test_logic.cpp -o test_logic && ./test_logic
#include <cstdio>
#include <cmath>
#include <initializer_list>
#include "guidesense_logic.h"
using namespace gs;

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

int main() {
  // Median filter rejects one spike
  MedianFilter<5> m;
  float out = 0;
  for (float x : {100.f, 101.f, 400.f, 99.f, 100.f}) out = m.push(x);
  CHECK(fabsf(out - 100.f) < 0.01f);

  // Geometry
  CHECK(fabsf(horizontalCm(200, 0) - 200) < 0.01f);
  CHECK(fabsf(verticalCm(200, 30) - 100) < 0.1f);

  // Forward classification: LiDAR at 90 cm height, beam tilted 15° up
  Thresholds t;
  CHECK(classifyForward(150, 15, 90, t) == Hazard::HeadChest);  // hit ~129 cm high
  CHECK(classifyForward(150, -5, 90, t) == Hazard::Body);       // hit ~77 cm high
  CHECK(classifyForward(500, 0, 90, t) == Hazard::None);        // beyond alert range
  CHECK(classifyForward(150, -35, 90, t) == Hazard::None);      // hit ~4 cm: ground

  // Drop detection: sensor 30 cm above ground, beam 45° down → slant ≈ 42.4 cm
  DropDetector dd(0.1f, 3, 10.0f);
  bool d = false;
  for (int i = 0; i < 20; i++) d = dd.update(42.4f, -45);
  CHECK(!d);
  for (int i = 0; i < 2; i++) d = dd.update(80.0f, -45);        // 2 readings: not yet
  CHECK(!d);
  d = dd.update(80.0f, -45);                                    // 3rd consecutive → drop
  CHECK(d);

  // Slope: depth rises 0.2 cm per sample → never a drop
  DropDetector slope(0.1f, 3, 10.0f);
  bool any = false;
  for (int i = 0; i < 100; i++) any |= slope.update((30.0f + 0.2f * i) / sinf(45 * DEG2RAD), -45);
  CHECK(!any);

  // Direction
  CHECK(sideFromSweep(20, 30) == Side::Left);
  CHECK(sideFromSweep(0, 30) == Side::Centre);
  CHECK(sideFromSweep(-20, 30) == Side::Right);

  // Haptics: closer = stronger and faster
  Pulse far = pulseForDistance(180), near = pulseForDistance(30);
  CHECK(near.duty > far.duty);
  CHECK(near.periodMs < far.periodMs);
  CHECK(pulseForDistance(250).duty == 0);

  // Priority + crowd mode
  CHECK(prioritise(Hazard::Body, true, false) == Hazard::Drop);
  CHECK(prioritise(Hazard::Body, false, true) == Hazard::None);
  CHECK(prioritise(Hazard::HeadChest, false, true) == Hazard::HeadChest);
  CrowdMode cm(10, 5);
  bool c = false;
  for (int i = 0; i < 10; i++) c = cm.update(Hazard::Body, 80);
  CHECK(c);

  // Tilt range gate
  CHECK(tiltOutOfRange(40, 10));
  CHECK(!tiltOutOfRange(20, 10));

  printf(fails ? "%d test(s) FAILED\n" : "All tests passed\n", fails);
  return fails ? 1 : 0;
}
