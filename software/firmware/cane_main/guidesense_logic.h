// guidesense_logic.h — hardware-independent signal processing for GuideSense.
// Pure C++ (no Arduino headers) so it can be unit-tested on a PC:
//   software/tests/test_logic.cpp
//
// Conventions
//   Angles in degrees. Pitch is the beam angle from horizontal: positive = up, negative = down.
//   Distances in centimetres.
#pragma once
#include <math.h>
#include <stdint.h>

namespace gs {

constexpr float DEG2RAD = 3.14159265f / 180.0f;

// ---------- Filters ----------
// Median of the last N samples (rejects single spikes from ultrasonic/LiDAR).
template <int N>
class MedianFilter {
  float buf_[N] = {0};
  int count_ = 0, head_ = 0;
 public:
  float push(float x) {
    buf_[head_] = x;
    head_ = (head_ + 1) % N;
    if (count_ < N) count_++;
    float tmp[N];
    for (int i = 0; i < count_; i++) tmp[i] = buf_[i];
    for (int i = 1; i < count_; i++) {          // insertion sort, N is small
      float v = tmp[i]; int j = i - 1;
      while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
      tmp[j + 1] = v;
    }
    return tmp[count_ / 2];
  }
};

// Exponential moving average, used for the slowly-updated ground reference.
struct Ema {
  float alpha, value; bool init = false;
  explicit Ema(float a) : alpha(a), value(0) {}
  float push(float x) { value = init ? value + alpha * (x - value) : x; init = true; return value; }
};

// Requires `need` consecutive true samples before reporting true (debounce, 3–5 readings).
struct Debounce {
  int need, run = 0;
  explicit Debounce(int n) : need(n) {}
  bool push(bool cond) { run = cond ? run + 1 : 0; return run >= need; }
  void reset() { run = 0; }
};

// ---------- Geometry / tilt compensation ----------
// Horizontal reach and vertical offset of the point the beam hits.
inline float horizontalCm(float d, float pitchDeg) { return d * cosf(pitchDeg * DEG2RAD); }
inline float verticalCm(float d, float pitchDeg)   { return d * sinf(pitchDeg * DEG2RAD); }

// Height above the ground of the point the forward LiDAR hit.
inline float hitHeightCm(float d, float beamPitchDeg, float sensorHeightCm) {
  return sensorHeightCm + verticalCm(d, beamPitchDeg);
}

// True when the current pitch is too far from the calibrated holding angle to correct (~25°).
inline bool tiltOutOfRange(float pitchDeg, float calibPitchDeg, float maxDevDeg = 25.0f) {
  return fabsf(pitchDeg - calibPitchDeg) > maxDevDeg;
}

// ---------- Hazard classification ----------
enum class Hazard : uint8_t { None = 0, Body = 1, HeadChest = 2, Drop = 3 };  // higher = higher priority
enum class Side : uint8_t { Left = 0, Centre = 1, Right = 2 };

struct Thresholds {
  float alertRangeCm   = 200.0f;  // forward obstacles beyond this are ignored
  float headChestMinCm = 100.0f;  // hit point above this height = head/chest hazard
  float groundMaxCm    = 25.0f;   // hit point below this = ground, the cane tip handles it
  float dropMarginCm   = 10.0f;   // depth below the ground reference that counts as a drop
};

// Forward LiDAR → Body / HeadChest / None
inline Hazard classifyForward(float d, float beamPitchDeg, float sensorHeightCm, const Thresholds& t) {
  if (d <= 0) return Hazard::None;
  if (horizontalCm(d, beamPitchDeg) > t.alertRangeCm) return Hazard::None;
  float h = hitHeightCm(d, beamPitchDeg, sensorHeightCm);
  if (h < t.groundMaxCm) return Hazard::None;
  return (h >= t.headChestMinCm) ? Hazard::HeadChest : Hazard::Body;
}

// Downward ultrasonic → depth of the surface below the sensor.
// A drop is a sudden jump of >= dropMarginCm beyond the running ground reference;
// gradual change is treated as a slope and absorbed by the EMA.
class DropDetector {
  Ema ground_;
  Debounce deb_;
  float margin_;
 public:
  DropDetector(float emaAlpha = 0.05f, int consecutive = 3, float marginCm = 10.0f)
      : ground_(emaAlpha), deb_(consecutive), margin_(marginCm) {}
  // d: slant distance (cm); beamPitchDeg: negative (pointing down)
  bool update(float d, float beamPitchDeg) {
    float depth = -verticalCm(d, beamPitchDeg);               // positive number, cm below sensor
    bool jump = ground_.init && (depth - ground_.value) >= margin_;
    bool drop = deb_.push(jump);
    if (!jump) ground_.push(depth);                           // don't learn the hole as "ground"
    return drop;
  }
  float groundRefCm() const { return ground_.value; }
};

// ---------- Direction from sweep ----------
// yawOffsetDeg: cane yaw relative to the running sweep centre (positive = left).
// halfSwingDeg: half of the calibrated swing width.
inline Side sideFromSweep(float yawOffsetDeg, float halfSwingDeg) {
  float third = halfSwingDeg / 3.0f;
  if (yawOffsetDeg > third)  return Side::Left;
  if (yawOffsetDeg < -third) return Side::Right;
  return Side::Centre;
}

// ---------- Haptic output ----------
struct Pulse { uint8_t duty; uint16_t periodMs; };   // PWM duty 0–255, on/off cycle period

// Closer = stronger and faster. Off beyond alertRange.
inline Pulse pulseForDistance(float horizCm, float alertRangeCm = 200.0f) {
  if (horizCm <= 0 || horizCm > alertRangeCm) return {0, 0};
  float k = 1.0f - horizCm / alertRangeCm;            // 0 (far) … 1 (touching)
  uint8_t duty = (uint8_t)(120 + k * 135);            // coin motors stall below ~120
  uint16_t period = (uint16_t)(700 - k * 600);        // 700 ms … 100 ms
  return {duty, period};
}

// ---------- Priority & crowd mode ----------
// Crowd mode: when many close Body hits are seen, suppress Body alerts and keep Drop/HeadChest.
class CrowdMode {
  int closeHits_ = 0; int window_; int trigger_;
  int n_ = 0; bool active_ = false;
 public:
  CrowdMode(int windowSamples = 40, int triggerHits = 20) : window_(windowSamples), trigger_(triggerHits) {}
  bool update(Hazard h, float horizCm) {
    if (h == Hazard::Body && horizCm < 120.0f) closeHits_++;
    if (++n_ >= window_) { active_ = closeHits_ >= trigger_; closeHits_ = 0; n_ = 0; }
    return active_;
  }
  bool active() const { return active_; }
};

inline Hazard prioritise(Hazard forward, bool drop, bool crowd) {
  if (drop) return Hazard::Drop;
  if (forward == Hazard::Body && crowd) return Hazard::None;
  return forward;
}

}  // namespace gs
