// Pixel <-> servo-angle mapping for a FIXED camera.
//   px = a0*pan + a1*tilt + a2
//   py = b0*pan + b1*tilt + b2
// Learned by sweeping the laser over a 3x3 grid and finding the dot.
#pragma once
#include <Arduino.h>

struct Calibration {
  float a[3] = {0, 0, 0};
  float b[3] = {0, 0, 0};
  bool  fromSweep = false;   // false = rough default estimate

  void setDefault(int width, int height);
  bool pixelToAngles(float px, float py, float& pan, float& tilt) const;
  bool pixelDeltaToAngles(float dx, float dy, float& dpan, float& dtilt) const;
  void anglesToPixel(float pan, float tilt, float& px, float& py) const;
  bool load();
  void save() const;
  void clearSaved() const;
};

// Least-squares fit from n >= 4 (pan, tilt) -> (px, py) samples.
bool fitCalibration(const float* pan, const float* tilt,
                    const float* px, const float* py, int n, Calibration& out);
