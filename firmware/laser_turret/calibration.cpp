#include "calibration.h"
#include "config.h"
#include <Preferences.h>

void Calibration::setDefault(int width, int height) {
  float k = 1.0f / DEFAULT_DEG_PER_PX;
  a[0] = PAN_SIGN * k; a[1] = 0;               a[2] = width  / 2.0f - a[0] * PAN_HOME_DEG;
  b[0] = 0;            b[1] = TILT_SIGN * k;   b[2] = height / 2.0f - b[1] * TILT_HOME_DEG;
  fromSweep = false;
}

bool Calibration::pixelDeltaToAngles(float dx, float dy, float& dpan, float& dtilt) const {
  float det = a[0] * b[1] - a[1] * b[0];
  if (fabsf(det) < 1e-6f) return false;
  dpan  = ( b[1] * dx - a[1] * dy) / det;
  dtilt = (-b[0] * dx + a[0] * dy) / det;
  return true;
}

void Calibration::anglesToPixel(float pan, float tilt, float& px, float& py) const {
  px = a[0] * pan + a[1] * tilt + a[2];
  py = b[0] * pan + b[1] * tilt + b[2];
}

bool Calibration::pixelToAngles(float px, float py, float& pan, float& tilt) const {
  return pixelDeltaToAngles(px - a[2], py - b[2], pan, tilt);
}

bool Calibration::load() {
  Preferences p;
  if (!p.begin("turret", true)) return false;
  bool ok = p.getBytesLength("cal") == sizeof(float) * 6;
  if (ok) {
    float v[6];
    p.getBytes("cal", v, sizeof(v));
    memcpy(a, v, sizeof(a));
    memcpy(b, v + 3, sizeof(b));
    fromSweep = true;
  }
  p.end();
  return ok;
}

void Calibration::save() const {
  Preferences p;
  p.begin("turret", false);
  float v[6] = { a[0], a[1], a[2], b[0], b[1], b[2] };
  p.putBytes("cal", v, sizeof(v));
  p.end();
}

void Calibration::clearSaved() const {
  Preferences p;
  p.begin("turret", false);
  p.remove("cal");
  p.end();
}

static float det3(const float m[3][3]) {
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
       - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
       + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

// Solve M x = r (3x3) with Cramer's rule.
static bool solve3(const float M[3][3], const float r[3], float x[3]) {
  float d = det3(M);
  if (fabsf(d) < 1e-9f) return false;
  for (int c = 0; c < 3; c++) {
    float t[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) t[i][j] = (j == c) ? r[i] : M[i][j];
    x[c] = det3(t) / d;
  }
  return true;
}

bool fitCalibration(const float* pan, const float* tilt,
                    const float* px, const float* py, int n, Calibration& out) {
  if (n < 4) return false;
  float M[3][3] = {{0}}, rx[3] = {0}, ry[3] = {0};
  for (int i = 0; i < n; i++) {
    float v[3] = { pan[i], tilt[i], 1.0f };
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) M[r][c] += v[r] * v[c];
      rx[r] += v[r] * px[i];
      ry[r] += v[r] * py[i];
    }
  }
  Calibration c;
  if (!solve3(M, rx, c.a) || !solve3(M, ry, c.b)) return false;
  float dummy1, dummy2;
  if (!c.pixelDeltaToAngles(1, 1, dummy1, dummy2)) return false;  // degenerate map
  c.fromSweep = true;
  out = c;
  return true;
}
