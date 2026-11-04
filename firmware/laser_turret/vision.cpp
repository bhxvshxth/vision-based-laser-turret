#include "vision.h"
#include "config.h"
#include "esp_heap_caps.h"

// Per-pixel class bits
static const uint8_t BIT_BALL  = 0x01;
static const uint8_t BIT_LASER = 0x02;

static uint8_t* s_mask  = nullptr;
static int32_t* s_queue = nullptr;
static int s_w = 0, s_h = 0;

static void* bigAlloc(size_t n) {
  void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return p ? p : malloc(n);
}

bool visionBegin(int width, int height) {
  free(s_mask);  free(s_queue);
  s_w = width; s_h = height;
  s_mask  = (uint8_t*)bigAlloc((size_t)s_w * s_h);
  s_queue = (int32_t*)bigAlloc((size_t)s_w * s_h * sizeof(int32_t));
  return s_mask && s_queue;
}

static inline uint16_t readPx(const uint8_t* p) {
#if RGB565_SWAP_BYTES
  return (uint16_t)(p[0] | (p[1] << 8));
#else
  return (uint16_t)((p[0] << 8) | p[1]);   // esp32-camera RGB565 is big-endian
#endif
}

static inline void writePx(uint8_t* p, uint16_t v) {
#if RGB565_SWAP_BYTES
  p[0] = v & 0xFF; p[1] = v >> 8;
#else
  p[0] = v >> 8;   p[1] = v & 0xFF;
#endif
}

static inline void toRGB(uint16_t v, int& r, int& g, int& b) {
  r = (v >> 8) & 0xF8; r |= r >> 5;
  g = (v >> 3) & 0xFC; g |= g >> 6;
  b = (v << 3) & 0xF8; b |= b >> 5;
}

static inline bool isBall(int r, int g, int b) {
  int mx = max(r, max(g, b));
  int mn = min(r, min(g, b));
  int d = mx - mn;
  if (mx < BALL_VAL_MIN || d == 0) return false;
  if ((d * 255) / mx < BALL_SAT_MIN) return false;
  int h;
  if (mx == r)      h = (60 * (g - b)) / d;
  else if (mx == g) h = 120 + (60 * (b - r)) / d;
  else              h = 240 + (60 * (r - g)) / d;
  if (h < 0) h += 360;
  return h >= BALL_HUE_MIN && h <= BALL_HUE_MAX;
}

static inline bool isLaser(int r, int g, int b) {
  return r >= LASER_MIN_R && (r - g) >= LASER_MIN_DIFF && (r - b) >= LASER_MIN_DIFF;
}

// Best 4-connected blob of `bit` with area in [minA, maxA].
// No prediction: the largest. With a prediction: the closest one within
// LASER_SEARCH_PX, so other red things can't be mistaken for the dot.
static Blob bestBlob(uint8_t bit, int minA, int maxA, const float* pred) {
  const uint8_t seen = bit << 4;
  const int n = s_w * s_h;
  Blob best;
  float bestScore = 1e30f;
  for (int i = 0; i < n; i++) {
    if (!(s_mask[i] & bit) || (s_mask[i] & seen)) continue;
    int head = 0, tail = 0;
    s_queue[tail++] = i;
    s_mask[i] |= seen;
    long sx = 0, sy = 0;
    int area = 0;
    while (head < tail) {
      int p = s_queue[head++];
      int x = p % s_w, y = p / s_w;
      sx += x; sy += y; area++;
      int nb[4] = { x > 0 ? p - 1 : -1, x < s_w - 1 ? p + 1 : -1,
                    y > 0 ? p - s_w : -1, y < s_h - 1 ? p + s_w : -1 };
      for (int k = 0; k < 4; k++) {
        int q = nb[k];
        if (q >= 0 && (s_mask[q] & bit) && !(s_mask[q] & seen)) {
          s_mask[q] |= seen;
          s_queue[tail++] = q;
        }
      }
    }
    if (area < minA || area > maxA) continue;
    float cx = (float)sx / area, cy = (float)sy / area;
    float score = -(float)area;
    if (pred && LASER_SEARCH_PX > 0) {
      score = hypotf(cx - pred[0], cy - pred[1]);
      if (score > LASER_SEARCH_PX) continue;
    }
    if (score < bestScore) {
      bestScore = score;
      best.found = true;
      best.area = area;
      best.x = cx;
      best.y = cy;
    }
  }
  return best;
}

// Fraction of pixels around the dot that are ball-coloured. (The dot itself
// is classed as laser, so this is really the ring around it.)
static bool dotOnBall(const Blob& dot) {
  const int R = ON_TARGET_RADIUS;
  int cx = (int)(dot.x + 0.5f), cy = (int)(dot.y + 0.5f);
  int hits = 0, total = 0;
  for (int y = cy - R; y <= cy + R; y++) {
    for (int x = cx - R; x <= cx + R; x++) {
      if (x < 0 || y < 0 || x >= s_w || y >= s_h) continue;
      total++;
      if (s_mask[y * s_w + x] & BIT_BALL) hits++;
    }
  }
  return total > 0 && hits >= total * ON_TARGET_FRAC;
}

Detections visionDetect(const camera_fb_t* fb, const float* laserPred) {
  Detections d;
  if (!fb || !s_mask || fb->format != PIXFORMAT_RGB565) return d;
  if ((int)fb->width != s_w || (int)fb->height != s_h) {
    if (!visionBegin(fb->width, fb->height)) return d;
  }
  const uint8_t* px = fb->buf;
  const int n = s_w * s_h;
  for (int i = 0; i < n; i++, px += 2) {
    int r, g, b;
    toRGB(readPx(px), r, g, b);
    // A pixel is either laser or ball, never both, so the dot can't merge into the target.
    s_mask[i] = isLaser(r, g, b) ? BIT_LASER : (isBall(r, g, b) ? BIT_BALL : 0);
  }
  d.ball  = bestBlob(BIT_BALL,  BALL_MIN_AREA,  BALL_MAX_AREA,  nullptr);
  d.laser = bestBlob(BIT_LASER, LASER_MIN_AREA, LASER_MAX_AREA, laserPred);
  d.dotOnTarget = d.ball.found && d.laser.found && dotOnBall(d.laser);
  return d;
}

static void cross(camera_fb_t* fb, const Blob& b, int size, uint16_t color) {
  if (!b.found) return;
  int cx = (int)(b.x + 0.5f), cy = (int)(b.y + 0.5f);
  for (int k = -size; k <= size; k++) {
    int pts[2][2] = { { cx + k, cy }, { cx, cy + k } };
    for (auto& p : pts) {
      if (p[0] < 0 || p[1] < 0 || p[0] >= (int)fb->width || p[1] >= (int)fb->height) continue;
      writePx(fb->buf + 2 * (p[1] * fb->width + p[0]), color);
    }
  }
}

void visionDrawOverlay(camera_fb_t* fb, const Detections& d) {
  cross(fb, d.ball,  8, 0x07E0);  // green = ball
  cross(fb, d.laser, 4, 0x07FF);  // cyan  = detected laser dot
}
