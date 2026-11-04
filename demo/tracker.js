// Same detection + control logic as the firmware (vision.cpp / laser_turret.ino),
// ported to JS so the demo behaves like the real turret.
(function (root) {
  const CFG = {
    BALL_HUE_MIN: 35, BALL_HUE_MAX: 85, BALL_SAT_MIN: 90, BALL_VAL_MIN: 80,
    BALL_MIN_AREA: 20, BALL_MAX_AREA: 6000,
    LASER_MIN_R: 230, LASER_MIN_DIFF: 80, LASER_MIN_AREA: 1, LASER_MAX_AREA: 120, LASER_SEARCH_PX: 30, ON_TARGET_RADIUS: 3, ON_TARGET_FRAC: 0.4,
    PAN_MIN: 30, PAN_MAX: 150, TILT_MIN: 45, TILT_MAX: 135, PAN_HOME: 90, TILT_HOME: 90,
    MAX_STEP_DEG: 4, DEFAULT_DEG_PER_PX: 0.4,
    BIAS_GAIN: 0.5, BIAS_LIMIT: 15, DEADBAND_PX: 1.5, LOCK_PX: 3,
    SETTLE_MS: 150, SETTLE_MOVE_DEG: 0.6, TARGET_SMOOTH: 0.5, LOST_TIMEOUT_MS: 600,
  };

  const BIT_BALL = 1, BIT_LASER = 2;
  let mask = null, queue = null;

  // Default target = tennis ball, same as the firmware.
  const DEFAULT_TARGET = { hMin: CFG.BALL_HUE_MIN, hMax: CFG.BALL_HUE_MAX, sMin: CFG.BALL_SAT_MIN, vMin: CFG.BALL_VAL_MIN };

  function hsv(r, g, b) {
    const mx = Math.max(r, g, b), mn = Math.min(r, g, b), d = mx - mn;
    if (mx === 0 || d === 0) return [0, 0, mx];
    let h;
    if (mx === r) h = (60 * (g - b)) / d | 0;
    else if (mx === g) h = 120 + ((60 * (b - r)) / d | 0);
    else h = 240 + ((60 * (r - g)) / d | 0);
    if (h < 0) h += 360;
    return [h, (d * 255) / mx | 0, mx];
  }

  // hMin > hMax means the range wraps through 0 (reds).
  function isTarget(r, g, b, t) {
    const [h, s, v] = hsv(r, g, b);
    if (v < t.vMin || s < t.sMin) return false;
    return t.hMin <= t.hMax ? h >= t.hMin && h <= t.hMax : h >= t.hMin || h <= t.hMax;
  }

  function isLaser(r, g, b) {
    return r >= CFG.LASER_MIN_R && r - g >= CFG.LASER_MIN_DIFF && r - b >= CFG.LASER_MIN_DIFF;
  }

  // Best 4-connected blob of `bit` with area in [minA, maxA].
  // No prediction: the largest one. With a prediction {x, y}: the one closest
  // to it, within CFG.LASER_SEARCH_PX - so other red things can't be mistaken
  // for the laser dot.
  function bestBlob(w, h, bit, minA, maxA, pred) {
    let bestScore = Infinity;
    const seen = bit << 4, n = w * h;
    let best = { found: false, x: 0, y: 0, area: 0 };
    for (let i = 0; i < n; i++) {
      if (!(mask[i] & bit) || (mask[i] & seen)) continue;
      let head = 0, tail = 0, sx = 0, sy = 0, area = 0;
      queue[tail++] = i; mask[i] |= seen;
      while (head < tail) {
        const p = queue[head++], x = p % w, y = (p / w) | 0;
        sx += x; sy += y; area++;
        const nb = [x > 0 ? p - 1 : -1, x < w - 1 ? p + 1 : -1, y > 0 ? p - w : -1, y < h - 1 ? p + w : -1];
        for (const q of nb) {
          if (q >= 0 && (mask[q] & bit) && !(mask[q] & seen)) { mask[q] |= seen; queue[tail++] = q; }
        }
      }
      if (area < minA || area > maxA) continue;
      const cx = sx / area, cy = sy / area;
      let score = -area;
      if (pred) {
        score = Math.hypot(cx - pred.x, cy - pred.y);
        if (score > CFG.LASER_SEARCH_PX) continue;
      }
      if (score < bestScore) { bestScore = score; best = { found: true, x: cx, y: cy, area }; }
    }
    return best;
  }

  // rgba: Uint8ClampedArray / Uint8Array of w*h*4
  function detect(rgba, w, h, target = DEFAULT_TARGET, laserPred = null) {
    const n = w * h;
    if (!mask || mask.length !== n) { mask = new Uint8Array(n); queue = new Int32Array(n); }
    for (let i = 0, j = 0; i < n; i++, j += 4) {
      const r = rgba[j], g = rgba[j + 1], b = rgba[j + 2];
      mask[i] = isLaser(r, g, b) ? BIT_LASER : isTarget(r, g, b, target) ? BIT_BALL : 0;
    }
    const ball = bestBlob(w, h, BIT_BALL, CFG.BALL_MIN_AREA, CFG.BALL_MAX_AREA, null);
    const laser = bestBlob(w, h, BIT_LASER, CFG.LASER_MIN_AREA, CFG.LASER_MAX_AREA, laserPred);
    return { ball, laser, dotOnTarget: ball.found && laser.found && onTarget(w, h, laser) };
  }

  // Is the dot sitting on the target's colour? Look at the pixels around the
  // dot (the dot itself is classed as laser, so check the ring around it).
  function onTarget(w, h, laser) {
    const R = CFG.ON_TARGET_RADIUS, cx = Math.round(laser.x), cy = Math.round(laser.y);
    let hits = 0, total = 0;
    for (let y = cy - R; y <= cy + R; y++) {
      for (let x = cx - R; x <= cx + R; x++) {
        if (x < 0 || y < 0 || x >= w || y >= h) continue;
        total++;
        if (mask[y * w + x] & BIT_BALL) hits++;
      }
    }
    return total > 0 && hits >= total * CFG.ON_TARGET_FRAC;
  }

  class Calibration {
    constructor(w, h) {
      const k = 1 / CFG.DEFAULT_DEG_PER_PX;
      this.a = [k, 0, w / 2 - k * CFG.PAN_HOME];
      this.b = [0, k, h / 2 - k * CFG.TILT_HOME];
    }
    deltaToAngles(dx, dy) {
      const [a0, a1] = this.a, [b0, b1] = this.b, det = a0 * b1 - a1 * b0;
      if (Math.abs(det) < 1e-6) return null;
      return [(b1 * dx - a1 * dy) / det, (-b0 * dx + a0 * dy) / det];
    }
    pixelToAngles(px, py) { return this.deltaToAngles(px - this.a[2], py - this.b[2]); }
    anglesToPixel(pan, tilt) {
      return { x: this.a[0] * pan + this.a[1] * tilt + this.a[2], y: this.b[0] * pan + this.b[1] * tilt + this.b[2] };
    }
  }

  const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

  class Controller {
    constructor(w, h) {
      this.cal = new Calibration(w, h);
      this.pan = CFG.PAN_HOME; this.tilt = CFG.TILT_HOME;
      this.biasPan = 0; this.biasTilt = 0;
      this.laserOn = false; this.locked = false; this.errPx = -1;
      this.lastBallMs = -1e9; this.lastMoveMs = 0;
      this.armed = true;
      this.tx = null; this.ty = null;   // smoothed target position
    }
    // Where the dot should be right now. The bias is the learned map error, so
    // the dot sits where the map says (aim - bias) would land.
    predictDot() {
      if (!this.laserOn) return null;
      return this.cal.anglesToPixel(this.pan - this.biasPan, this.tilt - this.biasTilt);
    }
    resetBias() { this.biasPan = 0; this.biasTilt = 0; }
    move(p, t, now) {
      p = clamp(p, CFG.PAN_MIN, CFG.PAN_MAX); t = clamp(t, CFG.TILT_MIN, CFG.TILT_MAX);
      // Tiny nudges from a wobbly target centre don't count as moving.
      if (Math.abs(p - this.pan) > CFG.SETTLE_MOVE_DEG || Math.abs(t - this.tilt) > CFG.SETTLE_MOVE_DEG) this.lastMoveMs = now;
      this.pan = p; this.tilt = t;
    }
    step(d, now) {
      this.locked = false;
      if (!d.ball.found) {
        this.errPx = -1;
        if (now - this.lastBallMs > CFG.LOST_TIMEOUT_MS) { this.laserOn = false; this.tx = null; }
        return;
      }
      this.lastBallMs = now;
      // Smooth the target centre - big or ragged blobs wobble frame to frame.
      if (this.tx === null || Math.hypot(d.ball.x - this.tx, d.ball.y - this.ty) > 8) { this.tx = d.ball.x; this.ty = d.ball.y; }
      else { const k = CFG.TARGET_SMOOTH; this.tx += k * (d.ball.x - this.tx); this.ty += k * (d.ball.y - this.ty); }
      // Dot already on the target's colour: hold still. Big targets have a
      // noisy centre, and chasing it just makes the dot jitter around.
      if (d.dotOnTarget && this.laserOn) {
        this.locked = true;
        this.errPx = 0;
        return;
      }
      const ff = this.cal.pixelToAngles(this.tx, this.ty);
      if (!ff) return;
      const tp0 = ff[0] + this.biasPan, tt0 = ff[1] + this.biasTilt;
      // Only learn from the dot once we're already aimed where the target is now
      // and the servos have settled. Otherwise (e.g. the target just jumped) the
      // gap is just "haven't got there yet", not map error.
      const aimed = Math.abs(tp0 - this.pan) < 1 && Math.abs(tt0 - this.tilt) < 1;
      if (d.laser.found && this.laserOn) {
        const ex = this.tx - d.laser.x, ey = this.ty - d.laser.y;
        this.errPx = Math.hypot(ex, ey);
        this.locked = this.errPx <= CFG.LOCK_PX;
        if (aimed && now - this.lastMoveMs >= CFG.SETTLE_MS && this.errPx > CFG.DEADBAND_PX) {
          const da = this.cal.deltaToAngles(ex, ey);
          if (da) {
            this.biasPan = clamp(this.biasPan + CFG.BIAS_GAIN * da[0], -CFG.BIAS_LIMIT, CFG.BIAS_LIMIT);
            this.biasTilt = clamp(this.biasTilt + CFG.BIAS_GAIN * da[1], -CFG.BIAS_LIMIT, CFG.BIAS_LIMIT);
          }
        }
      } else {
        this.errPx = -1;
      }
      const tp = ff[0] + this.biasPan, tt = ff[1] + this.biasTilt;   // bias may have just changed
      this.move(this.pan + clamp(tp - this.pan, -CFG.MAX_STEP_DEG, CFG.MAX_STEP_DEG),
                this.tilt + clamp(tt - this.tilt, -CFG.MAX_STEP_DEG, CFG.MAX_STEP_DEG), now);
      this.laserOn = this.armed;
    }
  }

  const api = { CFG, DEFAULT_TARGET, hsv, detect, Calibration, Controller };
  root.Tracker = api;
  if (typeof module !== 'undefined') module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
