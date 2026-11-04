// =====================================================================
//  Vision Based Laser Turret
//  ESP32-CAM + 2x SG90 (pan/tilt) + laser + rangefinder
//
//  The camera is fixed and watches the scene. Every frame we find:
//    - the tennis ball (yellow blob)  -> where we WANT the laser
//    - the red laser dot              -> where the laser IS
//  and steer the servos until the dot sits on the ball. The rangefinder
//  rides on the laser, so once locked it reads the distance to the ball.
//
//  Serial commands (115200):  c = calibrate   a = arm/disarm
//                             r = reset bias  x = clear calibration
//                             h = help
// =====================================================================
#include <Arduino.h>
#include "esp_camera.h"
#include "config.h"
#include "turret_state.h"
#include "servo_ledc.h"
#include "vision.h"
#include "calibration.h"
#include "rangefinder.h"
#if ENABLE_WEB_DEBUG
#include "web_debug.h"
#endif

TurretState turret;
Calibration cal;

static LedcServo panServo, tiltServo;

// ---------------------------------------------------------------------
//  Camera (AI-Thinker pinout)
// ---------------------------------------------------------------------
static bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = 5;   c.pin_d1 = 18;  c.pin_d2 = 19;  c.pin_d3 = 21;
  c.pin_d4 = 36;  c.pin_d5 = 39;  c.pin_d6 = 34;  c.pin_d7 = 35;
  c.pin_xclk     = 0;
  c.pin_pclk     = 22;
  c.pin_vsync    = 25;
  c.pin_href     = 23;
  c.pin_sccb_sda = 26;
  c.pin_sccb_scl = 27;
  c.pin_pwdn     = 32;
  c.pin_reset    = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_RGB565;   // raw pixels - no JPEG decode needed
  c.frame_size   = CAM_FRAMESIZE;
  c.jpeg_quality = 12;
  c.fb_count     = 2;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("[cam] init failed: 0x%x\n", err);
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  s->set_vflip(s, CAM_VFLIP);
  s->set_hmirror(s, CAM_HMIRROR);
  s->set_saturation(s, CAM_SATURATION);
  return true;
}

// ---------------------------------------------------------------------
//  Actuators
// ---------------------------------------------------------------------
static void setLaser(bool on) {
  turret.laserOn = on;
  digitalWrite(PIN_LASER, on ? HIGH : LOW);
}

static void moveServos(float pan, float tilt) {
  pan  = constrain(pan,  PAN_MIN_DEG,  PAN_MAX_DEG);
  tilt = constrain(tilt, TILT_MIN_DEG, TILT_MAX_DEG);
  // Tiny nudges from a wobbly target centre don't count as moving.
  if (fabsf(pan - turret.pan) > SETTLE_MOVE_DEG || fabsf(tilt - turret.tilt) > SETTLE_MOVE_DEG) turret.lastMoveMs = millis();
  turret.pan = pan;
  turret.tilt = tilt;
  panServo.writeDeg(pan);
  tiltServo.writeDeg(tilt);
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---------------------------------------------------------------------
//  Calibration: sweep a 3x3 grid, find the dot each time, fit the map.
//  Do this with the laser pointing at the surface the ball will be on.
// ---------------------------------------------------------------------
static bool findDot(Blob& out) {
  for (int i = 0; i < 2; i++) {               // drop stale frames
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
  }
  float sx = 0, sy = 0;
  int hits = 0;
  for (int i = 0; i < 3; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) continue;
    Detections d = visionDetect(fb);
    esp_camera_fb_return(fb);
    if (d.laser.found) { sx += d.laser.x; sy += d.laser.y; hits++; }
  }
  if (hits < 2) return false;
  out.found = true;
  out.x = sx / hits;
  out.y = sy / hits;
  return true;
}

static void runCalibration() {
  if (!turret.armed) {
    Serial.println("[cal] turret is disarmed - arm it first (the laser has to be on)");
    return;
  }
  Serial.println("[cal] sweeping 3x3 grid...");
  float P[9], T[9], X[9], Y[9];
  int n = 0;
  setLaser(true);
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      float p = PAN_HOME_DEG + i * CAL_SPAN_DEG;
      float t = TILT_HOME_DEG + j * CAL_SPAN_DEG;
      moveServos(p, t);
      delay(CAL_SETTLE_MS);
      Blob dot;
      if (findDot(dot)) {
        P[n] = turret.pan; T[n] = turret.tilt; X[n] = dot.x; Y[n] = dot.y; n++;
        Serial.printf("[cal]  pan %.0f tilt %.0f -> dot (%.1f, %.1f)\n", turret.pan, turret.tilt, dot.x, dot.y);
      } else {
        Serial.printf("[cal]  pan %.0f tilt %.0f -> dot not seen\n", turret.pan, turret.tilt);
      }
    }
  }
  setLaser(false);
  Calibration c;
  if (fitCalibration(P, T, X, Y, n, c)) {
    cal = c;
    cal.save();
    turret.biasPan = turret.biasTilt = 0;
    Serial.printf("[cal] done with %d points, saved.\n", n);
    Serial.printf("[cal]  px = %.3f*pan + %.3f*tilt + %.1f\n", cal.a[0], cal.a[1], cal.a[2]);
    Serial.printf("[cal]  py = %.3f*pan + %.3f*tilt + %.1f\n", cal.b[0], cal.b[1], cal.b[2]);
  } else {
    Serial.printf("[cal] FAILED (%d usable points, need 4). Check laser thresholds / lighting.\n", n);
  }
  moveServos(PAN_HOME_DEG, TILT_HOME_DEG);
  turret.lastBallMs = 0;
}

// ---------------------------------------------------------------------
//  Tracking
// ---------------------------------------------------------------------
// Where the dot should be right now. The bias is the learned map error, so the
// dot lands where the map says (aim - bias) would.
static bool predictDot(float pred[2]) {
  if (!turret.laserOn) return false;
  cal.anglesToPixel(turret.pan - turret.biasPan, turret.tilt - turret.biasTilt, pred[0], pred[1]);
  return true;
}

static void controlStep(const Detections& d, uint32_t now) {
  turret.locked = false;

  if (!d.ball.found) {
    turret.errPx = -1;
    if (now - turret.lastBallMs > LOST_TIMEOUT_MS) { setLaser(false); turret.targetX = -1; }
    return;
  }
  turret.lastBallMs = now;

  // Smooth the ball centre - big or ragged blobs wobble from frame to frame.
  // A big jump (ball really moved) resets the filter.
  if (turret.targetX < 0 || hypotf(d.ball.x - turret.targetX, d.ball.y - turret.targetY) > 8) {
    turret.targetX = d.ball.x; turret.targetY = d.ball.y;
  } else {
    turret.targetX += TARGET_SMOOTH * (d.ball.x - turret.targetX);
    turret.targetY += TARGET_SMOOTH * (d.ball.y - turret.targetY);
  }

  // Dot already on the ball's colour: hold still. Chasing the exact centre of
  // a big target just makes the dot jitter. If the ball moves off the dot,
  // dotOnTarget goes false and tracking resumes.
  if (d.dotOnTarget && turret.laserOn) {
    turret.locked = true;
    turret.errPx = 0;
    return;
  }

  // 1) Feed-forward: where the map says the ball is in servo angles.
  float ffPan, ffTilt;
  if (!cal.pixelToAngles(turret.targetX, turret.targetY, ffPan, ffTilt)) return;
  bool aimed = fabsf(ffPan + turret.biasPan - turret.pan) < 1.0f &&
               fabsf(ffTilt + turret.biasTilt - turret.tilt) < 1.0f;

  // 2) Feedback: nudge the bias so the dot lands on the ball. Only once we're
  //    aimed where the ball is now and the servos have settled - otherwise the
  //    gap is just "not there yet" (e.g. the ball jumped), not map error.
  if (d.laser.found && turret.laserOn) {
    float ex = turret.targetX - d.laser.x, ey = turret.targetY - d.laser.y;
    turret.errPx = sqrtf(ex * ex + ey * ey);
    turret.locked = turret.errPx <= LOCK_PX;
    if (aimed && now - turret.lastMoveMs >= SETTLE_MS && turret.errPx > DEADBAND_PX) {
      float dp, dt;
      if (cal.pixelDeltaToAngles(ex, ey, dp, dt)) {
        turret.biasPan  = clampf(turret.biasPan  + BIAS_GAIN * dp, -BIAS_LIMIT_DEG, BIAS_LIMIT_DEG);
        turret.biasTilt = clampf(turret.biasTilt + BIAS_GAIN * dt, -BIAS_LIMIT_DEG, BIAS_LIMIT_DEG);
      }
    }
  } else {
    turret.errPx = -1;
  }

  // 3) Move toward the target, rate limited.
  float tp = ffPan + turret.biasPan, tt = ffTilt + turret.biasTilt;
  moveServos(turret.pan  + clampf(tp - turret.pan,  -MAX_STEP_DEG, MAX_STEP_DEG),
             turret.tilt + clampf(tt - turret.tilt, -MAX_STEP_DEG, MAX_STEP_DEG));
  setLaser(turret.armed);
}

// ---------------------------------------------------------------------
//  Serial
// ---------------------------------------------------------------------
static void printHelp() {
  Serial.println("commands: c=calibrate  a=arm/disarm  r=reset bias  x=clear calibration  h=help");
}

static void handleSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case 'c': turret.reqCalibrate = true; break;
      case 'a': turret.armed = !turret.armed;
                Serial.println(turret.armed ? "[turret] ARMED" : "[turret] disarmed");
                if (!turret.armed) setLaser(false);
                break;
      case 'r': turret.reqResetBias = true; break;
      case 'x': turret.reqClearCal = true; break;
      case 'h': printHelp(); break;
      default: break;
    }
  }
}

static void printStatus() {
  const Detections& d = turret.det;
  Serial.printf("ball %s", d.ball.found ? "" : "--");
  if (d.ball.found) Serial.printf("(%5.1f,%5.1f)", d.ball.x, d.ball.y);
  Serial.printf("  dot %s", d.laser.found ? "" : "--");
  if (d.laser.found) Serial.printf("(%5.1f,%5.1f)", d.laser.x, d.laser.y);
  Serial.printf("  pan %5.1f tilt %5.1f", turret.pan, turret.tilt);
  if (rangeValid()) Serial.printf("  range %6.1f cm", rangeCm());
  else              Serial.print("  range   --  ");
  Serial.printf("  %s%s  %.0ffps\n", turret.laserOn ? "LASER " : "", turret.locked ? "LOCKED" : "", turret.fps);
}

// ---------------------------------------------------------------------
void setup() {
  // Laser off before anything else.
  pinMode(PIN_LASER, OUTPUT);
  digitalWrite(PIN_LASER, LOW);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, HIGH);   // off (active low)

  Serial.begin(SERIAL_BAUD);
  delay(300);
  Serial.println("\n=== vision based laser turret ===");

  if (!psramFound()) Serial.println("[warn] no PSRAM found - enable it in Tools > PSRAM");

  if (!initCamera()) {
    Serial.println("[fatal] camera failed, halting");
    while (true) delay(1000);
  }

  // Read one frame to learn the real resolution.
  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) { turret.frameW = fb->width; turret.frameH = fb->height; esp_camera_fb_return(fb); }
  if (!visionBegin(turret.frameW, turret.frameH)) Serial.println("[warn] vision buffers failed to allocate");
  Serial.printf("[cam] %dx%d RGB565\n", turret.frameW, turret.frameH);

  panServo.begin(PIN_SERVO_PAN, LEDC_CHANNEL_2);
  tiltServo.begin(PIN_SERVO_TILT, LEDC_CHANNEL_3);
  turret.pan = turret.tilt = -1000;      // force first write
  moveServos(PAN_HOME_DEG, TILT_HOME_DEG);

  if (cal.load()) Serial.println("[cal] loaded saved calibration");
  else {
    cal.setDefault(turret.frameW, turret.frameH);
    Serial.println("[cal] no calibration yet - using a rough default. Send 'c' to calibrate.");
  }

  rangeBegin();
  Serial.printf("[range] sensor: %s\n", rangeName());

  turret.armed = START_ARMED;
  Serial.println(turret.armed ? "[turret] ARMED" : "[turret] disarmed - send 'a' to arm");

#if ENABLE_WEB_DEBUG
  webDebugBegin();
#endif
  printHelp();
}

void loop() {
  static uint32_t lastPrint = 0, lastFrame = 0;

  handleSerial();
  if (turret.reqCalibrate) { turret.reqCalibrate = false; runCalibration(); }
  if (turret.reqResetBias) { turret.reqResetBias = false; turret.biasPan = turret.biasTilt = 0; Serial.println("[turret] bias reset"); }
  if (turret.reqClearCal) {
    turret.reqClearCal = false;
    cal.clearSaved();
    cal.setDefault(turret.frameW, turret.frameH);
    turret.biasPan = turret.biasTilt = 0;
    Serial.println("[cal] cleared - back to default map");
  }
  if (!turret.armed && turret.laserOn) setLaser(false);

  rangeUpdate();

  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) {
    float pred[2];
    turret.det = visionDetect(fb, predictDot(pred) ? pred : nullptr);
    esp_camera_fb_return(fb);
    uint32_t now = millis();
    controlStep(turret.det, now);
    if (lastFrame) turret.fps = 0.9f * turret.fps + 0.1f * (1000.0f / max<uint32_t>(1, now - lastFrame));
    lastFrame = now;
  }

  rangeUpdate();
  digitalWrite(PIN_STATUS_LED, turret.locked ? LOW : HIGH);

#if ENABLE_WEB_DEBUG
  webDebugLoop();
#endif

  if (millis() - lastPrint >= STATUS_PRINT_MS) {
    lastPrint = millis();
    printStatus();
  }
}
