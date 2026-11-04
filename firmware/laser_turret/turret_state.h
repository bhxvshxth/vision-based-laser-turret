// Shared state between the control loop and the debug web page.
#pragma once
#include "vision.h"
#include "calibration.h"

struct TurretState {
  float pan  = 90, tilt = 90;          // commanded angles (deg)
  float biasPan = 0, biasTilt = 0;     // learned correction (deg)
  bool  armed = false;
  bool  laserOn = false;
  bool  locked = false;                // dot is on the ball
  float errPx = -1;                    // ball-to-dot distance, -1 if unknown
  float fps = 0;
  int   frameW = 160, frameH = 120;
  Detections det;
  float targetX = -1, targetY = -1;   // smoothed ball centre (px), -1 = none
  uint32_t lastBallMs = 0;
  uint32_t lastMoveMs = 0;

  // requests from serial / web, handled in loop()
  volatile bool reqCalibrate = false;
  volatile bool reqResetBias = false;
  volatile bool reqClearCal  = false;
};

extern TurretState turret;
extern Calibration cal;
