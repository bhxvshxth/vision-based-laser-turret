// Ball + laser-dot detection on raw RGB565 camera frames.
#pragma once
#include <Arduino.h>
#include "esp_camera.h"

struct Blob {
  bool  found = false;
  float x = 0, y = 0;   // centroid in pixels
  int   area = 0;
};

struct Detections {
  Blob ball;
  Blob laser;
  bool dotOnTarget = false;   // laser dot is sitting on the ball's colour
};

bool       visionBegin(int width, int height);
// laserPred: where the dot is expected {x, y}, or nullptr to take the largest red blob.
Detections visionDetect(const camera_fb_t* fb, const float* laserPred = nullptr);
// Draws crosshairs on the frame (for the debug snapshot only).
void       visionDrawOverlay(camera_fb_t* fb, const Detections& d);
