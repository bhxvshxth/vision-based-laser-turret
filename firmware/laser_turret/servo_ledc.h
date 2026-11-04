// Minimal servo driver on the ESP32 LEDC peripheral.
// The camera uses LEDC timer 0 / channel 0 for its clock, so the servos
// use timer 1 and channels 2-3 to stay out of its way. (Generic servo
// libraries often grab timer 0 and break the camera.)
#pragma once
#include <Arduino.h>
#include "driver/ledc.h"

class LedcServo {
public:
  void begin(int pin, ledc_channel_t channel);
  void writeDeg(float deg);
private:
  ledc_channel_t _ch = LEDC_CHANNEL_2;
  bool _ok = false;
};
