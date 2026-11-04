// Distance module mounted on the laser - it always points where the laser points.
// Supported (pick in config.h):
//   RANGE_HCSR04 - ultrasonic, measures echo time
//   RANGE_TFLUNA - TF-Luna / TFmini family, UART time-of-flight
#pragma once
#include <Arduino.h>

void  rangeBegin();
void  rangeUpdate();          // call often; never blocks
bool  rangeValid();
float rangeCm();              // median-filtered, NAN if no reading
const char* rangeName();
