#include "servo_ledc.h"
#include "config.h"

static const ledc_mode_t     SERVO_MODE  = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t    SERVO_TIMER = LEDC_TIMER_1;
static const uint32_t        SERVO_HZ    = 50;            // 20 ms period
static const uint32_t        DUTY_MAX    = (1u << 16) - 1; // 16-bit resolution
static bool timerReady = false;

void LedcServo::begin(int pin, ledc_channel_t channel) {
  _ch = channel;
  if (!timerReady) {
    ledc_timer_config_t t = {};
    t.speed_mode      = SERVO_MODE;
    t.duty_resolution = LEDC_TIMER_16_BIT;
    t.timer_num       = SERVO_TIMER;
    t.freq_hz         = SERVO_HZ;
    t.clk_cfg         = LEDC_AUTO_CLK;
    timerReady = (ledc_timer_config(&t) == ESP_OK);
  }
  ledc_channel_config_t c = {};
  c.gpio_num   = pin;
  c.speed_mode = SERVO_MODE;
  c.channel    = _ch;
  c.intr_type  = LEDC_INTR_DISABLE;
  c.timer_sel  = SERVO_TIMER;
  c.duty       = 0;
  c.hpoint     = 0;
  _ok = timerReady && (ledc_channel_config(&c) == ESP_OK);
  if (!_ok) Serial.printf("[servo] failed to set up pin %d\n", pin);
}

void LedcServo::writeDeg(float deg) {
  if (!_ok) return;
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (deg / 180.0f);
  uint32_t duty = (uint32_t)(us * DUTY_MAX / 20000.0f);
  ledc_set_duty(SERVO_MODE, _ch, duty);
  ledc_update_duty(SERVO_MODE, _ch);
}
