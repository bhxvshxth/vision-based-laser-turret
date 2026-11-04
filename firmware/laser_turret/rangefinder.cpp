#include "rangefinder.h"
#include "config.h"

static float    s_cm = NAN;
static uint32_t s_lastMs = 0;
static float    s_hist[5];
static int      s_histN = 0, s_histI = 0;

// Median of the last 5 readings - kills single bad echoes.
static void __attribute__((unused)) pushReading(float cm) {
  s_hist[s_histI] = cm;
  s_histI = (s_histI + 1) % 5;
  if (s_histN < 5) s_histN++;
  float t[5];
  memcpy(t, s_hist, sizeof(float) * s_histN);
  for (int i = 1; i < s_histN; i++) {
    float v = t[i]; int j = i - 1;
    while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
    t[j + 1] = v;
  }
  s_cm = t[s_histN / 2] + RANGE_OFFSET_CM;
  s_lastMs = millis();
}

bool  rangeValid() { return !isnan(s_cm) && (millis() - s_lastMs) < RANGE_STALE_MS; }
float rangeCm()    { return rangeValid() ? s_cm : NAN; }

#if RANGEFINDER_TYPE == RANGE_HCSR04
// ---------------- HC-SR04 ultrasonic ----------------
static volatile uint32_t s_rise = 0, s_width = 0;
static volatile bool     s_ready = false;
static uint32_t          s_lastPing = 0;

static void IRAM_ATTR onEcho() {
  uint32_t t = micros();
  if (digitalRead(PIN_RANGE_B)) s_rise = t;
  else { s_width = t - s_rise; s_ready = true; }
}

const char* rangeName() { return "HC-SR04"; }

void rangeBegin() {
  pinMode(PIN_RANGE_A, OUTPUT);
  digitalWrite(PIN_RANGE_A, LOW);
  pinMode(PIN_RANGE_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_RANGE_B), onEcho, CHANGE);
}

void rangeUpdate() {
  if (s_ready) {
    noInterrupts();
    uint32_t w = s_width;
    s_ready = false;
    interrupts();
    float cm = w / 58.0f;               // speed of sound, round trip
    if (cm >= 2.0f && cm <= 400.0f) pushReading(cm);
  }
  uint32_t now = millis();
  if (now - s_lastPing >= HCSR04_PERIOD_MS) {
    s_lastPing = now;
    digitalWrite(PIN_RANGE_A, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_RANGE_A, LOW);
  }
}

#elif RANGEFINDER_TYPE == RANGE_TFLUNA
// ---------------- TF-Luna / TFmini (UART, 115200) ----------------
// Frame: 0x59 0x59 distL distH strL strH tempL tempH checksum
static uint8_t s_buf[9];
static int     s_idx = 0;

const char* rangeName() { return "TF-Luna"; }

void rangeBegin() {
  Serial1.begin(115200, SERIAL_8N1, PIN_RANGE_B /*RX*/, PIN_RANGE_A /*TX*/);
}

void rangeUpdate() {
  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    if (s_idx < 2 && c != 0x59) { s_idx = 0; continue; }
    s_buf[s_idx++] = c;
    if (s_idx < 9) continue;
    s_idx = 0;
    uint8_t sum = 0;
    for (int i = 0; i < 8; i++) sum += s_buf[i];
    if (sum != s_buf[8]) continue;
    uint16_t dist = s_buf[2] | (s_buf[3] << 8);
    uint16_t str  = s_buf[4] | (s_buf[5] << 8);
    if (dist > 0 && str >= TFLUNA_MIN_STRENGTH && str != 0xFFFF) pushReading(dist);
  }
}

#else
// ---------------- none ----------------
const char* rangeName() { return "none"; }
void rangeBegin() {}
void rangeUpdate() {}
#endif
