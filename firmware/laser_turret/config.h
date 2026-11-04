// =====================================================================
//  Vision Based Laser Turret - configuration
//  Board: AI-Thinker ESP32-CAM (OV2640)
//  Everything you are likely to tune lives in this file.
// =====================================================================
#pragma once

// ---------------------------------------------------------------------
//  Pins
//  The ESP32-CAM has very few free GPIOs once the camera is running.
//  These are the safe ones when the SD card is NOT used.
//  (GPIO 12 is a boot strapping pin - nothing that pulls it HIGH may be
//   connected at power-up. A servo signal input is fine.)
// ---------------------------------------------------------------------
#define PIN_SERVO_PAN    12   // Servo 1 - left / right
#define PIN_SERVO_TILT    2   // Servo 2 - up / down
#define PIN_LASER        13   // Laser enable (through a transistor)
#define PIN_RANGE_A      14   // HC-SR04: TRIG     | TF-Luna: ESP TX -> sensor RX
#define PIN_RANGE_B      15   // HC-SR04: ECHO*    | TF-Luna: sensor TX -> ESP RX
                              //  *HC-SR04 echo is 5V - use a voltage divider!
#define PIN_STATUS_LED   33   // Onboard red LED (active LOW). Lit = locked on.

// ---------------------------------------------------------------------
//  Servos (SG90)
// ---------------------------------------------------------------------
#define SERVO_MIN_US      500     // pulse width at 0 deg
#define SERVO_MAX_US     2400     // pulse width at 180 deg

// Soft limits - the turret will never be commanded outside these.
// Set them so the laser can only ever hit the area you are using.
#define PAN_MIN_DEG       30.0f
#define PAN_MAX_DEG      150.0f
#define TILT_MIN_DEG      45.0f
#define TILT_MAX_DEG     135.0f

#define PAN_HOME_DEG      90.0f
#define TILT_HOME_DEG     90.0f

// Max servo movement per control update (smooths motion, limits jerk).
#define MAX_STEP_DEG       4.0f

// ---------------------------------------------------------------------
//  Camera / vision
// ---------------------------------------------------------------------
#define CAM_FRAMESIZE    FRAMESIZE_QQVGA   // 160x120 - fast enough for tracking
#define CAM_VFLIP         0
#define CAM_HMIRROR       0
#define CAM_SATURATION    1                // -2..2, a bit of boost helps the ball pop

// If the ball/laser colours look swapped (blue <-> red) in the debug
// snapshot, flip this to 1.
#define RGB565_SWAP_BYTES 0

// Tennis ball (optic yellow) in HSV. Hue 0-359, Sat/Val 0-255.
#define BALL_HUE_MIN      35
#define BALL_HUE_MAX      85
#define BALL_SAT_MIN      90
#define BALL_VAL_MIN      80
#define BALL_MIN_AREA     20      // px at 160x120 - ignores small specks
#define BALL_MAX_AREA   6000

// Red laser dot. The dot's core is often blown out to white, so we look
// for the strongly red halo around it.
#define LASER_MIN_R      230      // a real dot is near-saturated red; skin/lips aren't
#define LASER_MIN_DIFF    80      // R must beat both G and B by this much
#define LASER_MIN_AREA     1
#define LASER_MAX_AREA   120      // anything bigger is a red object, not the dot
#define LASER_SEARCH_PX   30      // only accept a dot this close to where we expect it (0 = anywhere)

// "Dot is on the target" = at least this fraction of the pixels around the
// dot are target-coloured. Then the turret holds still.
#define ON_TARGET_RADIUS   3
#define ON_TARGET_FRAC     0.4f

// ---------------------------------------------------------------------
//  Tracking control
//  Aim = calibrated map(ball pixel) + bias.  The bias is learned from
//  the visible laser dot so the dot converges onto the ball.
// ---------------------------------------------------------------------
#define DEFAULT_DEG_PER_PX 0.40f  // used before calibration (~66deg FOV / 160px)
#define PAN_SIGN           1.0f   // flip to -1 if, before calibration, pan runs the wrong way
#define TILT_SIGN          1.0f   // flip to -1 if tilt runs the wrong way

#define BIAS_GAIN          0.5f   // 0..1, how much of the dot error is corrected per step
#define BIAS_LIMIT_DEG    15.0f
#define DEADBAND_PX        1.5f   // don't chase sub-pixel noise
#define LOCK_PX            3.0f   // dot within this many px of the ball = "locked"
#define SETTLE_MS          150    // wait for servos to settle before trusting the dot
#define SETTLE_MOVE_DEG    0.6f   // moves smaller than this don't count as "moving"
#define TARGET_SMOOTH      0.5f   // 0..1 smoothing of the target centre (big targets wobble)
#define LOST_TIMEOUT_MS    600    // laser switches off this long after losing the ball

// Calibration sweep (3x3 grid around home)
#define CAL_SPAN_DEG      20.0f
#define CAL_SETTLE_MS      500

// Laser is only ever on while a ball is being tracked AND the turret is armed.
#define START_ARMED         1

// ---------------------------------------------------------------------
//  Rangefinder (module mounted on top of the laser)
// ---------------------------------------------------------------------
#define RANGE_NONE      0
#define RANGE_HCSR04    1   // ultrasonic, echo timing
#define RANGE_TFLUNA    2   // TF-Luna / TFmini / TFmini-S / TFmini Plus (UART, time of flight)

#define RANGEFINDER_TYPE   RANGE_HCSR04
#define RANGE_OFFSET_CM    0.0f    // add/subtract if your mount sits ahead of/behind the pivot
#define RANGE_STALE_MS     500     // reading older than this = invalid
#define HCSR04_PERIOD_MS    60     // don't ping faster than this (echo ringing)
#define TFLUNA_MIN_STRENGTH 100    // below this the TF-Luna reading is unreliable

// ---------------------------------------------------------------------
//  WiFi debug page (optional - the turret works without it)
//  Put your network in secrets.h (see secrets.example.h).
//  If it can't connect it starts its own access point instead.
// ---------------------------------------------------------------------
#define ENABLE_WEB_DEBUG   1
#define AP_SSID            "LaserTurret"
#define AP_PASS            "turret123"     // min 8 chars

#define SERIAL_BAUD        115200
#define STATUS_PRINT_MS    200
