# Vision Based Laser Turret

A pan/tilt laser turret that uses a fixed ESP32-CAM to find a tennis ball and
keep a laser dot on it. A rangefinder rides on top of the laser, so once the
dot is on the ball it can also tell you how far away the ball is.

Everything runs on the ESP32-CAM itself. No laptop is needed.

**[Try the web demo →](https://bhxvshxth.github.io/vision-based-laser-turret/demo/)**

```
           fixed camera                         turret
        ┌──────────────┐                  ┌──────────────────┐
        │  ESP32-CAM   │  sees ball +     │  rangefinder     │
        │   (OV2640)   │  red dot   ───►  │  laser           │
        └──────────────┘                  │  servo 2 (tilt)  │
                                          │  servo 1 (pan)   │
                                          └──────────────────┘
```

## How it works

Every frame (160×120, about 15–20 fps):

1. **Find the ball.** Each pixel is converted to HSV. Optic-yellow pixels go
   into a mask, and the largest connected blob is taken as the ball.
2. **Find the laser dot.** The firmware looks for small, near-saturated red
   blobs. The dot's centre usually saturates to white, so it detects the red
   halo around it. It only accepts a blob close to where the dot *should* be
   given the current aim, so other red things in view (skin, clothes) can't be
   mistaken for it.
3. **Aim.**
   - *Feed-forward:* a calibrated map converts the ball's pixel position into
     pan/tilt angles.
   - *Feedback:* once the turret has aimed where the ball is and the servos
     have settled, the pixel gap between the dot and the ball is turned into
     an angle correction (a "bias") that is gradually learned.
   - *Hold:* if the dot is already on the ball's colour, the turret doesn't
     move. When the ball moves off the dot, tracking picks up again. This stops
     the dot jittering around on big targets.

   Together these let the turret jump straight to the ball and then fine-tune
   until the dot sits on it.
4. **Range.** The rangefinder points wherever the laser points. When the turret
   is locked on, its reading is the distance to the ball. The firmware combines
   that distance with the pan/tilt angles to give the ball's x/y/z position
   relative to the turret.

The camera stays still and the turret sits a little to the side of it. Because
of that, the pixel-to-angle map is only exact at the distance where you
calibrated. The feedback loop corrects for the rest.

## Parts

| Part | Notes |
|---|---|
| AI-Thinker ESP32-CAM | with the OV2640 camera and PSRAM (standard) |
| ESP32-CAM-MB USB board or a USB-serial adapter | for flashing |
| 2× SG90 servos | servo 1 = pan (left/right), servo 2 = tilt (up/down) |
| Laser module | low power only: **Class 2, < 1 mW** (e.g. KY-008) |
| Rangefinder | HC-SR04 (ultrasonic) **or** TF-Luna / TFmini (time-of-flight). See below |
| NPN transistor (2N2222) + 1 kΩ resistor | to switch the laser |
| 5 V supply, 2 A or more | for the servos. **Do not** power servos from the ESP32 |
| 470 µF capacitor | across the servo supply, to stop resets |
| 1 kΩ + 2 kΩ resistors | voltage divider for the HC-SR04 echo pin |

## Wiring

```
ESP32-CAM            Connects to
─────────            ───────────────────────────────────────────
GPIO 12  ─────────── Servo 1 (pan)  signal  (orange)
GPIO 2   ─────────── Servo 2 (tilt) signal  (orange)
GPIO 13  ──[1kΩ]──── 2N2222 base
                     2N2222 emitter → GND
                     2N2222 collector → laser −   (laser + → 5V)
GPIO 14  ─────────── HC-SR04 TRIG      | TF-Luna RX (pin 3)
GPIO 15  ─────────── HC-SR04 ECHO *    | TF-Luna TX (pin 2)
5V       ─────────── rangefinder VCC
GND      ─────────── common ground (ESP32, servo supply, laser, rangefinder)

Servo red/brown → external 5V supply + / −   (grounds tied to the ESP32 GND!)

* HC-SR04 ECHO is 5V. Use a divider:  ECHO ──[1kΩ]──┬── GPIO 15
                                                    └─[2kΩ]── GND
```

Why these pins? With the camera running, the ESP32-CAM only has GPIO 2, 4, 12,
13, 14 and 15 free, and those are only free because the SD card isn't used.

- GPIO 4 drives the very bright flash LED, so it's left alone.
- GPIO 12 must not be pulled HIGH at power-up or the board won't boot. A servo
  signal wire is fine there; a sensor output is not.

All pins are set in `firmware/laser_turret/config.h`.

## Rangefinder

You can use either of two modules. Pick yours in `config.h`:

```c
#define RANGEFINDER_TYPE   RANGE_HCSR04   // or RANGE_TFLUNA, or RANGE_NONE
```

| Module | How it measures | Range | Beam | Notes |
|---|---|---|---|---|
| **HC-SR04** | ultrasonic, times the echo | 2–400 cm | wide (~15°) | cheap; needs the 5V→3.3V divider on ECHO |
| **TF-Luna / TFmini** | light time-of-flight (LiDAR) | 20–800 cm | narrow (~2°) | much better match for a laser: it measures roughly where the dot is |

Mount it **on top of the laser, pointing the same way**, so it moves with the
turret.

- Readings are median-filtered over the last 5 samples.
- A reading older than 500 ms counts as invalid.
- `RANGE_OFFSET_CM` corrects for how far the sensor sits from the pivot.

The distance shows up in the serial output and on the debug page. When locked,
the debug page also shows the ball's position:

```
range 142.3 cm (TF-Luna)   target x 31.2 y -4.0 z 138.8 cm
```

## Setup

1. **Arduino IDE:** install the *esp32* board package by Espressif
   (Boards Manager).
2. Open `firmware/laser_turret/laser_turret.ino`.
3. Board settings:
   - Board: **AI Thinker ESP32-CAM**
   - PSRAM: **Enabled**
   - Partition scheme: **Huge APP**
4. Optional WiFi debug page: copy `secrets.example.h` to `secrets.h` and fill in
   your network. If you skip this, the board makes its own WiFi network called
   `LaserTurret` (password `turret123`). You can also turn the page off
   completely with `ENABLE_WEB_DEBUG 0`.
5. Flash it. If you're not using the MB board: hold GPIO 0 to GND while
   resetting.
6. Open Serial Monitor at **115200**.

## Calibration (do this once)

1. Point the camera at the area you'll use: a wall, floor or table.
2. Keep the ball out of view and dim any other red lights.
3. Send `c` in Serial Monitor (or press **calibrate** on the debug page).

The turret sweeps a 3×3 grid, finds the dot at each point and fits the map. The
result is saved to flash, so it survives a reboot. Re-run it if you move the
camera or the turret.

Before calibration the firmware uses a rough default map. It still tracks, just
more slowly. If the dot moves the *wrong way* before calibration, flip
`PAN_SIGN` or `TILT_SIGN`.

## Serial commands

| Key | Action |
|---|---|
| `c` | calibrate |
| `a` | arm / disarm (disarmed = laser always off) |
| `r` | reset learned bias |
| `x` | clear calibration |
| `h` | help |

Status prints 5× a second:

```
ball ( 96.4, 51.2)  dot ( 95.8, 51.9)  pan  97.1 tilt  86.3  range  142.3 cm  LASER LOCKED  18fps
```

## Debug page

Open the IP address printed at boot. The page has:

- a live snapshot (green cross = ball, cyan cross = detected dot)
- tracking status and range
- buttons: calibrate, arm, disarm, reset bias, clear calibration

Use it for tuning the colour thresholds.

## Tuning

Everything is in `config.h`.

- **Ball not found:** widen `BALL_HUE_MIN/MAX`, or lower `BALL_SAT_MIN` and
  `BALL_VAL_MIN`. Light matters a lot; a dim room makes the ball look dull
  olive.
- **Dot not found:** lower `LASER_MIN_R` or `LASER_MIN_DIFF`. A darker room
  helps.
- **Random red things detected as the dot:** raise those thresholds, lower
  `LASER_MAX_AREA`, or lower `LASER_SEARCH_PX`.
- **Dot stops at the edge of the ball:** that's the hold rule. Raise
  `ON_TARGET_FRAC` to make it push further into the ball before holding.
- **Colours look wrong in the snapshot (red ↔ blue):** set
  `RGB565_SWAP_BYTES 1`.
- **Jittery or overshooting:** lower `MAX_STEP_DEG` or `BIAS_GAIN`, or raise
  `TARGET_SMOOTH` toward 1 for less smoothing / toward 0 for more.
- **Turret swings too far:** tighten `PAN_MIN/MAX_DEG` and `TILT_MIN/MAX_DEG`.

## Safety

- Use a **< 1 mW (Class 2)** laser only. Never look into it.
- The laser is only on while a ball is being tracked and the turret is armed. It
  switches off 0.6 s after the ball is lost.
- Servo soft limits (`PAN_*`, `TILT_*`) stop the turret from aiming outside your
  area. Set them so it can't reach head height.
- The target is a tennis ball by design. Don't retarget it at people or animals.

## Web demo

`demo/` runs the turret's tracking on your laptop camera; no hardware is needed.
Open it [online](https://bhxvshxth.github.io/vision-based-laser-turret/demo/)
and allow camera access.

- Your laptop camera acts as the turret's fixed camera. The tennis ball is the
  target by default.
- **Click any object** to make it the target. The demo samples that object's
  colour, and a simulated laser dot then follows it as it moves. Once the dot
  is on the object it holds still until the object moves away.
- **Reset target** (top-right, or press `r`) switches back to the tennis ball.
- Pick something brightly coloured that stands out from the background. Plain
  white, grey or dark things can't be tracked by colour.
- With no camera, or if access is denied, the demo falls back to a still frame
  where you drag a tennis ball around. You can also drop any photo onto the
  window to use it as the frame.

The demo runs the same detection and control code as the firmware, ported to JS
in `demo/tracker.js`, on a 160×120 copy of each frame. The laser is simulated,
since there isn't one on your laptop. The simulated turret is deliberately
mis-aligned and offset from the camera, so you can watch the feedback loop pull
the dot onto the target.

To run it locally, browsers only allow camera access over `https` or
`localhost`. Opening the file directly may not work, so serve the folder
instead:

```
cd demo && python3 -m http.server 8000
# then open http://localhost:8000
```

## Project layout

```
vision based laser turret/
├── README.md
├── firmware/laser_turret/
│   ├── laser_turret.ino      main loop, tracking control, calibration, serial
│   ├── config.h              pins, thresholds, gains, limits
│   ├── vision.cpp/.h         ball + laser detection on RGB565 frames
│   ├── calibration.cpp/.h    pixel ↔ angle map, least-squares fit, flash storage
│   ├── servo_ledc.cpp/.h     servo PWM that doesn't clash with the camera clock
│   ├── rangefinder.cpp/.h    HC-SR04 / TF-Luna drivers
│   ├── web_debug.cpp/.h      optional WiFi debug page
│   ├── turret_state.h        shared state
│   └── secrets.example.h     WiFi template (copy to secrets.h)
└── demo/
    ├── index.html
    ├── demo.js               scene, simulated turret, input
    └── tracker.js            detection + control (same as firmware)
```

## Troubleshooting

- **Board resets when the servos move:** the servos are browning out the board.
  Give them their own 5 V supply, connect the grounds together, and add the
  470 µF cap.
- **Won't boot or flash with everything connected:** something is pulling
  GPIO 12 HIGH at power-up. Unplug it, flash, and check the wiring.
- **`camera init failed`:** reseat the camera ribbon and check that PSRAM is
  enabled.
- **Servos twitch at boot:** this is normal. GPIO 2 and 12 float briefly before
  the firmware takes over.
