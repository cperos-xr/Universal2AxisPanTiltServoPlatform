// ------------------------------------------------------------
// Realtime “keyboard nudge” servo controller (two servos)
// ------------------------------------------------------------
// PURPOSE:
//   Control two servos interactively from a serial terminal by
//   pressing keys (ideally without pressing Enter).
//
// WHAT IT DOES:
//   - Maintains two signed command values v1 and v2 in degrees
//     where 0 = centered, range is -90..+90.
//   - Each keypress nudges v1/v2 by "step" degrees.
//   - Converts v1/v2 into calibrated microsecond pulse widths,
//     using each servo’s measured safe range.
//
// IMPORTANT:
//   Arduino Serial Monitor is often *not* good for realtime keypresses.
//   Use PuTTY / TeraTerm / CoolTerm if you want true "press keys" control.
// ------------------------------------------------------------

#include <Arduino.h>
#include <ESP32Servo.h>

Servo s1, s2;
#define SERVO1_PIN 3
#define SERVO2_PIN 4

// ------------------------------------------------------------
// Calibrated safe ranges (microseconds) measured earlier
//   Servo 1 is full-range:  500..2400
//   Servo 2 is narrower:    800..2050
//
// These are *hard limits* this program will never exceed.
// ------------------------------------------------------------

static const int S1_MIN_US = 500;
static const int S1_MAX_US = 2400;
static const int S2_MIN_US = 800;
static const int S2_MAX_US = 2050;

static int v1 = 0;   // -90..+90
static int v2 = 0;
static int step = 5; // degrees per keypress

static int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void writeSigned(Servo& s, int v, int minUs, int maxUs) {
  v = clampInt(v, -90, 90);
  int center = (minUs + maxUs) / 2;
  int halfRange = (maxUs - minUs) / 2;
  int us = center + (v * halfRange) / 90;
  us = clampInt(us, minUs, maxUs);
  s.writeMicroseconds(us);
}

static void apply() {
  writeSigned(s1, v1, S1_MIN_US, S1_MAX_US);
  writeSigned(s2, v2, S2_MIN_US, S2_MAX_US);
  Serial.printf("v1=%d  v2=%d  step=%d\n", v1, v2, step);
}

void setup() {
  Serial.begin(115200);
  delay(600);

  s1.setPeriodHertz(50);
  s2.setPeriodHertz(50);
  s1.attach(SERVO1_PIN, 500, 2400);
  s2.attach(SERVO2_PIN, 500, 2400);

  apply();

  Serial.println("\nRealtime keys (no Enter if your terminal sends raw chars):");
  Serial.println("  w/s : servo1 + / -");
  Serial.println("  d/a : servo2 + / -");
  Serial.println("  space: center both");
  Serial.println("  [ / ]: step - / +");
}

void loop() {
  if (Serial.available() > 0) {
    char c = (char)Serial.read();

    switch (c) {
      case 'w': v2 -= step; break;
      case 's': v2 += step; break;
      case 'd': v1 -= step; break;
      case 'a': v1 += step; break;


      case ' ': v1 = 0; v2 = 0; break;

      case '[': step = clampInt(step - 1, 1, 30); break;
      case ']': step = clampInt(step + 1, 1, 30); break;

      default:
        // ignore other bytes (including CR/LF)
        return;
    }

    v1 = clampInt(v1, -90, 90);
    v2 = clampInt(v2, -90, 90);
    apply();
  }
}
