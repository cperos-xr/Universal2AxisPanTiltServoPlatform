// ------------------------------------------------------------
// Servo calibration utility (microseconds)
// ------------------------------------------------------------
// PURPOSE:
//   This sketch lets you manually drive two servos by sending
//   microsecond pulse values over Serial.
//
//   You use this to find each servo’s *safe mechanical limits*
//   (min / max) without guessing angles.
//
// HOW TO USE (IMPORTANT):
//   1. Upload this sketch
//   2. Open Serial Monitor at 115200 baud
//   3. Set line ending to "Newline" or "Both NL & CR"
//   4. Send commands like:
//        s1 1500
//        s1 900
//        s2 2100
//
//   Each command immediately moves that servo.
//   Increase/decrease slowly to find safe endpoints.
// ------------------------------------------------------------

#include <Arduino.h>
#include <ESP32Servo.h>

Servo s1, s2;
#define SERVO1_PIN 3
#define SERVO2_PIN 4

int us1 = 1500, us2 = 1500;

static int clampUS(int us) {
  if (us < 400)  return 400;
  if (us > 2600) return 2600;
  return us;
}

void setup() {
  Serial.begin(115200);
  delay(400);

  s1.setPeriodHertz(50);
  s2.setPeriodHertz(50);
  s1.attach(SERVO1_PIN, 500, 2400);
  s2.attach(SERVO2_PIN, 500, 2400);

  s1.writeMicroseconds(us1);
  s2.writeMicroseconds(us2);

  Serial.println("Servo µs calibrator.");
  Serial.println("Commands: s1 <us>   or   s2 <us>   (try 800..2200)");
  Serial.println("Example: s1 900");
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char which[3] = {0};
  int us = 0;

  if (sscanf(line.c_str(), "%2s %d", which, &us) == 2) {
    us = clampUS(us);

    if (strcmp(which, "s1") == 0) {
      us1 = us;
      s1.writeMicroseconds(us1);
      Serial.printf("s1 -> %d us\n", us1);
    } else if (strcmp(which, "s2") == 0) {
      us2 = us;
      s2.writeMicroseconds(us2);
      Serial.printf("s2 -> %d us\n", us2);
    } else {
      Serial.println("Use s1 or s2.");
    }
  } else {
    Serial.println("Parse error. Use: s1 1500");
  }
}
