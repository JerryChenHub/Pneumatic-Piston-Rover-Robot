/*
  Simple piston + 5 ms button logger

  Wiring:
  D2 -> piston MOSFET signal
  D8 -> start button, button other side to GND
  D4 -> left switch, switch other side to GND
  D5 -> right switch, switch other side to GND

  All buttons/switches use INPUT_PULLUP:
  pressed = LOW
*/

#include <Arduino.h>

const byte PISTON_PIN = 2;
const byte LEFT_BUTTON_PIN = 4;
const byte RIGHT_BUTTON_PIN = 5;
const byte START_BUTTON_PIN = 8;

const unsigned long BAUD = 115200;
const unsigned long SAMPLE_MS = 10;
const unsigned long RECORD_MS = 2000;
const unsigned long PISTON_ON_MS = 150;

void setup() {
  pinMode(PISTON_PIN, OUTPUT);
  digitalWrite(PISTON_PIN, LOW);

  pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(BAUD);
  delay(500);

  Serial.println("READY");
  Serial.println("Press START button on D8 to begin.");

  // Wait for start button press
  while (digitalRead(START_BUTTON_PIN) == HIGH) {
    delay(1);
  }

  Serial.println("START");
  Serial.println("time_ms,left_pressed,right_pressed,start_pressed,piston_on");

  unsigned long startMs = millis();
  unsigned long nextSampleMs = startMs;

  digitalWrite(PISTON_PIN, HIGH);

  while (millis() - startMs <= RECORD_MS) {
    unsigned long nowMs = millis();
    unsigned long elapsedMs = nowMs - startMs;

    if (elapsedMs >= PISTON_ON_MS) {
      digitalWrite(PISTON_PIN, LOW);
    }

    if ((long)(nowMs - nextSampleMs) >= 0) {
      bool leftPressed = digitalRead(LEFT_BUTTON_PIN) == LOW;
      bool rightPressed = digitalRead(RIGHT_BUTTON_PIN) == LOW;
      bool startPressed = digitalRead(START_BUTTON_PIN) == LOW;
      bool pistonOn = digitalRead(PISTON_PIN) == HIGH;

      Serial.print(elapsedMs);
      Serial.print(",");
      Serial.print(leftPressed ? 1 : 0);
      Serial.print(",");
      Serial.print(rightPressed ? 1 : 0);
      Serial.print(",");
      Serial.print(startPressed ? 1 : 0);
      Serial.print(",");
      Serial.println(pistonOn ? 1 : 0);

      nextSampleMs += SAMPLE_MS;
    }
  }

  digitalWrite(PISTON_PIN, LOW);
  Serial.println("STOP");
}

void loop() {
  // Do nothing after one run
}