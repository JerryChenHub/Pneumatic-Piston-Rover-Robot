#include <Wire.h>
#include <LIS3MDL.h>
#include <math.h>

/* ============================= User Config ============================= */

const uint32_t SERIAL_BAUD = 1000000UL;

const uint16_t SAMPLE_MS = 10;        // 10 ms sampling cycle
const uint16_t PRINT_MS  = 500;       // print average heading every 0.5 s

const uint16_t COMPASS_CAL_MS = 15000;
const uint16_t CAL_LED_TOGGLE_MS = 250;

const uint8_t STATUS_LED_PIN = 13;

// Compass calibration presets.
// Startup calibration overwrites these if calibration span is usable.
const float MAG_MIN_X_PRESET = -1162.0f;
const float MAG_MAX_X_PRESET = 1996.0f;
const float MAG_MIN_Y_PRESET = 1760.0f;
const float MAG_MAX_Y_PRESET = 4932.0f;
const float MAG_MIN_Z_PRESET = -1000.0f;
const float MAG_MAX_Z_PRESET = 1000.0f;

const float MIN_USABLE_MAG_SPAN = 100.0f;

const float HEADING_OFFSET_DEG = 0.0f;
const float MAG_X_SIGN = 1.0f;
const float MAG_Y_SIGN = 1.0f;

/* ============================= Derived Config =========================== */

const uint32_t SAMPLE_US = (uint32_t)SAMPLE_MS * 1000UL;
const uint16_t PRINT_SAMPLE_COUNT = PRINT_MS / SAMPLE_MS;

/* ================================ Globals ================================ */

LIS3MDL mag;

bool statusLed = false;

uint32_t nextSampleUs = 0;

float magMinX = MAG_MIN_X_PRESET;
float magMaxX = MAG_MAX_X_PRESET;
float magMinY = MAG_MIN_Y_PRESET;
float magMaxY = MAG_MAX_Y_PRESET;
float magMinZ = MAG_MIN_Z_PRESET;
float magMaxZ = MAG_MAX_Z_PRESET;

float sinSum = 0.0f;
float cosSum = 0.0f;
uint16_t sampleCount = 0;

/* ================================ Helpers ================================ */

bool due(uint32_t now, uint32_t scheduled) {
  return (int32_t)(now - scheduled) >= 0;
}

void advanceBy(uint32_t &timeUs, uint32_t periodUs, uint32_t nowUs) {
  timeUs += periodUs;
  if ((int32_t)(nowUs - timeUs) >= (int32_t)periodUs) {
    timeUs = nowUs + periodUs;
  }
}

float wrap360(float angleDeg) {
  while (angleDeg >= 360.0f) angleDeg -= 360.0f;
  while (angleDeg < 0.0f) angleDeg += 360.0f;
  return angleDeg;
}

float normAxis(float raw, float lo, float hi) {
  float halfSpan = 0.5f * (hi - lo);
  if (fabs(halfSpan) < 1.0f) halfSpan = 1.0f;
  return (raw - 0.5f * (lo + hi)) / halfSpan;
}

float headingFromRaw(float rawX, float rawY) {
  float x = MAG_X_SIGN * normAxis(rawX, magMinX, magMaxX);
  float y = MAG_Y_SIGN * normAxis(rawY, magMinY, magMaxY);

  return wrap360(atan2(y, x) * 180.0f / PI + HEADING_OFFSET_DEG);
}

float readCompassHeadingDeg() {
  mag.read();
  return headingFromRaw((float)mag.m.x, (float)mag.m.y);
}

/* =========================== Compass Calibration ========================== */

void updateCalibrationExtrema(float &minX, float &maxX,
                              float &minY, float &maxY,
                              float &minZ, float &maxZ) {
  mag.read();

  minX = min(minX, (float)mag.m.x);
  maxX = max(maxX, (float)mag.m.x);

  minY = min(minY, (float)mag.m.y);
  maxY = max(maxY, (float)mag.m.y);

  minZ = min(minZ, (float)mag.m.z);
  maxZ = max(maxZ, (float)mag.m.z);
}

void printCalibrationValues() {
  Serial.println(F("CAL_DONE"));

  Serial.print(F("minX=")); Serial.print(magMinX, 1);
  Serial.print(F(", maxX=")); Serial.println(magMaxX, 1);

  Serial.print(F("minY=")); Serial.print(magMinY, 1);
  Serial.print(F(", maxY=")); Serial.println(magMaxY, 1);

  Serial.print(F("minZ=")); Serial.print(magMinZ, 1);
  Serial.print(F(", maxZ=")); Serial.println(magMaxZ, 1);

  Serial.println(F("Copyable preset lines:"));

  Serial.print(F("const float MAG_MIN_X_PRESET = "));
  Serial.print(magMinX, 1);
  Serial.println(F("f;"));

  Serial.print(F("const float MAG_MAX_X_PRESET = "));
  Serial.print(magMaxX, 1);
  Serial.println(F("f;"));

  Serial.print(F("const float MAG_MIN_Y_PRESET = "));
  Serial.print(magMinY, 1);
  Serial.println(F("f;"));

  Serial.print(F("const float MAG_MAX_Y_PRESET = "));
  Serial.print(magMaxY, 1);
  Serial.println(F("f;"));

  Serial.print(F("const float MAG_MIN_Z_PRESET = "));
  Serial.print(magMinZ, 1);
  Serial.println(F("f;"));

  Serial.print(F("const float MAG_MAX_Z_PRESET = "));
  Serial.print(magMaxZ, 1);
  Serial.println(F("f;"));
}

void calibrateCompass() {
  float minX = 32767.0f;
  float minY = 32767.0f;
  float minZ = 32767.0f;

  float maxX = -32768.0f;
  float maxY = -32768.0f;
  float maxZ = -32768.0f;

  uint32_t startMs = millis();
  uint32_t nextBlinkMs = startMs;
  uint32_t nextReadUs = micros();

  Serial.println(F("CAL_START: rotate rover through as many headings as possible."));

  while ((uint32_t)(millis() - startMs) < COMPASS_CAL_MS) {
    uint32_t nowMs = millis();
    uint32_t nowUs = micros();

    if ((uint32_t)(nowMs - nextBlinkMs) >= CAL_LED_TOGGLE_MS) {
      nextBlinkMs += CAL_LED_TOGGLE_MS;
      statusLed = !statusLed;
      digitalWrite(STATUS_LED_PIN, statusLed ? HIGH : LOW);
    }

    if (due(nowUs, nextReadUs)) {
      advanceBy(nextReadUs, SAMPLE_US, nowUs);
      updateCalibrationExtrema(minX, maxX, minY, maxY, minZ, maxZ);
    }
  }

  digitalWrite(STATUS_LED_PIN, LOW);
  statusLed = false;

  bool xyUsable = (maxX - minX >= MIN_USABLE_MAG_SPAN) &&
                  (maxY - minY >= MIN_USABLE_MAG_SPAN);

  if (xyUsable) {
    magMinX = minX;
    magMaxX = maxX;

    magMinY = minY;
    magMaxY = maxY;

    magMinZ = minZ;
    magMaxZ = maxZ;
  } else {
    Serial.println(F("WARN: compass calibration span too small; using preset values."));
  }

  printCalibrationValues();
}

/* ================================= Setup ================================= */

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  Serial.begin(SERIAL_BAUD);

  Wire.begin();
  Wire.setClock(400000UL);

  if (!mag.init()) {
    Serial.println(F("ERROR: LIS3MDL compass not detected."));
    while (true) {
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(250);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(250);
    }
  }

  mag.enableDefault();

  calibrateCompass();

  sinSum = 0.0f;
  cosSum = 0.0f;
  sampleCount = 0;

  nextSampleUs = micros() + SAMPLE_US;
}

/* ================================= Loop ================================== */

void loop() {
  uint32_t nowUs = micros();

  if (due(nowUs, nextSampleUs)) {
    advanceBy(nextSampleUs, SAMPLE_US, nowUs);

    float headingDeg = readCompassHeadingDeg();
    float headingRad = headingDeg * PI / 180.0f;

    sinSum += sin(headingRad);
    cosSum += cos(headingRad);
    sampleCount++;

    if (sampleCount >= PRINT_SAMPLE_COUNT) {
      float avgHeadingDeg = wrap360(atan2(sinSum, cosSum) * 180.0f / PI);

      Serial.println(avgHeadingDeg, 2);

      sinSum = 0.0f;
      cosSum = 0.0f;
      sampleCount = 0;
    }
  }
}