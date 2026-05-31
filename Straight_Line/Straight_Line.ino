/*
  Piston Rover Control

  This sketch keeps the propeller-stand reference structure: a fast sampling
  stage gathers valid measurements, and a timed execution stage consumes those
  measurements and updates the actuator command. The compass is never read while
  the piston MOSFET is ON because that interval can corrupt the magnetic field.
*/

#include <Wire.h>
#include <Servo.h>
#include <LIS3MDL.h>
#include <math.h>

/*
  Hardware map and fixed timing configuration.
*/
const byte PISTON_PIN = 2;
const byte SERVO_PIN = 3;
const byte LEFT_REAR_LIMIT_PIN = 4;
const byte RIGHT_REAR_LIMIT_PIN = 5;
const byte CAL_LED_PIN = 13;

const unsigned long CALIBRATION_WINDOW_MS = 10000UL;
const unsigned long CAL_LED_TOGGLE_MS = 50UL;
const unsigned long CAL_COMPASS_SAMPLE_MS = 20UL;
const unsigned long COMPASS_SAMPLE_MS = 20UL;
const unsigned long COMPASS_SETTLE_AFTER_OFF_MS = 20UL;
const unsigned long EXECUTION_PERIOD_MS = 100UL;
const unsigned long STATUS_PERIOD_MS = 500UL;

const unsigned int TOTAL_PISTON_CYCLES = 10;
const unsigned long PISTON_ON_MS = 100UL;
const unsigned long PISTON_OFF_MS = 400UL;

/*
  Compass calibration values and steering tuning.
*/
const float PRE_MIN_X = -1162.0f;
const float PRE_MAX_X = 1996.0f;
const float PRE_MIN_Y =   1760.0f;
const float PRE_MAX_Y =   4932.0f;
const float PRE_MIN_Z =  9057.0f;
const float PRE_MAX_Z =  9374.0f;

float minX = PRE_MIN_X, maxX = PRE_MAX_X;
float minY = PRE_MIN_Y, maxY = PRE_MAX_Y;
float minZ = PRE_MIN_Z, maxZ = PRE_MAX_Z;

const float HEADING_OFFSET_DEG = 0.0f;
const float HEADING_X_SIGN = 1.0f;
const float HEADING_Y_SIGN = 1.0f;
const float HEADING_DEADBAND_DEG = 2.0f;

const int SERVO_MIN_DEG = 65;
const int SERVO_CENTER_DEG = 90;
const int SERVO_MAX_DEG = 125;

const float STEERING_SIGN = 1.0f;
const float STEER_KP_LEFT = 0.75f;
const float STEER_KP_RIGHT = 0.55f;

const bool ENABLE_STATUS_PRINTS = false;

/*
  Rover state, timing registers, and compass sample accumulation.
*/
LIS3MDL mag;
Servo steeringServo;

enum RoverState {
  STATE_CALIBRATING,
  STATE_START_RUN,
  STATE_RUNNING,
  STATE_DONE,
  STATE_COMPASS_ERROR
};

RoverState roverState = STATE_CALIBRATING;

unsigned long calibrationStartMs = 0;
unsigned long lastCalLedMs = 0;
unsigned long lastCalSampleMs = 0;
unsigned long lastCompassSampleMs = 0;
unsigned long lastExecutionMs = 0;
unsigned long lastStatusMs = 0;
unsigned long pistonPhaseStartMs = 0;
unsigned long pistonLastOffMs = 0;

bool calLedState = false;
bool pistonIsOn = false;
bool donePrinted = false;

unsigned int pistonCyclesStarted = 0;
unsigned int pistonCyclesCompleted = 0;

float targetHeadingDeg = 0.0f;
float lastCleanHeadingDeg = 0.0f;
float lastHeadingErrorDeg = 0.0f;
int lastServoCommandDeg = SERVO_CENTER_DEG;

float sampleSinSum = 0.0f;
float sampleCosSum = 0.0f;
unsigned int cleanSampleCount = 0;
unsigned int skippedControlWindows = 0;

/*
  General timing, angle, and output helpers.
*/
bool elapsedMs(unsigned long nowMs, unsigned long startMs, unsigned long intervalMs) {
  return (nowMs - startMs) >= intervalMs;
}

float wrap360(float angleDeg) {
  while (angleDeg >= 360.0f) angleDeg -= 360.0f;
  while (angleDeg < 0.0f) angleDeg += 360.0f;
  return angleDeg;
}

float wrap180(float angleDeg) {
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg <= -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

int safeServoCommand(float commandDeg) {
  int command = (int)round(commandDeg);
  return constrain(command, SERVO_MIN_DEG, SERVO_MAX_DEG);
}

void writeSteeringDeg(int commandDeg) {
  lastServoCommandDeg = constrain(commandDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  steeringServo.write(lastServoCommandDeg);
}

void resetSampleWindow() {
  sampleSinSum = 0.0f;
  sampleCosSum = 0.0f;
  cleanSampleCount = 0;
}

void setPiston(bool on, unsigned long transitionMs) {
  if (pistonIsOn == on) return;

  pistonIsOn = on;
  digitalWrite(PISTON_PIN, on ? HIGH : LOW);

  if (!on) {
    pistonLastOffMs = transitionMs;
    lastCompassSampleMs = transitionMs;
  }
}

void allStop(unsigned long nowMs) {
  setPiston(false, nowMs);
  writeSteeringDeg(SERVO_CENTER_DEG);
}

/*
  Compass conversion and circular averaging.
*/
float normalizedAxis(float rawValue, float lowValue, float highValue) {
  const float center = 0.5f * (lowValue + highValue);
  float halfRange = 0.5f * (highValue - lowValue);

  if (fabs(halfRange) < 1.0f) {
    halfRange = 1.0f;
  }

  return (rawValue - center) / halfRange;
}

void readCompassRaw(float &rawX, float &rawY, float &rawZ) {
  mag.read();
  rawX = (float)mag.m.x;
  rawY = (float)mag.m.y;
  rawZ = (float)mag.m.z;
}

float headingFromRawXY(float rawX, float rawY) {
  const float x = HEADING_X_SIGN * normalizedAxis(rawX, minX, maxX);
  const float y = HEADING_Y_SIGN * normalizedAxis(rawY, minY, maxY);
  return wrap360(atan2(y, x) * 180.0f / PI + HEADING_OFFSET_DEG);
}

float readCompassHeadingDeg() {
  float rawX, rawY, rawZ;
  readCompassRaw(rawX, rawY, rawZ);
  return headingFromRawXY(rawX, rawY);
}

void acceptCleanHeading(float headingDeg) {
  const float headingRad = headingDeg * PI / 180.0f;
  sampleSinSum += sin(headingRad);
  sampleCosSum += cos(headingRad);
  cleanSampleCount++;
  lastCleanHeadingDeg = headingDeg;
}

bool consumeCleanHeadingMean(float &headingDeg) {
  if (cleanSampleCount == 0) return false;

  headingDeg = wrap360(atan2(sampleSinSum, sampleCosSum) * 180.0f / PI);
  lastCleanHeadingDeg = headingDeg;
  return true;
}

/*
  Calibration stage. The LED blink and compass sampling are scheduled from
  millis(), so the user gets a 10 s rotation window without blocking the loop.
*/
void loadPreCalibrationValues() {
  minX = PRE_MIN_X; maxX = PRE_MAX_X;
  minY = PRE_MIN_Y; maxY = PRE_MAX_Y;
  minZ = PRE_MIN_Z; maxZ = PRE_MAX_Z;
}

void beginCalibration(unsigned long nowMs) {
  loadPreCalibrationValues();

  calibrationStartMs = nowMs;
  lastCalLedMs = nowMs;
  lastCalSampleMs = nowMs;
  calLedState = false;

  digitalWrite(CAL_LED_PIN, LOW);
  allStop(nowMs);

  Serial.println(F("Compass calibration started. Rotate the rover for 10 seconds."));
}

void updateCalibrationBounds() {
  float rawX, rawY, rawZ;
  readCompassRaw(rawX, rawY, rawZ);

  if (rawX < minX) minX = rawX;
  if (rawX > maxX) maxX = rawX;
  if (rawY < minY) minY = rawY;
  if (rawY > maxY) maxY = rawY;
  if (rawZ < minZ) minZ = rawZ;
  if (rawZ > maxZ) maxZ = rawZ;
}

void printCalibrationValues() {
  Serial.println(F("Calibration complete. Current values:"));
  Serial.print(F("float minX = ")); Serial.print(minX, 1);
  Serial.print(F(", maxX = ")); Serial.print(maxX, 1); Serial.println(F(";"));
  Serial.print(F("float minY = ")); Serial.print(minY, 1);
  Serial.print(F(", maxY = ")); Serial.print(maxY, 1); Serial.println(F(";"));
  Serial.print(F("float minZ = ")); Serial.print(minZ, 1);
  Serial.print(F(", maxZ = ")); Serial.print(maxZ, 1); Serial.println(F(";"));
}

void calibrationStage(unsigned long nowMs) {
  if (elapsedMs(nowMs, lastCalLedMs, CAL_LED_TOGGLE_MS)) {
    lastCalLedMs += CAL_LED_TOGGLE_MS;
    calLedState = !calLedState;
    digitalWrite(CAL_LED_PIN, calLedState ? HIGH : LOW);
  }

  if (elapsedMs(nowMs, lastCalSampleMs, CAL_COMPASS_SAMPLE_MS)) {
    lastCalSampleMs += CAL_COMPASS_SAMPLE_MS;
    updateCalibrationBounds();
  }

  if (elapsedMs(nowMs, calibrationStartMs, CALIBRATION_WINDOW_MS)) {
    digitalWrite(CAL_LED_PIN, LOW);
    printCalibrationValues();
    roverState = STATE_START_RUN;
  }
}

/*
  Sampling stage. Compass samples are accepted only when the piston has been OFF
  long enough for the magnetic disturbance to settle.
*/
void samplingStage(unsigned long nowMs) {
  if (pistonIsOn) return;
  if (!elapsedMs(nowMs, pistonLastOffMs, COMPASS_SETTLE_AFTER_OFF_MS)) return;
  if (!elapsedMs(nowMs, lastCompassSampleMs, COMPASS_SAMPLE_MS)) return;

  lastCompassSampleMs += COMPASS_SAMPLE_MS;
  acceptCleanHeading(readCompassHeadingDeg());
}

/*
  Execution stage. This runs every 100 ms. Steering is updated only from clean
  OFF-phase compass samples; an execution tick that begins during piston ON holds
  the previous steering command and then advances the piston schedule.
*/
void updateSteeringFromHeading(float measuredHeadingDeg) {
  lastHeadingErrorDeg = wrap180(targetHeadingDeg - measuredHeadingDeg);

  float signedErrorDeg = STEERING_SIGN * lastHeadingErrorDeg;

  if (fabs(signedErrorDeg) < HEADING_DEADBAND_DEG) {
    signedErrorDeg = 0.0f;
  }

  const float kp = (signedErrorDeg >= 0.0f) ? STEER_KP_RIGHT : STEER_KP_LEFT;
  const float commandDeg = (float)SERVO_CENTER_DEG + kp * signedErrorDeg;
  writeSteeringDeg(safeServoCommand(commandDeg));
}

void updatePistonSchedule(unsigned long nowMs) {
  if (pistonCyclesCompleted >= TOTAL_PISTON_CYCLES) {
    setPiston(false, nowMs);
    return;
  }

  if (pistonIsOn) {
    if (elapsedMs(nowMs, pistonPhaseStartMs, PISTON_ON_MS)) {
      pistonPhaseStartMs = nowMs;
      setPiston(false, nowMs);
    }
    return;
  }

  if (elapsedMs(nowMs, pistonPhaseStartMs, PISTON_OFF_MS)) {
    pistonCyclesCompleted = pistonCyclesStarted;

    if (pistonCyclesCompleted >= TOTAL_PISTON_CYCLES) {
      setPiston(false, nowMs);
      return;
    }

    pistonCyclesStarted++;
    pistonPhaseStartMs = nowMs;
    resetSampleWindow();
    setPiston(true, nowMs);
  }
}

void printStatus(unsigned long nowMs, bool usedCleanHeading, bool tickStartedWithPistonOn) {
  if (!ENABLE_STATUS_PRINTS) return;
  if (!elapsedMs(nowMs, lastStatusMs, STATUS_PERIOD_MS)) return;

  lastStatusMs += STATUS_PERIOD_MS;

  Serial.print(F("started=")); Serial.print(pistonCyclesStarted);
  Serial.print(F(" completed=")); Serial.print(pistonCyclesCompleted);
  Serial.print(F(" piston=")); Serial.print(pistonIsOn ? F("ON") : F("OFF"));
  Serial.print(F(" control="));
  Serial.print(usedCleanHeading ? F("updated") : (tickStartedWithPistonOn ? F("held_piston_on") : F("held_no_sample")));
  Serial.print(F(" target=")); Serial.print(targetHeadingDeg, 1);
  Serial.print(F(" heading=")); Serial.print(lastCleanHeadingDeg, 1);
  Serial.print(F(" error=")); Serial.print(lastHeadingErrorDeg, 1);
  Serial.print(F(" servo=")); Serial.print(lastServoCommandDeg);
  Serial.print(F(" skipped=")); Serial.println(skippedControlWindows);
}

void executionStage(unsigned long nowMs) {
  const bool tickStartedWithPistonOn = pistonIsOn;
  bool usedCleanHeading = false;
  float measuredHeadingDeg = lastCleanHeadingDeg;

  if (!tickStartedWithPistonOn) {
    usedCleanHeading = consumeCleanHeadingMean(measuredHeadingDeg);
  }

  if (usedCleanHeading) {
    updateSteeringFromHeading(measuredHeadingDeg);
  } else {
    skippedControlWindows++;
  }

  resetSampleWindow();
  updatePistonSchedule(nowMs);
  printStatus(nowMs, usedCleanHeading, tickStartedWithPistonOn);

  if (pistonCyclesCompleted >= TOTAL_PISTON_CYCLES && !pistonIsOn) {
    allStop(nowMs);
    roverState = STATE_DONE;
  }
}

/*
  Run startup and Arduino entry points.
*/
void startRun(unsigned long nowMs) {
  allStop(nowMs);
  writeSteeringDeg(SERVO_CENTER_DEG);

  targetHeadingDeg = readCompassHeadingDeg();
  lastCleanHeadingDeg = targetHeadingDeg;
  lastHeadingErrorDeg = 0.0f;

  resetSampleWindow();
  lastCompassSampleMs = nowMs;
  lastExecutionMs = nowMs;
  lastStatusMs = nowMs;
  skippedControlWindows = 0;
  donePrinted = false;

  pistonCyclesStarted = 1;
  pistonCyclesCompleted = 0;
  pistonPhaseStartMs = nowMs;
  setPiston(true, nowMs);

  Serial.print(F("Run started. Holding initial heading: "));
  Serial.println(targetHeadingDeg, 2);
  Serial.println(F("Execution period: 100 ms. Compass and steering update are skipped while piston is ON."));

  roverState = STATE_RUNNING;
}

void setup() {
  pinMode(PISTON_PIN, OUTPUT);
  pinMode(CAL_LED_PIN, OUTPUT);
  pinMode(LEFT_REAR_LIMIT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_REAR_LIMIT_PIN, INPUT_PULLUP);

  digitalWrite(PISTON_PIN, LOW);
  digitalWrite(CAL_LED_PIN, LOW);

  Serial.begin(115200);
  Wire.begin();

  steeringServo.attach(SERVO_PIN);
  writeSteeringDeg(SERVO_CENTER_DEG);

  if (!mag.init()) {
    Serial.println(F("ERROR: LIS3MDL compass not detected. Rover stopped."));
    roverState = STATE_COMPASS_ERROR;
    allStop(millis());
    return;
  }

  mag.enableDefault();
  beginCalibration(millis());
}

void loop() {
  const unsigned long nowMs = millis();

  switch (roverState) {
    case STATE_CALIBRATING:
      calibrationStage(nowMs);
      break;

    case STATE_START_RUN:
      startRun(nowMs);
      break;

    case STATE_RUNNING:
      samplingStage(nowMs);
      if (elapsedMs(nowMs, lastExecutionMs, EXECUTION_PERIOD_MS)) {
        lastExecutionMs += EXECUTION_PERIOD_MS;
        executionStage(nowMs);
      }
      break;

    case STATE_DONE:
      allStop(nowMs);
      if (!donePrinted) {
        donePrinted = true;
        Serial.println(F("Run complete. Piston OFF, steering centered."));
      }
      break;

    case STATE_COMPASS_ERROR:
    default:
      allStop(nowMs);
      if (elapsedMs(nowMs, lastCalLedMs, 500UL)) {
        lastCalLedMs = nowMs;
        calLedState = !calLedState;
        digitalWrite(CAL_LED_PIN, calLedState ? HIGH : LOW);
      }
      break;
  }
}
