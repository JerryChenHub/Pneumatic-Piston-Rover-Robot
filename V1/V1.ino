/*
  Rover control overview:
    - Startup compass calibration runs for COMPASS_CAL_MS.
    - The heading at the end of calibration becomes the initial heading.
    - Stage 1: drive straight for STAGE1_DISTANCE_M.
    - Turn: keep a fixed steering command until the compass heading changes by TURN_ANGLE_DEG.
    - Stage 2: center steering, then drive straight for STAGE2_DISTANCE_M.
    - Stop piston and center steering after Stage 2.

  Compass rule:
    - The piston disturbs compass readings.
    - Compass is read only when the piston is OFF and after COMPASS_SETTLE_AFTER_OFF_MS.
    - Turn completion is checked only in the DECISION_MS control cycle.

  Hardware:
    - Piston MOSFET: D2
    - Steering servo: D3
    - Left / right rear wheel limit switches: D4 / D5, active LOW, INPUT_PULLUP
    - Compass: LIS3MDL via I2C
*/

#include <Wire.h>
#include <Servo.h>
#include <LIS3MDL.h>
#include <math.h>

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  #include <avr/interrupt.h>
  #define HAS_AVR_PCINT 1
#else
  #define HAS_AVR_PCINT 0
#endif

/* ============================= User Config ============================= */

// Mission profile.
const float STAGE1_DISTANCE_M = 3.0f;                // First straight distance.
const float TURN_ANGLE_DEG = 90.0f;                  // Right-turn heading change.
const float STAGE2_DISTANCE_M = 10.0f;                // Second straight/cruise distance.
const float TURN_DONE_TOL_DEG = 2.0f;                // Turn is done at target - tolerance.
const uint8_t TURN_DONE_CONFIRM_SAMPLES = 2;         // Consecutive 10 Hz confirmations.

// Pin map.
const uint8_t PISTON_PIN = 2;
const uint8_t SERVO_PIN = 3;
const uint8_t LEFT_LIMIT_PIN = 4;
const uint8_t RIGHT_LIMIT_PIN = 5;
const uint8_t STATUS_LED_PIN = 13;

// Timing.
const uint32_t SERIAL_BAUD = 1000000UL;
const uint16_t SAMPLE_MS = 10;                       // 100 Hz sample/log cycle.
const uint16_t DECISION_MS = 100;                    // 10 Hz steering decision cycle.
const uint16_t PISTON_PERIOD_MS = 1000;
const uint16_t PISTON_ON_MS = 250;
const uint16_t COMPASS_SETTLE_AFTER_OFF_MS = 20;
const uint16_t COMPASS_CAL_MS = 10000;
const uint16_t CAL_LED_TOGGLE_MS = 250;
const uint16_t COMPASS_TIMEOUT_MS = 600;
const uint32_t LIMIT_DEBOUNCE_US = 3000UL;

// Steering.
const int SERVO_PHYS_MIN_DEG = 65;
const int SERVO_PHYS_MAX_DEG = 125;
const int SERVO_STRAIGHT_MIN_DEG = 85;
const int SERVO_STRAIGHT_MAX_DEG = 97;
const int SERVO_CENTER_TRIM_DEG = 90;                // Tune if actual center is not 90.
const int STEERING_SIGN = -1;                        // Flip if PID correction direction is reversed.
const int TURN_SERVO_DEG = 110;                      // Fixed right-turn steering command.
const int RIGHT_TURN_HEADING_SIGN = +1;              // Use -1 if right turn decreases compass heading.

// Wheel and chassis geometry.
const float TRACK_WIDTH_M = 0.188f;                  // 188 mm.
const float TICK_M = 3.45f * 3.14f * 0.01f;          // One press = 3.45*pi cm ~= 0.10833 m.

// Compass calibration presets. Startup calibration overwrites runtime min/max values.
const float MAG_MIN_X_PRESET = -1162.0f;
const float MAG_MAX_X_PRESET = 1996.0f;
const float MAG_MIN_Y_PRESET = 1760.0f;
const float MAG_MAX_Y_PRESET = 4932.0f;
const float MAG_MIN_Z_PRESET = -1000.0f;
const float MAG_MAX_Z_PRESET = 1000.0f;
const float MIN_USABLE_MAG_SPAN = 100.0f;
const float HEADING_OFFSET_DEG = 0.0f;
const float MAG_X_SIGN = 1.0f;
const float MAG_Y_SIGN = 1.0f;                       // Flip one axis if heading is mirrored.

// Heading-hold PID. Error unit is degree; output unit is servo degree.
const float KP_HEADING = 1.20f;
const float KI_HEADING = 0.01f;
const float KD_HEADING = 0.08f;
const float HEADING_DEADBAND_DEG = 2.0f;
const float I_LIMIT_DEG_S = 80.0f;
const float PID_LIMIT_DEG = 30.0f;

/* ============================= Derived Config =========================== */

const uint32_t SAMPLE_US = (uint32_t)SAMPLE_MS * 1000UL;
const uint32_t DECISION_US = (uint32_t)DECISION_MS * 1000UL;
const uint32_t COMPASS_SETTLE_US = (uint32_t)COMPASS_SETTLE_AFTER_OFF_MS * 1000UL;

/* ================================ Globals ================================ */

LIS3MDL mag;
Servo steeringServo;

struct WheelSnap {
  bool leftPressed, rightPressed;
  bool leftNewPress, rightNewPress;
  uint32_t leftCount, rightCount;
  uint32_t leftDtUs, rightDtUs;
} wheel;

enum RoverPhase {
  PHASE_STRAIGHT_1 = 0,
  PHASE_TURN_RIGHT = 1,
  PHASE_STRAIGHT_2 = 2,
  PHASE_DONE = 3
};

RoverPhase phase = PHASE_STRAIGHT_1;

bool systemHealthy = true;
bool pistonOn = false;
bool roverDone = false;
bool statusLed = false;
bool compassValid = false;
bool compassUpdatedSinceDecision = false;

uint8_t turnDoneConfirmCount = 0;

uint32_t pistonStartMs = 0;
uint32_t pistonLastOffUs = 0;
uint32_t pistonCycles = 0;
uint32_t nextSampleUs = 0;
uint32_t nextDecisionUs = 0;
uint32_t lastBlinkMs = 0;
uint32_t lastCompassMs = 0;
uint32_t startLeftCount = 0;
uint32_t startRightCount = 0;
uint32_t turnStartLeftCount = 0;
uint32_t turnStartRightCount = 0;

float magMinX = MAG_MIN_X_PRESET;
float magMaxX = MAG_MAX_X_PRESET;
float magMinY = MAG_MIN_Y_PRESET;
float magMaxY = MAG_MAX_Y_PRESET;
float magMinZ = MAG_MIN_Z_PRESET;
float magMaxZ = MAG_MAX_Z_PRESET;

float initialHeadingDeg = NAN;
float targetHeadingDeg = NAN;
float rightTurnTargetHeadingDeg = NAN;
float lastCompassDeg = NAN;
float headingErrDeg = 0.0f;

float distanceM = 0.0f;
float turnProgressDeg = NAN;
float turnRadiusM = NAN;
float turnWheelAngleDeg = NAN;

float errIntegral = 0.0f;
float lastErrDeg = 0.0f;
float lastPidDeg = 0.0f;
int lastServoDeg = SERVO_CENTER_TRIM_DEG;

/* ================================ Helpers ================================ */

bool due(uint32_t now, uint32_t scheduled) {
  return (int32_t)(now - scheduled) >= 0;
}

void advanceBy(uint32_t &timeUs, uint32_t periodUs, uint32_t nowUs) {
  timeUs += periodUs;
  if ((int32_t)(nowUs - timeUs) >= (int32_t)periodUs) timeUs = nowUs + periodUs;
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

int roundToInt(float value) {
  return (int)(value + (value >= 0.0f ? 0.5f : -0.5f));
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

void writeServoDeg(int deg, bool useStraightLimit = false) {
  if (useStraightLimit) {
    lastServoDeg = constrain(deg, SERVO_STRAIGHT_MIN_DEG, SERVO_STRAIGHT_MAX_DEG);
  } else {
    lastServoDeg = constrain(deg, SERVO_PHYS_MIN_DEG, SERVO_PHYS_MAX_DEG);
  }
  steeringServo.write(lastServoDeg);
}

void resetPidState() {
  errIntegral = 0.0f;
  lastErrDeg = 0.0f;
  lastPidDeg = 0.0f;
}

void resetSegmentDistanceCounts() {
  startLeftCount = wheel.leftCount;
  startRightCount = wheel.rightCount;
  distanceM = 0.0f;
}

float currentStraightTargetDistanceM() {
  if (phase == PHASE_STRAIGHT_1) return STAGE1_DISTANCE_M;
  if (phase == PHASE_STRAIGHT_2) return STAGE2_DISTANCE_M;
  return NAN;
}

float turnProgressFromInitialHeading() {
  if (isnan(lastCompassDeg) || isnan(initialHeadingDeg)) return NAN;
  return RIGHT_TURN_HEADING_SIGN * wrap180(lastCompassDeg - initialHeadingDeg);
}

void failStop(const __FlashStringHelper *message) {
  systemHealthy = false;
  digitalWrite(PISTON_PIN, LOW);
  writeServoDeg(SERVO_CENTER_TRIM_DEG);
  Serial.println(message);
}

void blinkError(uint32_t nowMs) {
  if (nowMs - lastBlinkMs < 250UL) return;
  lastBlinkMs = nowMs;
  statusLed = !statusLed;
  digitalWrite(STATUS_LED_PIN, statusLed ? HIGH : LOW);
}

void printFloatOrNan(float value, uint8_t digits) {
  if (isnan(value)) Serial.print(F("nan"));
  else Serial.print(value, digits);
}

/* =========================== Compass Calibration ========================== */

void printCalibrationValues() {
  Serial.println(F("CAL_DONE"));
  Serial.print(F("minX=")); Serial.print(magMinX, 1); Serial.print(F(", maxX=")); Serial.println(magMaxX, 1);
  Serial.print(F("minY=")); Serial.print(magMinY, 1); Serial.print(F(", maxY=")); Serial.println(magMaxY, 1);
  Serial.print(F("minZ=")); Serial.print(magMinZ, 1); Serial.print(F(", maxZ=")); Serial.println(magMaxZ, 1);
  Serial.println(F("Copyable preset lines:"));
  Serial.print(F("const float MAG_MIN_X_PRESET = ")); Serial.print(magMinX, 1); Serial.println(F("f;"));
  Serial.print(F("const float MAG_MAX_X_PRESET = ")); Serial.print(magMaxX, 1); Serial.println(F("f;"));
  Serial.print(F("const float MAG_MIN_Y_PRESET = ")); Serial.print(magMinY, 1); Serial.println(F("f;"));
  Serial.print(F("const float MAG_MAX_Y_PRESET = ")); Serial.print(magMaxY, 1); Serial.println(F("f;"));
  Serial.print(F("const float MAG_MIN_Z_PRESET = ")); Serial.print(magMinZ, 1); Serial.println(F("f;"));
  Serial.print(F("const float MAG_MAX_Z_PRESET = ")); Serial.print(magMaxZ, 1); Serial.println(F("f;"));
}

void updateCalibrationExtrema(float &minX, float &maxX,
                              float &minY, float &maxY,
                              float &minZ, float &maxZ) {
  mag.read();
  minX = min(minX, (float)mag.m.x); maxX = max(maxX, (float)mag.m.x);
  minY = min(minY, (float)mag.m.y); maxY = max(maxY, (float)mag.m.y);
  minZ = min(minZ, (float)mag.m.z); maxZ = max(maxZ, (float)mag.m.z);
}

void calibrateCompass() {
  digitalWrite(PISTON_PIN, LOW);
  writeServoDeg(SERVO_CENTER_TRIM_DEG);

  float minX = 32767.0f, minY = 32767.0f, minZ = 32767.0f;
  float maxX = -32768.0f, maxY = -32768.0f, maxZ = -32768.0f;
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
    magMinX = minX; magMaxX = maxX;
    magMinY = minY; magMaxY = maxY;
    magMinZ = minZ; magMaxZ = maxZ;
  } else {
    Serial.println(F("WARN: compass calibration span too small; using preset values."));
  }

  printCalibrationValues();
}

float averageCompassHeading(uint8_t sampleCount) {
  float sinSum = 0.0f;
  float cosSum = 0.0f;

  for (uint8_t i = 0; i < sampleCount; i++) {
    float headingRad = readCompassHeadingDeg() * PI / 180.0f;
    sinSum += sin(headingRad);
    cosSum += cos(headingRad);
    delay(10);
  }

  return wrap360(atan2(sinSum, cosSum) * 180.0f / PI);
}

/* ============================= Wheel Counters ============================ */

#if !HAS_AVR_PCINT
struct PolledCounter {
  uint8_t pin;
  bool raw;
  bool stable;
  bool newPress;
  uint32_t changedUs;
  uint32_t lastPressUs;
  uint32_t intervalUs;
  uint32_t count;

  void begin(uint8_t inputPin, uint32_t nowUs) {
    pin = inputPin;
    raw = stable = (digitalRead(pin) == LOW);
    newPress = false;
    changedUs = nowUs;
    lastPressUs = intervalUs = count = 0;
  }

  void update(uint32_t nowUs) {
    bool reading = (digitalRead(pin) == LOW);
    if (reading != raw) {
      raw = reading;
      changedUs = nowUs;
    }

    if (reading != stable && nowUs - changedUs >= LIMIT_DEBOUNCE_US) {
      stable = reading;
      if (stable) {
        intervalUs = lastPressUs ? nowUs - lastPressUs : 0;
        lastPressUs = nowUs;
        count++;
        newPress = true;
      }
    }
  }
};

PolledCounter leftCtr, rightCtr;
#endif

#if HAS_AVR_PCINT
const uint8_t LEFT_MASK = _BV(PD4);
const uint8_t RIGHT_MASK = _BV(PD5);
volatile uint8_t vLastD = 0;
volatile bool vLP = false, vRP = false, vLN = false, vRN = false;
volatile uint32_t vLC = 0, vRC = 0, vLLast = 0, vRLast = 0, vLAcc = 0, vRAcc = 0, vLdt = 0, vRdt = 0;

void acceptWheelPress(volatile uint32_t &count,
                      volatile uint32_t &lastPressUs,
                      volatile uint32_t &acceptedUs,
                      volatile uint32_t &dtUs,
                      volatile bool &newPress,
                      uint32_t nowUs) {
  if (nowUs - acceptedUs < LIMIT_DEBOUNCE_US) return;
  dtUs = lastPressUs ? nowUs - lastPressUs : 0;
  lastPressUs = acceptedUs = nowUs;
  count++;
  newPress = true;
}

ISR(PCINT2_vect) {
  uint8_t d = PIND;
  uint8_t changed = d ^ vLastD;
  uint32_t nowUs = micros();

  if (changed & LEFT_MASK) {
    vLP = ((d & LEFT_MASK) == 0);
    if (vLP) acceptWheelPress(vLC, vLLast, vLAcc, vLdt, vLN, nowUs);
  }

  if (changed & RIGHT_MASK) {
    vRP = ((d & RIGHT_MASK) == 0);
    if (vRP) acceptWheelPress(vRC, vRLast, vRAcc, vRdt, vRN, nowUs);
  }

  vLastD = d;
}
#endif

void beginWheelCounters(uint32_t nowUs) {
  pinMode(LEFT_LIMIT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_LIMIT_PIN, INPUT_PULLUP);

#if HAS_AVR_PCINT
  noInterrupts();
  vLastD = PIND;
  vLP = ((vLastD & LEFT_MASK) == 0);
  vRP = ((vLastD & RIGHT_MASK) == 0);
  vLAcc = vRAcc = nowUs - LIMIT_DEBOUNCE_US;
  PCICR |= _BV(PCIE2);
  PCMSK2 |= _BV(PCINT20) | _BV(PCINT21);
  interrupts();
#else
  leftCtr.begin(LEFT_LIMIT_PIN, nowUs);
  rightCtr.begin(RIGHT_LIMIT_PIN, nowUs);
#endif
}

void updateWheelCounters(uint32_t nowUs) {
#if HAS_AVR_PCINT
  (void)nowUs;
#else
  leftCtr.update(nowUs);
  rightCtr.update(nowUs);
#endif
}

void snapshotWheelCounters() {
#if HAS_AVR_PCINT
  noInterrupts();
  wheel.leftPressed = vLP;
  wheel.rightPressed = vRP;
  wheel.leftNewPress = vLN;
  wheel.rightNewPress = vRN;
  wheel.leftCount = vLC;
  wheel.rightCount = vRC;
  wheel.leftDtUs = vLdt;
  wheel.rightDtUs = vRdt;
  vLN = vRN = false;
  interrupts();
#else
  wheel.leftPressed = leftCtr.stable;
  wheel.rightPressed = rightCtr.stable;
  wheel.leftNewPress = leftCtr.newPress;
  wheel.rightNewPress = rightCtr.newPress;
  wheel.leftCount = leftCtr.count;
  wheel.rightCount = rightCtr.count;
  wheel.leftDtUs = leftCtr.intervalUs;
  wheel.rightDtUs = rightCtr.intervalUs;
  leftCtr.newPress = rightCtr.newPress = false;
#endif
}

/* ================================ Control ================================ */

void setPiston(bool on, uint32_t nowUs) {
  if (roverDone) on = false;
  if (pistonOn == on) return;

  pistonOn = on;
  digitalWrite(PISTON_PIN, on ? HIGH : LOW);
  if (!on) pistonLastOffUs = nowUs;
}

void updatePiston(uint32_t nowMs, uint32_t nowUs) {
  if (roverDone) {
    setPiston(false, nowUs);
    return;
  }

  uint32_t elapsedMs = nowMs - pistonStartMs;
  if (elapsedMs >= PISTON_PERIOD_MS) {
    pistonStartMs = nowMs;
    pistonCycles++;
    elapsedMs = 0;
  }

  setPiston(elapsedMs < PISTON_ON_MS, nowUs);
}

bool compassSafe(uint32_t nowUs) {
  return !pistonOn && (uint32_t)(nowUs - pistonLastOffUs) >= COMPASS_SETTLE_US;
}

void updateCompassIfSafe(uint32_t nowUs, uint32_t nowMs) {
  if (!compassSafe(nowUs)) return;

  lastCompassDeg = readCompassHeadingDeg();
  headingErrDeg = wrap180(lastCompassDeg - targetHeadingDeg);
  lastCompassMs = nowMs;
  compassValid = true;
  compassUpdatedSinceDecision = true;
}

void updateTurnGeometryFromLimitSwitches() {
  uint32_t leftTicks = wheel.leftCount - turnStartLeftCount;
  uint32_t rightTicks = wheel.rightCount - turnStartRightCount;
  float dL = (float)leftTicks * TICK_M;
  float dR = (float)rightTicks * TICK_M;
  float diff = fabs(dL - dR);
  float centerArc = 0.5f * (dL + dR);

  if (diff > 0.0001f) {
    float thetaRad = diff / TRACK_WIDTH_M;
    turnWheelAngleDeg = thetaRad * 180.0f / PI;
    turnRadiusM = centerArc / thetaRad;
  } else {
    turnWheelAngleDeg = 0.0f;
    turnRadiusM = NAN;
  }
}

void beginRightTurn() {
  phase = PHASE_TURN_RIGHT;
  turnStartLeftCount = wheel.leftCount;
  turnStartRightCount = wheel.rightCount;

  resetSegmentDistanceCounts();
  resetPidState();

  targetHeadingDeg = rightTurnTargetHeadingDeg;
  headingErrDeg = wrap180(lastCompassDeg - targetHeadingDeg);
  turnProgressDeg = 0.0f;
  turnRadiusM = NAN;
  turnWheelAngleDeg = NAN;
  turnDoneConfirmCount = 0;
  compassUpdatedSinceDecision = false;

  writeServoDeg(TURN_SERVO_DEG);
}

void beginSecondStraight() {
  phase = PHASE_STRAIGHT_2;
  resetSegmentDistanceCounts();
  resetPidState();

  targetHeadingDeg = rightTurnTargetHeadingDeg;
  headingErrDeg = wrap180(lastCompassDeg - targetHeadingDeg);
  turnDoneConfirmCount = 0;
  compassUpdatedSinceDecision = false;

  writeServoDeg(SERVO_CENTER_TRIM_DEG);
}

void finishRover() {
  phase = PHASE_DONE;
  roverDone = true;
  resetPidState();
  writeServoDeg(SERVO_CENTER_TRIM_DEG);
}

void updateDistanceFromLimitSwitches() {
  uint32_t leftTicks = wheel.leftCount - startLeftCount;
  uint32_t rightTicks = wheel.rightCount - startRightCount;
  distanceM = 0.5f * (float)(leftTicks + rightTicks) * TICK_M;

  if (phase == PHASE_TURN_RIGHT) updateTurnGeometryFromLimitSwitches();

  float targetDistanceM = currentStraightTargetDistanceM();
  if (isnan(targetDistanceM) || distanceM < targetDistanceM) return;

  if (phase == PHASE_STRAIGHT_1) beginRightTurn();
  else if (phase == PHASE_STRAIGHT_2) finishRover();
}

void updateTurnDecision() {
  resetPidState();
  writeServoDeg(TURN_SERVO_DEG);

  if (!compassUpdatedSinceDecision) return;
  compassUpdatedSinceDecision = false;

  turnProgressDeg = turnProgressFromInitialHeading();
  bool turnDone = !isnan(turnProgressDeg) &&
                  turnProgressDeg >= TURN_ANGLE_DEG - TURN_DONE_TOL_DEG;
  turnDoneConfirmCount = turnDone ? turnDoneConfirmCount + 1 : 0;

  if (turnDoneConfirmCount >= TURN_DONE_CONFIRM_SAMPLES) beginSecondStraight();
}

void updateStraightSteering(uint32_t nowMs) {
  bool freshCompass = compassValid && ((uint32_t)(nowMs - lastCompassMs) <= COMPASS_TIMEOUT_MS);
  if (!freshCompass) {
    lastPidDeg = 0.0f;
    writeServoDeg(SERVO_CENTER_TRIM_DEG);
    return;
  }

  float dt = DECISION_MS * 0.001f;
  float err = fabs(headingErrDeg) < HEADING_DEADBAND_DEG ? 0.0f : headingErrDeg;
  errIntegral = constrain(errIntegral + err * dt, -I_LIMIT_DEG_S, I_LIMIT_DEG_S);

  float dErr = (err - lastErrDeg) / dt;
  lastErrDeg = err;

  lastPidDeg = KP_HEADING * err + KI_HEADING * errIntegral + KD_HEADING * dErr;
  lastPidDeg = constrain(lastPidDeg, -PID_LIMIT_DEG, PID_LIMIT_DEG);

  int servoCommand = SERVO_CENTER_TRIM_DEG + STEERING_SIGN * roundToInt(lastPidDeg);
  writeServoDeg(servoCommand, true);
}

void updateSteering(uint32_t nowMs) {
  if (roverDone) {
    writeServoDeg(SERVO_CENTER_TRIM_DEG);
    return;
  }

  if (phase == PHASE_TURN_RIGHT) updateTurnDecision();
  else updateStraightSteering(nowMs);
}

/* ================================ Logging ================================ */

void printHeader() {
  Serial.println(F("t_ms,piston,cycle,Lcnt,Rcnt,dist_m,target_deg,mag_deg,err_deg,pid_deg,servo_deg,done,phase,turn_target_deg,turn_progress_deg,turn_radius_m,turn_wheel_angle_deg,turn_confirm,L_dt_ms,R_dt_ms"));
}

void printSample(uint32_t nowMs) {
  if (Serial.availableForWrite() < 48) return;

  Serial.print(nowMs); Serial.print(',');
  Serial.print(pistonOn ? 1 : 0); Serial.print(',');
  Serial.print(pistonCycles); Serial.print(',');
  Serial.print(wheel.leftCount); Serial.print(',');
  Serial.print(wheel.rightCount); Serial.print(',');
  Serial.print(distanceM, 4); Serial.print(',');
  Serial.print(targetHeadingDeg, 2); Serial.print(',');
  printFloatOrNan(lastCompassDeg, 2); Serial.print(',');
  Serial.print(headingErrDeg, 2); Serial.print(',');
  Serial.print(lastPidDeg, 2); Serial.print(',');
  Serial.print(lastServoDeg); Serial.print(',');
  Serial.print(roverDone ? 1 : 0); Serial.print(',');
  Serial.print((int)phase); Serial.print(',');
  printFloatOrNan(rightTurnTargetHeadingDeg, 2); Serial.print(',');
  printFloatOrNan(turnProgressDeg, 2); Serial.print(',');
  printFloatOrNan(turnRadiusM, 4); Serial.print(',');
  printFloatOrNan(turnWheelAngleDeg, 2); Serial.print(',');
  Serial.print(turnDoneConfirmCount); Serial.print(',');
  Serial.print(wheel.leftDtUs ? wheel.leftDtUs * 0.001f : -1.0f, 2); Serial.print(',');
  Serial.println(wheel.rightDtUs ? wheel.rightDtUs * 0.001f : -1.0f, 2);
}

void sample100Hz(uint32_t nowUs, uint32_t nowMs) {
  snapshotWheelCounters();
  updateDistanceFromLimitSwitches();
  updateCompassIfSafe(nowUs, nowMs);
  printSample(nowMs);
}

/* ================================= Setup ================================= */

void setup() {
  pinMode(PISTON_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(PISTON_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);

  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000UL);

  steeringServo.attach(SERVO_PIN);
  writeServoDeg(SERVO_CENTER_TRIM_DEG);

  if (!mag.init()) {
    failStop(F("ERROR: LIS3MDL compass not detected. Rover stopped."));
    return;
  }
  mag.enableDefault();

  calibrateCompass();

  initialHeadingDeg = averageCompassHeading(25);
  targetHeadingDeg = initialHeadingDeg;
  rightTurnTargetHeadingDeg = wrap360(initialHeadingDeg + RIGHT_TURN_HEADING_SIGN * TURN_ANGLE_DEG);

  lastCompassDeg = targetHeadingDeg;
  headingErrDeg = 0.0f;
  lastCompassMs = millis();
  compassValid = true;

  uint32_t nowUs = micros();
  uint32_t nowMs = millis();

  beginWheelCounters(nowUs);
  snapshotWheelCounters();
  resetSegmentDistanceCounts();
  turnStartLeftCount = wheel.leftCount;
  turnStartRightCount = wheel.rightCount;

  phase = PHASE_STRAIGHT_1;
  roverDone = false;
  pistonLastOffUs = nowUs;
  pistonStartMs = nowMs;
  pistonCycles = 1;
  nextSampleUs = nowUs + SAMPLE_US;
  nextDecisionUs = nowUs + DECISION_US;

  printHeader();
  setPiston(true, nowUs);
}

/* ================================= Loop ================================== */

void loop() {
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();

  if (!systemHealthy) {
    blinkError(nowMs);
    return;
  }

  updateWheelCounters(nowUs);
  updatePiston(nowMs, nowUs);

  if (due(nowUs, nextSampleUs)) {
    advanceBy(nextSampleUs, SAMPLE_US, nowUs);
    sample100Hz(nowUs, nowMs);
  }

  if (due(nowUs, nextDecisionUs)) {
    advanceBy(nextDecisionUs, DECISION_US, nowUs);
    updateSteering(nowMs);
  }
}
