/*
  Rover_CompassHold_Compact.ino

  Control mode:
    - IMU/Kalman removed.
    - Startup compass calibration: 10 s. Rotate the rover during this window.
      Built-in LED D13 blinks at 2 Hz during calibration.
    - After calibration, the current compass heading becomes the target heading.
    - Compass PID holds target heading; wheel limit switches count distance only.
    - Drive forward 3.0 m, then stop piston and center steering.

  Timing:
    - 100 Hz sample/log cycle.
    - 10 Hz steering decision cycle.
    - Piston cycle fixed at 300 ms; no piston adjustment in controller.

  Hardware:
    - Piston MOSFET: D2
    - Steering servo: D3, hard-limited to 65..125 deg
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

/* ------------------------------- Config ---------------------------------- */

const uint8_t PISTON_PIN = 2, SERVO_PIN = 3, LEFT_LIMIT_PIN = 4, RIGHT_LIMIT_PIN = 5, STATUS_LED_PIN = 13;
const uint32_t SERIAL_BAUD = 1000000UL;

const uint16_t SAMPLE_MS = 10;                       // 100 Hz
const uint16_t DECISION_MS = 100;                    // 10 Hz
const uint16_t PISTON_PERIOD_MS = 2000;               // fixed piston cycle
const uint16_t PISTON_ON_MS = 150;
const uint16_t COMPASS_SETTLE_AFTER_OFF_MS = 20;     // avoid piston magnetic transient
const uint16_t COMPASS_CAL_MS = 10000;               // rotate rover for 10 s at startup
const uint16_t CAL_LED_TOGGLE_MS = 250;              // 2 Hz blink cycle: toggle every 250 ms
const uint16_t COMPASS_TIMEOUT_MS = 600;             // last valid compass sample may be reused briefly

const uint32_t SAMPLE_US = (uint32_t)SAMPLE_MS * 1000UL;
const uint32_t DECISION_US = (uint32_t)DECISION_MS * 1000UL;
const uint32_t COMPASS_SETTLE_US = (uint32_t)COMPASS_SETTLE_AFTER_OFF_MS * 1000UL;
const uint32_t LIMIT_DEBOUNCE_US = 3000UL;

const int SERVO_MIN_DEG = 80;
const int SERVO_MAX_DEG = 105;
const int SERVO_CENTER_TRIM_DEG = 90;                // tune if actual center is not 90
const int STEERING_SIGN = -1;                         // change to -1 if correction direction is reversed

const float TARGET_DISTANCE_M = 1.0f;
const float TICK_M = 3.45f * 3.14f * 0.01f;          // one press = 3.45*pi cm ~= 0.10833 m

// Compass heading calibration presets. Runtime startup calibration overwrites these variables.
const float MAG_MIN_X_PRESET = -1162.0f, MAG_MAX_X_PRESET = 1996.0f;
const float MAG_MIN_Y_PRESET =  1760.0f, MAG_MAX_Y_PRESET = 4932.0f;
const float MAG_MIN_Z_PRESET = -1000.0f, MAG_MAX_Z_PRESET = 1000.0f;
const float MIN_USABLE_MAG_SPAN = 100.0f;
const float HEADING_OFFSET_DEG = 0.0f;
const float MAG_X_SIGN = 1.0f, MAG_Y_SIGN = 1.0f;     // flip one if heading is mirrored

// Heading-hold PID. Error unit is degree; output unit is servo degree.
const float KP_HEADING = 1.20f;
const float KI_HEADING = 0.01f;
const float KD_HEADING = 0.08f;
const float HEADING_DEADBAND_DEG = 2.0f;
const float I_LIMIT_DEG_S = 80.0f;
const float PID_LIMIT_DEG = 30.0f;

/* ------------------------------- Globals --------------------------------- */

LIS3MDL mag;
Servo steeringServo;

struct WheelSnap { bool lp, rp, ln, rn; uint32_t lc, rc, ldt, rdt; } wheel;

bool systemHealthy = true, pistonOn = false, roverDone = false, statusLed = false, compassValid = false;
uint32_t pistonStartMs = 0, pistonLastOffUs = 0, pistonCycles = 0;
uint32_t nextSampleUs = 0, nextDecisionUs = 0, lastBlinkMs = 0, lastCompassMs = 0;
uint32_t startLeftCount = 0, startRightCount = 0;

float magMinX = MAG_MIN_X_PRESET, magMaxX = MAG_MAX_X_PRESET;
float magMinY = MAG_MIN_Y_PRESET, magMaxY = MAG_MAX_Y_PRESET;
float magMinZ = MAG_MIN_Z_PRESET, magMaxZ = MAG_MAX_Z_PRESET;
float targetHeadingDeg = NAN, lastCompassDeg = NAN, headingErrDeg = 0.0f;
float distanceM = 0.0f, errIntegral = 0.0f, lastErrDeg = 0.0f, lastPidDeg = 0.0f;
int lastServoDeg = SERVO_CENTER_TRIM_DEG;

/* ------------------------------- Helpers --------------------------------- */

bool due(uint32_t now, uint32_t scheduled) { return (int32_t)(now - scheduled) >= 0; }
void advanceBy(uint32_t &t, uint32_t p, uint32_t now) { t += p; if ((int32_t)(now - t) >= (int32_t)p) t = now + p; }
float wrap360(float a) { while (a >= 360.0f) a -= 360.0f; while (a < 0.0f) a += 360.0f; return a; }
float wrap180(float a) { while (a > 180.0f) a -= 360.0f; while (a <= -180.0f) a += 360.0f; return a; }
int roundToInt(float v) { return (int)(v + (v >= 0.0f ? 0.5f : -0.5f)); }
int clampServo(int deg) { return constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG); }

float normAxis(float raw, float lo, float hi) {
  float half = 0.5f * (hi - lo);
  if (fabs(half) < 1.0f) half = 1.0f;
  return (raw - 0.5f * (lo + hi)) / half;
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

void writeServoDeg(int deg) {
  lastServoDeg = clampServo(deg);
  steeringServo.write(lastServoDeg);
}

void failStop(const __FlashStringHelper *msg) {
  systemHealthy = false;
  digitalWrite(PISTON_PIN, LOW);
  writeServoDeg(SERVO_CENTER_TRIM_DEG);
  Serial.println(msg);
}

void blinkError(uint32_t nowMs) {
  if (nowMs - lastBlinkMs < 250UL) return;
  lastBlinkMs = nowMs;
  statusLed = !statusLed;
  digitalWrite(STATUS_LED_PIN, statusLed ? HIGH : LOW);
}

/* -------------------------- Compass Calibration --------------------------- */

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

void calibrateCompass10s() {
  digitalWrite(PISTON_PIN, LOW);
  writeServoDeg(SERVO_CENTER_TRIM_DEG);

  float minX =  32767.0f, minY =  32767.0f, minZ =  32767.0f;
  float maxX = -32768.0f, maxY = -32768.0f, maxZ = -32768.0f;
  uint32_t startMs = millis(), nextBlink = startMs, nextReadUs = micros();

  Serial.println(F("CAL_START: rotate rover through as many headings as possible for 10 s."));

  while ((uint32_t)(millis() - startMs) < COMPASS_CAL_MS) {
    uint32_t nowMs = millis();
    uint32_t nowUs = micros();

    if ((uint32_t)(nowMs - nextBlink) >= CAL_LED_TOGGLE_MS) {
      nextBlink += CAL_LED_TOGGLE_MS;
      statusLed = !statusLed;
      digitalWrite(STATUS_LED_PIN, statusLed ? HIGH : LOW);
    }

    if (due(nowUs, nextReadUs)) {
      advanceBy(nextReadUs, SAMPLE_US, nowUs);
      mag.read();

      if (mag.m.x < minX) minX = mag.m.x;
      if (mag.m.y < minY) minY = mag.m.y;
      if (mag.m.z < minZ) minZ = mag.m.z;

      if (mag.m.x > maxX) maxX = mag.m.x;
      if (mag.m.y > maxY) maxY = mag.m.y;
      if (mag.m.z > maxZ) maxZ = mag.m.z;
    }
  }

  digitalWrite(STATUS_LED_PIN, LOW);
  statusLed = false;

  bool xyUsable = (maxX - minX >= MIN_USABLE_MAG_SPAN) && (maxY - minY >= MIN_USABLE_MAG_SPAN);
  if (xyUsable) {
    magMinX = minX; magMaxX = maxX;
    magMinY = minY; magMaxY = maxY;
    magMinZ = minZ; magMaxZ = maxZ;
  } else {
    Serial.println(F("WARN: compass calibration span too small; using previous preset values."));
  }
  printCalibrationValues();
}

float averageCompassHeading(uint8_t n) {
  float s = 0.0f, c = 0.0f;
  for (uint8_t i = 0; i < n; i++) {
    float h = readCompassHeadingDeg() * PI / 180.0f;
    s += sin(h); c += cos(h);
    delay(10);
  }
  return wrap360(atan2(s, c) * 180.0f / PI);
}

/* --------------------------- Wheel Counters ------------------------------- */

#if !HAS_AVR_PCINT
struct PolledCounter {
  uint8_t pin; bool raw, stable, newPress; uint32_t changedUs, lastPressUs, intervalUs, count;
  void begin(uint8_t p, uint32_t nowUs) {
    pin = p; raw = stable = (digitalRead(pin) == LOW); newPress = false;
    changedUs = nowUs; lastPressUs = intervalUs = count = 0;
  }
  void update(uint32_t nowUs) {
    bool r = (digitalRead(pin) == LOW);
    if (r != raw) { raw = r; changedUs = nowUs; }
    if (r != stable && nowUs - changedUs >= LIMIT_DEBOUNCE_US) {
      stable = r;
      if (stable) { intervalUs = lastPressUs ? nowUs - lastPressUs : 0; lastPressUs = nowUs; count++; newPress = true; }
    }
  }
};
PolledCounter leftCtr, rightCtr;
#endif

#if HAS_AVR_PCINT
const uint8_t LEFT_MASK = _BV(PD4), RIGHT_MASK = _BV(PD5);
volatile uint8_t vLastD = 0;
volatile bool vLP = false, vRP = false, vLN = false, vRN = false;
volatile uint32_t vLC = 0, vRC = 0, vLLast = 0, vRLast = 0, vLAcc = 0, vRAcc = 0, vLdt = 0, vRdt = 0;

void acceptL(uint32_t nowUs) { if (nowUs - vLAcc < LIMIT_DEBOUNCE_US) return; vLdt = vLLast ? nowUs - vLLast : 0; vLLast = vLAcc = nowUs; vLC++; vLN = true; }
void acceptR(uint32_t nowUs) { if (nowUs - vRAcc < LIMIT_DEBOUNCE_US) return; vRdt = vRLast ? nowUs - vRLast : 0; vRLast = vRAcc = nowUs; vRC++; vRN = true; }

ISR(PCINT2_vect) {
  uint8_t d = PIND, changed = d ^ vLastD;
  uint32_t nowUs = micros();
  if (changed & LEFT_MASK)  { vLP = ((d & LEFT_MASK) == 0);  if (vLP) acceptL(nowUs); }
  if (changed & RIGHT_MASK) { vRP = ((d & RIGHT_MASK) == 0); if (vRP) acceptR(nowUs); }
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
#if !HAS_AVR_PCINT
  leftCtr.update(nowUs);
#else
  (void)nowUs;
#endif
}

void snapshotWheelCounters() {
#if HAS_AVR_PCINT
  noInterrupts();
  wheel.lp = vLP; wheel.rp = vRP; wheel.ln = vLN; wheel.rn = vRN;
  wheel.lc = vLC; wheel.rc = vRC; wheel.ldt = vLdt; wheel.rdt = vRdt;
  vLN = vRN = false;
  interrupts();
#else
  wheel.lp = leftCtr.stable; wheel.rp = rightCtr.stable; wheel.ln = leftCtr.newPress; wheel.rn = rightCtr.newPress;
  wheel.lc = leftCtr.count; wheel.rc = rightCtr.count; wheel.ldt = leftCtr.intervalUs; wheel.rdt = rightCtr.intervalUs;
  leftCtr.newPress = rightCtr.newPress = false;
#endif
}

/* ----------------------------- Control ------------------------------------ */

void setPiston(bool on, uint32_t nowUs) {
  if (roverDone) on = false;
  if (pistonOn == on) return;
  pistonOn = on;
  digitalWrite(PISTON_PIN, on ? HIGH : LOW);
  if (!on) pistonLastOffUs = nowUs;
}

void updatePiston(uint32_t nowMs, uint32_t nowUs) {
  if (roverDone) { setPiston(false, nowUs); return; }
  uint32_t e = nowMs - pistonStartMs;
  if (e >= PISTON_PERIOD_MS) { pistonStartMs = nowMs; pistonCycles++; e = 0; }
  setPiston(e < PISTON_ON_MS, nowUs);
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
}

void updateDistanceFromLimitSwitches() {
  uint32_t lTicks = wheel.lc - startLeftCount;
  uint32_t rTicks = wheel.rc - startRightCount;
  distanceM = 0.5f * (float)(lTicks + rTicks) * TICK_M;

  if (distanceM >= TARGET_DISTANCE_M && !roverDone) {
    roverDone = true;
    errIntegral = 0.0f;
    lastPidDeg = 0.0f;
    writeServoDeg(SERVO_CENTER_TRIM_DEG);
  }
}

void updateSteeringPID(uint32_t nowMs) {
  if (roverDone) { writeServoDeg(SERVO_CENTER_TRIM_DEG); return; }

  bool freshCompass = compassValid && ((uint32_t)(nowMs - lastCompassMs) <= COMPASS_TIMEOUT_MS);
  if (!freshCompass) {                          // no reliable heading yet: fail neutral, not blind steering
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
  writeServoDeg(SERVO_CENTER_TRIM_DEG + STEERING_SIGN * roundToInt(lastPidDeg));
}

/* ------------------------------- Logging ---------------------------------- */

void printHeader() {
  Serial.println(F("t_ms,piston,cycle,Lcnt,Rcnt,dist_m,target_deg,mag_deg,err_deg,pid_deg,servo_deg,done,L_dt_ms,R_dt_ms"));
}

void printSample(uint32_t nowMs) {
  if (Serial.availableForWrite() < 48) return;
  Serial.print(nowMs); Serial.print(',');
  Serial.print(pistonOn ? 1 : 0); Serial.print(',');
  Serial.print(pistonCycles); Serial.print(',');
  Serial.print(wheel.lc); Serial.print(',');
  Serial.print(wheel.rc); Serial.print(',');
  Serial.print(distanceM, 4); Serial.print(',');
  Serial.print(targetHeadingDeg, 2); Serial.print(',');
  if (isnan(lastCompassDeg)) Serial.print(F("nan")); else Serial.print(lastCompassDeg, 2);
  Serial.print(',');
  Serial.print(headingErrDeg, 2); Serial.print(',');
  Serial.print(lastPidDeg, 2); Serial.print(',');
  Serial.print(lastServoDeg); Serial.print(',');
  Serial.print(roverDone ? 1 : 0); Serial.print(',');
  Serial.print(wheel.ldt ? wheel.ldt * 0.001f : -1.0f, 2); Serial.print(',');
  Serial.println(wheel.rdt ? wheel.rdt * 0.001f : -1.0f, 2);
}

void sample100Hz(uint32_t nowUs, uint32_t nowMs) {
  snapshotWheelCounters();
  updateDistanceFromLimitSwitches();
  updateCompassIfSafe(nowUs, nowMs);
  printSample(nowMs);
}

/* -------------------------------- Setup ----------------------------------- */

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

  if (!mag.init()) { failStop(F("ERROR: LIS3MDL compass not detected. Rover stopped.")); return; }
  mag.enableDefault();

  calibrateCompass10s();

  targetHeadingDeg = averageCompassHeading(25);       // heading at the end of calibration becomes target
  lastCompassDeg = targetHeadingDeg;
  headingErrDeg = 0.0f;
  lastCompassMs = millis();
  compassValid = true;

  uint32_t nowUs = micros(), nowMs = millis();
  beginWheelCounters(nowUs);
  snapshotWheelCounters();
  startLeftCount = wheel.lc;
  startRightCount = wheel.rc;

  pistonLastOffUs = nowUs;
  pistonStartMs = nowMs;
  pistonCycles = 1;
  nextSampleUs = nowUs + SAMPLE_US;
  nextDecisionUs = nowUs + DECISION_US;

  printHeader();
  setPiston(true, nowUs);
}

void loop() {
  uint32_t nowUs = micros(), nowMs = millis();
  if (!systemHealthy) { blinkError(nowMs); return; }

  updateWheelCounters(nowUs);
  updatePiston(nowMs, nowUs);

  if (due(nowUs, nextSampleUs)) {
    advanceBy(nextSampleUs, SAMPLE_US, nowUs);
    sample100Hz(nowUs, nowMs);
  }

  if (due(nowUs, nextDecisionUs)) {
    advanceBy(nextDecisionUs, DECISION_US, nowUs);
    updateSteeringPID(nowMs);
  }
}
