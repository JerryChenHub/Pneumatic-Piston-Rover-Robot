/*
  Piston Rover control sketch

  Functional structure:
    - Startup serial menu: press 1 to calibrate, press 0 to skip calibration.
    - 10 s non-blocking compass calibration with D13 blinking at a 10 Hz full blink cycle.
    - Sampling stage runs continuously and rejects compass readings while the piston is ON.
    - Execution stage runs every 100 ms and performs piston timing, steering control, and logging.
    - Piston sequence is 10 cycles of 100 ms ON and 400 ms OFF.

  The compass is not sampled during piston-ON windows because the piston interferes with the magnetometer.
  During those windows, the steering controller uses the most recent clean compass heading from piston-OFF time.
*/

#include <Wire.h>
#include <Servo.h>
#include <LIS3MDL.h>
#include <math.h>

#define ENABLE_IMU_PRINT 1
#if ENABLE_IMU_PRINT
#include <LSM6.h>
#endif

// ------------------------- Hardware map -------------------------
// Rover electrical interface from Agent.md. Limit switches are configured safely but are not used for control.
const byte PISTON_PIN = 2;
const byte STEERING_SERVO_PIN = 3;
const byte LEFT_REAR_LIMIT_PIN = 4;
const byte RIGHT_REAR_LIMIT_PIN = 5;
const byte CAL_LED_PIN = 13;

// ------------------------- Timing plan --------------------------
// All scheduling uses millis(); there are no delay() calls or blocking serial waits.
const unsigned long MENU_PROMPT_PERIOD_MS = 2000UL;
const unsigned long CALIBRATION_TIME_MS = 10000UL;
const unsigned long CAL_LED_TOGGLE_MS = 50UL;
const unsigned long CAL_SAMPLE_PERIOD_MS = 20UL;
const unsigned long COMPASS_SAMPLE_PERIOD_MS = 20UL;
const unsigned long IMU_SAMPLE_PERIOD_MS = 20UL;
const unsigned long CONTROL_PERIOD_MS = 100UL;
const unsigned long COMPASS_SETTLE_AFTER_PISTON_OFF_MS = 20UL;

// ------------------------- Piston cycle -------------------------
// One propulsion cycle is 100 ms ON followed by 400 ms OFF, repeated ten times.
const byte TOTAL_PISTON_CYCLES = 10;
const unsigned long PISTON_ON_MS = 100UL;
const unsigned long PISTON_OFF_MS = 400UL;

// ------------------------- Compass calibration ------------------
// The pre-calibration values are used when startup calibration is skipped or rejected.
const float PRE_MIN_X = -7554.0f;
const float PRE_MAX_X = -7410.0f;
const float PRE_MIN_Y =   697.0f;
const float PRE_MAX_Y =   886.0f;
const float PRE_MIN_Z =  9057.0f;
const float PRE_MAX_Z =  9374.0f;

float minX = PRE_MIN_X, maxX = PRE_MAX_X;
float minY = PRE_MIN_Y, maxY = PRE_MAX_Y;
float minZ = PRE_MIN_Z, maxZ = PRE_MAX_Z;

float calMinX, calMaxX;
float calMinY, calMaxY;
float calMinZ, calMaxZ;
unsigned int calSampleCount = 0;

const float MIN_VALID_CAL_SPAN = 20.0f;
const float HEADING_X_SIGN = 1.0f;
const float HEADING_Y_SIGN = 1.0f;
const float HEADING_OFFSET_DEG = 0.0f;
const float DEG_TO_RAD_F = 0.017453292519943295f;
const float RAD_TO_DEG_F = 57.29577951308232f;

// ------------------------- Steering control ---------------------
// Steering is asymmetric and always constrained to 65-125 degrees to protect the linkage.
const int SERVO_MIN_DEG = 65;
const int SERVO_CENTER_DEG = 90;
const int SERVO_MAX_DEG = 125;

const float STEERING_SIGN = 1.0f;
const float STEER_KP_LEFT = 0.75f;
const float STEER_KP_RIGHT = 0.55f;
const float HEADING_DEADBAND_DEG = 2.0f;

// ------------------------- Runtime state ------------------------
// State variables hold the serial menu, calibration, sensor sampling, actuator phase, and execution timing.
Servo steeringServo;
LIS3MDL mag;

#if ENABLE_IMU_PRINT
LSM6 imu;
bool imuAvailable = false;
bool haveImuSample = false;
long imuAx = 0, imuAy = 0, imuAz = 0;
long imuGx = 0, imuGy = 0, imuGz = 0;
unsigned long lastImuSampleMs = 0;
#endif

enum RoverState {
  STATE_WAIT_CAL_CHOICE,
  STATE_CALIBRATING,
  STATE_WAIT_START,
  STATE_RUNNING,
  STATE_DONE,
  STATE_COMPASS_ERROR
};

RoverState state = STATE_WAIT_CAL_CHOICE;

unsigned long lastMenuPromptMs = 0;
unsigned long calibrationStartMs = 0;
unsigned long lastCalSampleMs = 0;
unsigned long lastLedToggleMs = 0;
bool ledState = false;

unsigned long lastCompassSampleMs = 0;
unsigned long lastExecutionMs = 0;

float sinSum = 0.0f;
float cosSum = 0.0f;
unsigned int cleanSampleCount = 0;

bool haveCleanHeading = false;
float lastCleanHeadingDeg = 0.0f;
float targetHeadingDeg = 0.0f;
float lastHeadingErrorDeg = 0.0f;
int lastServoCommandDeg = SERVO_CENTER_DEG;

float lastCompassRawX = 0.0f;
float lastCompassRawY = 0.0f;
float lastCompassRawZ = 0.0f;
float lastControlHeadingDeg = 0.0f;

byte pistonCyclesStarted = 0;
byte pistonCyclesFinished = 0;
unsigned long pistonPhaseStartMs = 0;
unsigned long pistonLastOffMs = 0;
bool pistonIsOn = false;
bool donePrinted = false;

// ------------------------- Angle helpers ------------------------
// Circular heading math utilities for 0-360 and signed +/-180 degree values.
float wrap360(float angleDeg) {
  while (angleDeg >= 360.0f) angleDeg -= 360.0f;
  while (angleDeg < 0.0f) angleDeg += 360.0f;
  return angleDeg;
}

float wrap180(float angleDeg) {
  return wrap360(angleDeg + 180.0f) - 180.0f;
}

// ------------------------- Serial menu helpers ------------------
// Menu handling drains available serial bytes only; it never waits for input.
char readSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '0' || c == '1') return c;
  }
  return '\0';
}

void clearSerialInput() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void printCalibrationChoicePrompt() {
  Serial.println();
  Serial.println(F("Startup command:"));
  Serial.println(F("  Send 1 to run 10 s compass calibration."));
  Serial.println(F("  Send 0 to skip calibration and use the stored pre-calibration values."));
}

void printStartPrompt() {
  Serial.println();
  Serial.println(F("Send 1 to start the piston rover run."));
}

void repeatMenuPrompt(unsigned long nowMs, void (*promptFn)()) {
  if (nowMs - lastMenuPromptMs >= MENU_PROMPT_PERIOD_MS) {
    lastMenuPromptMs = nowMs;
    promptFn();
  }
}

// ------------------------- Actuator helpers ---------------------
// Centralized actuator writes keep the piston state and servo safety limit consistent.
void writeSteeringSafe(int angleDeg) {
  angleDeg = constrain(angleDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  lastServoCommandDeg = angleDeg;
  steeringServo.write(angleDeg);
}

void setPistonOff(unsigned long nowMs) {
  pistonIsOn = false;
  pistonLastOffMs = nowMs;
  digitalWrite(PISTON_PIN, LOW);
}

void setPistonOn(unsigned long nowMs) {
  pistonIsOn = true;
  digitalWrite(PISTON_PIN, HIGH);
}

void allStop(unsigned long nowMs) {
  setPistonOff(nowMs);
  writeSteeringSafe(SERVO_CENTER_DEG);
}

// ------------------------- Compass helpers ----------------------
// LIS3MDL raw X/Y values are normalized with the active calibration and converted to a planar heading.
void readCompassRaw(float &x, float &y, float &z) {
  mag.read();
  x = (float)mag.m.x;
  y = (float)mag.m.y;
  z = (float)mag.m.z;
}

float normalizedAxis(float raw, float minVal, float maxVal) {
  const float center = 0.5f * (minVal + maxVal);
  float halfRange = 0.5f * (maxVal - minVal);
  if (fabs(halfRange) < 1.0f) halfRange = 1.0f;
  return (raw - center) / halfRange;
}

float headingFromRaw(float rawX, float rawY) {
  const float x = HEADING_X_SIGN * normalizedAxis(rawX, minX, maxX);
  const float y = HEADING_Y_SIGN * normalizedAxis(rawY, minY, maxY);
  return wrap360(atan2(y, x) * RAD_TO_DEG_F + HEADING_OFFSET_DEG);
}

float readHeadingDeg() {
  float rawX, rawY, rawZ;
  readCompassRaw(rawX, rawY, rawZ);
  lastCompassRawX = rawX;
  lastCompassRawY = rawY;
  lastCompassRawZ = rawZ;
  return headingFromRaw(rawX, rawY);
}

// ------------------------- IMU logging helpers ------------------
// The IMU is logged only. It is not used by the steering or piston controller.
void initializeImuLogging() {
#if ENABLE_IMU_PRINT
  imuAvailable = imu.init();
  if (imuAvailable) {
    imu.enableDefault();
    Serial.println(F("LSM6 IMU logging enabled."));
  } else {
    Serial.println(F("LSM6 IMU not detected; IMU field will print not_detected."));
  }
#else
  Serial.println(F("IMU logging disabled at compile time."));
#endif
}

void sampleImuStage(unsigned long nowMs) {
#if ENABLE_IMU_PRINT
  if (!imuAvailable) return;
  if (nowMs - lastImuSampleMs < IMU_SAMPLE_PERIOD_MS) return;

  lastImuSampleMs = nowMs;
  imu.read();
  imuAx = imu.a.x;
  imuAy = imu.a.y;
  imuAz = imu.a.z;
  imuGx = imu.g.x;
  imuGy = imu.g.y;
  imuGz = imu.g.z;
  haveImuSample = true;
#endif
}

void printImuFields() {
#if ENABLE_IMU_PRINT
  if (!imuAvailable) {
    Serial.print(F(" IMU=not_detected"));
    return;
  }
  if (!haveImuSample) {
    Serial.print(F(" IMU=waiting"));
    return;
  }

  Serial.print(F(" IMU_ax=")); Serial.print(imuAx);
  Serial.print(F(" IMU_ay=")); Serial.print(imuAy);
  Serial.print(F(" IMU_az=")); Serial.print(imuAz);
  Serial.print(F(" IMU_gx=")); Serial.print(imuGx);
  Serial.print(F(" IMU_gy=")); Serial.print(imuGy);
  Serial.print(F(" IMU_gz=")); Serial.print(imuGz);
#else
  Serial.print(F(" IMU=disabled"));
#endif
}

// ------------------------- Calibration stage --------------------
// The calibration window samples extrema for 10 s while D13 blinks at a 10 Hz full blink cycle.
void beginCalibration(unsigned long nowMs) {
  calMinX =  1.0e9f; calMaxX = -1.0e9f;
  calMinY =  1.0e9f; calMaxY = -1.0e9f;
  calMinZ =  1.0e9f; calMaxZ = -1.0e9f;
  calSampleCount = 0;

  calibrationStartMs = nowMs;
  lastCalSampleMs = nowMs;
  lastLedToggleMs = nowMs;
  ledState = false;
  digitalWrite(CAL_LED_PIN, LOW);

  allStop(nowMs);
  Serial.println();
  Serial.println(F("Compass calibration started: rotate the rover for 10 seconds."));
  Serial.println(F("Pin 13 blinks during calibration. The controller remains non-blocking."));
}

void usePreCalibrationValues() {
  minX = PRE_MIN_X; maxX = PRE_MAX_X;
  minY = PRE_MIN_Y; maxY = PRE_MAX_Y;
  minZ = PRE_MIN_Z; maxZ = PRE_MAX_Z;
}

void sampleCalibrationCompass() {
  float rawX, rawY, rawZ;
  readCompassRaw(rawX, rawY, rawZ);

  if (rawX < calMinX) calMinX = rawX;
  if (rawX > calMaxX) calMaxX = rawX;
  if (rawY < calMinY) calMinY = rawY;
  if (rawY > calMaxY) calMaxY = rawY;
  if (rawZ < calMinZ) calMinZ = rawZ;
  if (rawZ > calMaxZ) calMaxZ = rawZ;

  calSampleCount++;
}

void printCalibrationValues() {
  Serial.println(F("Compass calibration parameters now in use:"));
  Serial.print(F("float minX = ")); Serial.print(minX, 1);
  Serial.print(F(", maxX = ")); Serial.print(maxX, 1); Serial.println(F(";"));
  Serial.print(F("float minY = ")); Serial.print(minY, 1);
  Serial.print(F(", maxY = ")); Serial.print(maxY, 1); Serial.println(F(";"));
  Serial.print(F("float minZ = ")); Serial.print(minZ, 1);
  Serial.print(F(", maxZ = ")); Serial.print(maxZ, 1); Serial.println(F(";"));
  Serial.print(F("samples=")); Serial.print(calSampleCount);
  Serial.print(F(" spanX=")); Serial.print(maxX - minX, 1);
  Serial.print(F(" spanY=")); Serial.print(maxY - minY, 1);
  Serial.print(F(" spanZ=")); Serial.println(maxZ - minZ, 1);
}

void finishCalibration(unsigned long nowMs) {
  digitalWrite(CAL_LED_PIN, LOW);
  ledState = false;

  const float spanX = calMaxX - calMinX;
  const float spanY = calMaxY - calMinY;

  if (calSampleCount > 10 && spanX > MIN_VALID_CAL_SPAN && spanY > MIN_VALID_CAL_SPAN) {
    minX = calMinX; maxX = calMaxX;
    minY = calMinY; maxY = calMaxY;
    minZ = calMinZ; maxZ = calMaxZ;
    Serial.println(F("Compass calibration accepted."));
  } else {
    usePreCalibrationValues();
    Serial.println(F("Compass calibration rejected; using stored pre-calibration values."));
  }

  printCalibrationValues();
  allStop(nowMs);
  clearSerialInput();
  lastMenuPromptMs = nowMs;
  printStartPrompt();
  state = STATE_WAIT_START;
}

void calibrationStage(unsigned long nowMs) {
  if (nowMs - lastLedToggleMs >= CAL_LED_TOGGLE_MS) {
    lastLedToggleMs = nowMs;
    ledState = !ledState;
    digitalWrite(CAL_LED_PIN, ledState ? HIGH : LOW);
  }

  if (nowMs - lastCalSampleMs >= CAL_SAMPLE_PERIOD_MS) {
    lastCalSampleMs = nowMs;
    sampleCalibrationCompass();
  }

  if (nowMs - calibrationStartMs >= CALIBRATION_TIME_MS) {
    finishCalibration(nowMs);
  }
}

void waitForCalibrationChoice(unsigned long nowMs) {
  const char cmd = readSerialCommand();

  if (cmd == '1') {
    beginCalibration(nowMs);
    state = STATE_CALIBRATING;
    return;
  }

  if (cmd == '0') {
    usePreCalibrationValues();
    calSampleCount = 0;
    Serial.println();
    Serial.println(F("Compass calibration skipped."));
    printCalibrationValues();
    allStop(nowMs);
    clearSerialInput();
    lastMenuPromptMs = nowMs;
    printStartPrompt();
    state = STATE_WAIT_START;
    return;
  }

  repeatMenuPrompt(nowMs, printCalibrationChoicePrompt);
}

void startRun(unsigned long nowMs);

void waitForStartCommand(unsigned long nowMs) {
  const char cmd = readSerialCommand();

  if (cmd == '1') {
    startRun(nowMs);
    return;
  }

  repeatMenuPrompt(nowMs, printStartPrompt);
}

// ------------------------- Sampling stage -----------------------
// Compass samples are accumulated only during clean piston-OFF windows; IMU samples are logged separately.
void resetSampleWindow() {
  sinSum = 0.0f;
  cosSum = 0.0f;
  cleanSampleCount = 0;
}

void sampleCompassStage(unsigned long nowMs) {
  if (pistonIsOn) return;
  if (nowMs - pistonLastOffMs < COMPASS_SETTLE_AFTER_PISTON_OFF_MS) return;
  if (nowMs - lastCompassSampleMs < COMPASS_SAMPLE_PERIOD_MS) return;

  lastCompassSampleMs = nowMs;
  const float headingDeg = readHeadingDeg();

  lastCleanHeadingDeg = headingDeg;
  haveCleanHeading = true;
  sinSum += sin(headingDeg * DEG_TO_RAD_F);
  cosSum += cos(headingDeg * DEG_TO_RAD_F);
  cleanSampleCount++;
}

void sampleStage(unsigned long nowMs) {
  sampleCompassStage(nowMs);
  sampleImuStage(nowMs);
}

bool headingForControl(float &headingDeg, bool &usedNewCleanSamples) {
  usedNewCleanSamples = false;

  if (cleanSampleCount > 0) {
    headingDeg = wrap360(atan2(sinSum, cosSum) * RAD_TO_DEG_F);
    lastCleanHeadingDeg = headingDeg;
    haveCleanHeading = true;
    usedNewCleanSamples = true;
    return true;
  }

  if (haveCleanHeading) {
    headingDeg = lastCleanHeadingDeg;
    return true;
  }

  return false;
}

// ------------------------- Execution stage ----------------------
// Every 100 ms, this stage advances piston phase, updates steering, and prints a one-line status packet.
void updatePistonSchedule(unsigned long nowMs) {
  if (pistonCyclesFinished >= TOTAL_PISTON_CYCLES) {
    setPistonOff(nowMs);
    state = STATE_DONE;
    return;
  }

  const unsigned long phaseDuration = pistonIsOn ? PISTON_ON_MS : PISTON_OFF_MS;
  if (nowMs - pistonPhaseStartMs < phaseDuration) return;

  pistonPhaseStartMs = nowMs;

  if (pistonIsOn) {
    setPistonOff(nowMs);
    return;
  }

  pistonCyclesFinished++;

  if (pistonCyclesFinished >= TOTAL_PISTON_CYCLES) {
    setPistonOff(nowMs);
    state = STATE_DONE;
    return;
  }

  pistonCyclesStarted++;
  setPistonOn(nowMs);
}

void updateSteeringFromHeading(float measuredHeadingDeg) {
  lastHeadingErrorDeg = wrap180(targetHeadingDeg - measuredHeadingDeg);
  float signedErrorDeg = STEERING_SIGN * lastHeadingErrorDeg;

  if (fabs(signedErrorDeg) < HEADING_DEADBAND_DEG) signedErrorDeg = 0.0f;

  const float kp = (signedErrorDeg < 0.0f) ? STEER_KP_LEFT : STEER_KP_RIGHT;
  const int commandDeg = (int)round((float)SERVO_CENTER_DEG + kp * signedErrorDeg);
  writeSteeringSafe(commandDeg);
}

void printExecutionParameters(unsigned long nowMs, bool usedNewCleanSamples, bool hadHeadingForControl) {
  const unsigned long phaseElapsedMs = nowMs - pistonPhaseStartMs;
  const unsigned long phaseTargetMs = pistonIsOn ? PISTON_ON_MS : PISTON_OFF_MS;

  Serial.print(F("exec_ms=")); Serial.print(nowMs);
  Serial.print(F(" cycle_started=")); Serial.print(pistonCyclesStarted);
  Serial.print(F(" cycle_finished=")); Serial.print(pistonCyclesFinished);
  Serial.print(F(" piston=")); Serial.print(pistonIsOn ? F("ON") : F("OFF"));
  Serial.print(F(" on_ms=")); Serial.print(PISTON_ON_MS);
  Serial.print(F(" off_ms=")); Serial.print(PISTON_OFF_MS);
  Serial.print(F(" phase_elapsed_ms=")); Serial.print(phaseElapsedMs);
  Serial.print(F(" phase_target_ms=")); Serial.print(phaseTargetMs);

  Serial.print(F(" compass_source="));
  if (!hadHeadingForControl) {
    Serial.print(F("none"));
  } else if (usedNewCleanSamples) {
    Serial.print(F("new_clean_off_window"));
  } else {
    Serial.print(F("last_clean"));
  }

  Serial.print(F(" compass_heading_deg=")); Serial.print(lastControlHeadingDeg, 2);
  Serial.print(F(" compass_target_deg=")); Serial.print(targetHeadingDeg, 2);
  Serial.print(F(" compass_error_deg=")); Serial.print(lastHeadingErrorDeg, 2);
  Serial.print(F(" compass_raw_x=")); Serial.print(lastCompassRawX, 0);
  Serial.print(F(" compass_raw_y=")); Serial.print(lastCompassRawY, 0);
  Serial.print(F(" compass_raw_z=")); Serial.print(lastCompassRawZ, 0);

  printImuFields();

  Serial.print(F(" steering_deg=")); Serial.print(lastServoCommandDeg);
  Serial.print(F(" steering_min=")); Serial.print(SERVO_MIN_DEG);
  Serial.print(F(" steering_max=")); Serial.print(SERVO_MAX_DEG);
  Serial.print(F(" state=")); Serial.println(state == STATE_DONE ? F("DONE") : F("RUNNING"));
}

void executionStage(unsigned long nowMs) {
  updatePistonSchedule(nowMs);

  float controlHeadingDeg = lastCleanHeadingDeg;
  bool usedNewCleanSamples = false;
  bool hadHeadingForControl = false;

  if (state != STATE_DONE) {
    hadHeadingForControl = headingForControl(controlHeadingDeg, usedNewCleanSamples);
    if (hadHeadingForControl) {
      lastControlHeadingDeg = controlHeadingDeg;
      updateSteeringFromHeading(controlHeadingDeg);
    }
  } else {
    allStop(nowMs);
    hadHeadingForControl = haveCleanHeading;
    lastControlHeadingDeg = lastCleanHeadingDeg;
  }

  printExecutionParameters(nowMs, usedNewCleanSamples, hadHeadingForControl);
  resetSampleWindow();
}

// ------------------------- Run initialization -------------------
// The rover locks the current compass heading before energizing the piston and then enters the 100 ms execution loop.
void startRun(unsigned long nowMs) {
  allStop(nowMs);

  targetHeadingDeg = readHeadingDeg();
  lastCleanHeadingDeg = targetHeadingDeg;
  lastControlHeadingDeg = targetHeadingDeg;
  haveCleanHeading = true;
  lastHeadingErrorDeg = 0.0f;

  resetSampleWindow();
  lastCompassSampleMs = nowMs;
  lastExecutionMs = nowMs;

#if ENABLE_IMU_PRINT
  lastImuSampleMs = 0;
  haveImuSample = false;
#endif

  pistonCyclesStarted = 1;
  pistonCyclesFinished = 0;
  pistonPhaseStartMs = nowMs;
  donePrinted = false;
  setPistonOn(nowMs);
  writeSteeringSafe(SERVO_CENTER_DEG);

  Serial.println();
  Serial.print(F("Run started. Holding initial heading_deg="));
  Serial.println(targetHeadingDeg, 2);
  Serial.println(F("Execution period is 100 ms. Compass readings are ignored while piston is ON."));

  state = STATE_RUNNING;
}

// ------------------------- Arduino setup and loop ---------------
// The main loop dispatches the active state; sensor sampling is continuous and execution is gated at 100 ms.
void setup() {
  pinMode(PISTON_PIN, OUTPUT);
  pinMode(LEFT_REAR_LIMIT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_REAR_LIMIT_PIN, INPUT_PULLUP);
  pinMode(CAL_LED_PIN, OUTPUT);

  digitalWrite(PISTON_PIN, LOW);
  digitalWrite(CAL_LED_PIN, LOW);

  Serial.begin(115200);
  Wire.begin();

  steeringServo.attach(STEERING_SERVO_PIN);
  writeSteeringSafe(SERVO_CENTER_DEG);

  if (!mag.init()) {
    Serial.println(F("ERROR: LIS3MDL compass not detected. Rover stopped."));
    state = STATE_COMPASS_ERROR;
    return;
  }

  mag.enableDefault();
  initializeImuLogging();

  usePreCalibrationValues();
  lastMenuPromptMs = millis();
  printCalibrationChoicePrompt();
}

void loop() {
  const unsigned long nowMs = millis();

  switch (state) {
    case STATE_WAIT_CAL_CHOICE:
      waitForCalibrationChoice(nowMs);
      break;

    case STATE_CALIBRATING:
      calibrationStage(nowMs);
      break;

    case STATE_WAIT_START:
      waitForStartCommand(nowMs);
      break;

    case STATE_RUNNING:
      sampleStage(nowMs);
      if (nowMs - lastExecutionMs >= CONTROL_PERIOD_MS) {
        lastExecutionMs = nowMs;
        executionStage(nowMs);
      }
      break;

    case STATE_DONE:
      if (!donePrinted) {
        donePrinted = true;
        allStop(nowMs);
        Serial.println(F("Run complete. Piston OFF, steering centered."));
      }
      break;

    case STATE_COMPASS_ERROR:
    default:
      allStop(nowMs);
      digitalWrite(CAL_LED_PIN, HIGH);
      break;
  }
}
