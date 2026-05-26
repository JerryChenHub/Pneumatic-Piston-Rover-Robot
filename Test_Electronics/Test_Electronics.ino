#include <Servo.h>

Servo steeringServo;

// Pins
const byte SERVO_PIN = 3;
const byte SOLENOID_PIN = 2;
const byte LIMIT_SWITCH_PIN = 4;

// Steering limits
const int SERVO_MIN_ANGLE = 65;
const int SERVO_MAX_ANGLE = 125;
const int SERVO_START_ANGLE = 90;

// Timing
const unsigned long UPDATE_INTERVAL_MS = 500;
const unsigned long SOLENOID_ON_MS = 100;
const unsigned long SOLENOID_OFF_MS = 200;

// Serial buffer
char rxBuffer[32];
byte rxIndex = 0;

// Pending command
bool commandPending = false;
int pendingAngle = SERVO_START_ANGLE;
int pendingPulseCount = 0;

// 500 ms update timer
unsigned long lastUpdateMs = 0;

// Solenoid state machine
enum SolenoidState {
  SOL_IDLE,
  SOL_ON,
  SOL_OFF
};

SolenoidState solenoidState = SOL_IDLE;
unsigned long solenoidStateStartMs = 0;
int pulsesRemaining = 0;

bool onlySpaces(const char *s) {
  while (*s) {
    if (*s != ' ' && *s != '\t') return false;
    s++;
  }
  return true;
}

void startSolenoidPulses(int count) {
  if (count <= 0) return;

  pulsesRemaining = count;
  solenoidState = SOL_ON;
  solenoidStateStartMs = millis();
  digitalWrite(SOLENOID_PIN, HIGH);
}

void updateSolenoid() {
  unsigned long now = millis();

  if (solenoidState == SOL_ON) {
    if (now - solenoidStateStartMs >= SOLENOID_ON_MS) {
      digitalWrite(SOLENOID_PIN, LOW);
      solenoidState = SOL_OFF;
      solenoidStateStartMs = now;
    }
  } 
  else if (solenoidState == SOL_OFF) {
    if (now - solenoidStateStartMs >= SOLENOID_OFF_MS) {
      pulsesRemaining--;

      if (pulsesRemaining > 0) {
        digitalWrite(SOLENOID_PIN, HIGH);
        solenoidState = SOL_ON;
        solenoidStateStartMs = now;
      } else {
        digitalWrite(SOLENOID_PIN, LOW);
        solenoidState = SOL_IDLE;
      }
    }
  }
}

void parseCommand(char *cmd) {
  // Expected format: angle,count
  // Example: 90,2

  char *comma = strchr(cmd, ',');
  if (comma == NULL) {
    Serial.println("Invalid command. Use format: 90,2;");
    return;
  }

  *comma = '\0';

  char *anglePart = cmd;
  char *countPart = comma + 1;

  char *angleEnd;
  char *countEnd;

  long rawAngle = strtol(anglePart, &angleEnd, 10);
  long rawCount = strtol(countPart, &countEnd, 10);

  if (anglePart == angleEnd || countPart == countEnd ||
      !onlySpaces(angleEnd) || !onlySpaces(countEnd)) {
    Serial.println("Invalid number. Use format: 90,2;");
    return;
  }

  int safeAngle = constrain((int)rawAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  if (rawCount < 0) rawCount = 0;
  if (rawCount > 20) rawCount = 20;   // Safety limit to avoid very long firing

  pendingAngle = safeAngle;
  pendingPulseCount = (int)rawCount;
  commandPending = true;

  Serial.print("Command received. Angle=");
  Serial.print(rawAngle);
  Serial.print(" -> limited to ");
  Serial.print(safeAngle);
  Serial.print(", pulses=");
  Serial.println(pendingPulseCount);
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == ';') {
      rxBuffer[rxIndex] = '\0';
      parseCommand(rxBuffer);
      rxIndex = 0;
    } 
    else if (c == '\n' || c == '\r') {
      // Ignore line endings
    } 
    else {
      if (rxIndex < sizeof(rxBuffer) - 1) {
        rxBuffer[rxIndex++] = c;
      } else {
        rxIndex = 0;
        Serial.println("Serial buffer overflow. Command discarded.");
      }
    }
  }
}

void updateEvery500ms() {
  unsigned long now = millis();

  if (now - lastUpdateMs < UPDATE_INTERVAL_MS) {
    return;
  }

  lastUpdateMs = now;

  // Limit switch uses INPUT_PULLUP:
  // Open/not pressed = HIGH
  // Pressed/triggered = LOW
  bool limitSwitchPressed = digitalRead(LIMIT_SWITCH_PIN) == LOW;

  Serial.print("Limit switch: ");
  Serial.println(limitSwitchPressed ? "PRESSED" : "OPEN");

  // Execute one pending command per 500 ms update cycle.
  // Solenoid command starts only when previous solenoid cycle is finished.
  if (commandPending && solenoidState == SOL_IDLE) {
    steeringServo.write(pendingAngle);

    Serial.print("Servo moved to ");
    Serial.println(pendingAngle);

    startSolenoidPulses(pendingPulseCount);

    commandPending = false;
  }
}

void setup() {
  Serial.begin(115200);

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_START_ANGLE);

  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);

  // Important:
  // This makes pin 4 HIGH by internal pull-up while listening.
  // It does NOT make pin 4 a HIGH output.
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

  Serial.println("Ready.");
  Serial.println("Send commands like: 90,2;");
  Serial.println("Servo angle is strictly limited to 70-110.");
}

void loop() {
  readSerialCommands();   // Always listen to Serial
  updateEvery500ms();     // Main update every 500 ms
  updateSolenoid();       // Non-blocking solenoid timing
}