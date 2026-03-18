/*
 * ============================================================
 *  LINE FOLLOWER — PID CONTROLLER
 *  Hardware: Arduino Nano + TCRT5000 x5 + L298N + 2x DC Motor
 *  Power:    3.7V Li-Ion (salvaged phone battery)
 *  Author:   Cosmin Leonardo Cozaciuc
 *  Version:  1.0 — reconstructed & documented 2025
 * ============================================================
 *
 *  SENSOR ARRAY LAYOUT (front of robot):
 *
 *   [S0] [S1] [S2] [S3] [S4]
 *    |    |    |    |    |
 *   far  mid  CTR  mid  far
 *  left left       right right
 *
 *  WIRING SUMMARY:
 *   TCRT5000 array  → Arduino Nano A0–A4
 *   L298N IN1       → D5  (Left motor direction A)
 *   L298N IN2       → D6  (Left motor direction B)
 *   L298N IN3       → D9  (Right motor direction A)
 *   L298N IN4       → D10 (Right motor direction B)
 *   L298N ENA (PWM) → D3  (Left motor speed)
 *   L298N ENB (PWM) → D11 (Right motor speed)
 *   L298N 5V out    → Arduino Nano 5V pin
 *   Battery 3.7V    → L298N 12V pin
 * ============================================================
 */

// ── PIN DEFINITIONS ─────────────────────────────────────────

const int SENSOR_PINS[5] = {A0, A1, A2, A3, A4};

const int LEFT_IN1  = 5;
const int LEFT_IN2  = 6;
const int LEFT_PWM  = 3;

const int RIGHT_IN3 = 9;
const int RIGHT_IN4 = 10;
const int RIGHT_PWM = 11;

// ── PID TUNING CONSTANTS ─────────────────────────────────────
/*
 *  Tuning procedure:
 *   1. Start with Ki=0, Kd=0, increase Kp until robot follows line
 *   2. Increase Kd until oscillation stops
 *   3. Add small Ki only if robot drifts on straight sections
 */
float Kp = 25.0;
float Ki = 0.0;
float Kd = 15.0;

// ── BASE SPEED ───────────────────────────────────────────────
const int BASE_SPEED = 180;
const int MAX_SPEED  = 255;
const int MIN_SPEED  = 0;

// ── SENSOR CALIBRATION ───────────────────────────────────────
int sensorMin[5] = {1023, 1023, 1023, 1023, 1023};
int sensorMax[5] = {0,    0,    0,    0,    0   };

// ── PID STATE VARIABLES ──────────────────────────────────────
float lastError    = 0;
float integralSum  = 0;
unsigned long lastTime = 0;

// ── SENSOR WEIGHTS ───────────────────────────────────────────
/*
 *  S0=-2, S1=-1, S2=0, S3=+1, S4=+2
 *  Center sensor (S2) = 0 (no error)
 *  Left sensors  = negative error (robot drifted right)
 *  Right sensors = positive error (robot drifted left)
 */
const float SENSOR_WEIGHTS[5] = {-2.0, -1.0, 0.0, 1.0, 2.0};

// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);

  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(LEFT_PWM,  OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);

  stopMotors();

  Serial.println("=== LINE FOLLOWER BOOT ===");
  Serial.println("Calibrating — move robot over line...");

  // ── CALIBRATION PHASE (3 seconds) ───────────────────────
  unsigned long calStart = millis();
  while (millis() - calStart < 3000) {
    for (int i = 0; i < 5; i++) {
      int val = analogRead(SENSOR_PINS[i]);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
  }

  Serial.println("Calibration done. Starting in 1s...");
  delay(1000);
  lastTime = millis();
}

// ════════════════════════════════════════════════════════════
void loop() {
  // ── 1. READ & NORMALIZE SENSORS ─────────────────────────
  float normalized[5];
  for (int i = 0; i < 5; i++) {
    int raw = analogRead(SENSOR_PINS[i]);
    normalized[i] = (float)(raw - sensorMin[i]) /
                    (float)(sensorMax[i] - sensorMin[i] + 1);
    normalized[i] = constrain(normalized[i], 0.0, 1.0);
  }

  // ── 2. COMPUTE WEIGHTED POSITION ERROR ──────────────────
  float weightedSum = 0;
  float totalWeight = 0;
  for (int i = 0; i < 5; i++) {
    weightedSum += SENSOR_WEIGHTS[i] * normalized[i];
    totalWeight += normalized[i];
  }

  float error = 0;
  if (totalWeight > 0.1) {
    error = weightedSum / totalWeight;
  } else {
    // Line lost — use last known error direction
    error = lastError * 1.5;
  }

  // ── 3. PID CALCULATION ──────────────────────────────────
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  if (dt <= 0) dt = 0.01;

  float derivative = (error - lastError) / dt;

  // Integral with anti-windup clamp
  integralSum += error * dt;
  integralSum = constrain(integralSum, -50.0, 50.0);

  float pidOutput = (Kp * error) +
                    (Ki * integralSum) +
                    (Kd * derivative);

  lastError = error;

  // ── 4. APPLY PID TO MOTOR SPEEDS ────────────────────────
  /*
   *  Positive PID output → turn right
   *    → increase right motor, decrease left motor
   *  Negative PID output → turn left
   *    → increase left motor, decrease right motor
   */
  int leftSpeed  = BASE_SPEED - (int)pidOutput;
  int rightSpeed = BASE_SPEED + (int)pidOutput;

  leftSpeed  = constrain(leftSpeed,  MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  driveMotors(leftSpeed, rightSpeed);

  // ── 5. DEBUG OUTPUT (disable for competition) ───────────
  if (millis() % 100 < 10) {
    Serial.print("Err:"); Serial.print(error, 2);
    Serial.print(" PID:"); Serial.print(pidOutput, 2);
    Serial.print(" L:"); Serial.print(leftSpeed);
    Serial.print(" R:"); Serial.println(rightSpeed);
  }
}

// ════════════════════════════════════════════════════════════
void driveMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  analogWrite(LEFT_PWM, leftSpeed);

  digitalWrite(RIGHT_IN3, HIGH);
  digitalWrite(RIGHT_IN4, LOW);
  analogWrite(RIGHT_PWM, rightSpeed);
}

void stopMotors() {
  digitalWrite(LEFT_IN1,  LOW);
  digitalWrite(LEFT_IN2,  LOW);
  analogWrite(LEFT_PWM,   0);
  digitalWrite(RIGHT_IN3, LOW);
  digitalWrite(RIGHT_IN4, LOW);
  analogWrite(RIGHT_PWM,  0);
}
