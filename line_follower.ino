/*
  High-Stability Line Following Robot
  ESP32 + 5 Digital IR Sensors + L298N

  Assumptions:
  - Black line on a light surface
  - Sensors output LOW on black
  - Motor direction and wiring are verified
*/

#define S1 32
#define S2 33
#define S3 34
#define S4 35
#define S5 36

#define ENA 25
#define IN1 26
#define IN2 27

#define ENB 14
#define IN3 16
#define IN4 17

// Change to HIGH if your sensors output HIGH on black.
const int LINE_DETECTED = LOW;

// PWM range: 0 to 255
const int MAX_PWM = 255;
int baseSpeed = 145;
const int MIN_SPEED = 75;

// Tune these values on your actual track.
float Kp = 0.075;
float Ki = 0.000;
float Kd = 0.32;

float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

int lastDirection = 1;  // 1 = last seen to right

const int weights[5] = {
  -2000, -1000, 0, 1000, 2000
};

unsigned long lastTime = 0;

void setMotor(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -MAX_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);

  // Left motor direction
  digitalWrite(IN1, leftSpeed >= 0 ? HIGH : LOW);
  digitalWrite(IN2, leftSpeed >= 0 ? LOW : HIGH);

  // Right motor direction
  digitalWrite(IN3, rightSpeed >= 0 ? HIGH : LOW);
  digitalWrite(IN4, rightSpeed >= 0 ? LOW : HIGH);

  analogWrite(ENA, abs(leftSpeed));
  analogWrite(ENB, abs(rightSpeed));
}

void stopRobot() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

bool readLine(float &position) {
  int pins[5] = {S1, S2, S3, S4, S5};
  long weightedSum = 0;
  int activeCount = 0;

  for (int i = 0; i < 5; i++) {
    if (digitalRead(pins[i]) == LINE_DETECTED) {
      weightedSum += weights[i];
      activeCount++;
    }
  }

  if (activeCount == 0) {
    return false;  // Line lost
  }

  position = (float)weightedSum / activeCount;
  return true;
}

void setup() {
  Serial.begin(115200);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopRobot();
  delay(1000);

  lastTime = micros();
}

void loop() {
  float position = 0;
  bool lineFound = readLine(position);

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;

  if (dt <= 0 || dt > 0.1f) {
    dt = 0.005f;
  }

  lastTime = now;

  // Recover when all sensors lose the line.
  if (!lineFound) {
    integral = 0;

    if (lastDirection < 0) {
      setMotor(-65, 95);
    } else {
      setMotor(95, -65);
    }

    previousError = error;
    delay(2);
    return;
  }

  error = position;

  if (error < 0) lastDirection = -1;
  if (error > 0) lastDirection = 1;

  // PID controller
  integral += error * dt;
  integral = constrain(integral, -4000.0f, 4000.0f);

  derivative = (error - previousError) / dt;

  float correction =
      Kp * error +
      Ki * integral +
      Kd * derivative;

  previousError = error;

  // Slow down as the line moves away from center.
  int turnAmount = abs((int)position);
  int speedReduction = map(
      constrain(turnAmount, 0, 2000),
      0, 2000, 0, 65
  );

  int targetSpeed = max(
      MIN_SPEED,
      baseSpeed - speedReduction
  );

  int leftSpeed = targetSpeed + (int)correction;
  int rightSpeed = targetSpeed - (int)correction;

  leftSpeed = constrain(leftSpeed, -MAX_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);

  setMotor(leftSpeed, rightSpeed);

  Serial.print("Position: ");
  Serial.print(position);
  Serial.print(" Correction: ");
  Serial.print(correction);
  Serial.print(" Left: ");
  Serial.print(leftSpeed);
  Serial.print(" Right: ");
  Serial.println(rightSpeed);
}