#include <Servo.h>

// Pins
const int joystickXPin = A0;
const int joystickYPin = A1;
const int potPin = A4;
const int pot2Pin = A6;
const int buzzerPin = 13;
const int servoPin = 12;
const int servo2Pin = 11;

// TB6612FNG Motor Driver Pins
const int PWMA = 3;
const int AIN2 = 5;
const int AIN1 = 2;
const int STBY = 9;
const int BIN1 = 8;
const int BIN2 = 7;
const int PWMB = 6;

// Constants
const int ADC_MAX = 1023;
const int PWM_MAX = 255;
const int SERVO_MAX_ANGLE = 180;
const int SERVO_CENTER = 90;

// Timing variables
unsigned long lastBuzzerTime = 0;
unsigned long buzzerStartTime = 0;
const int buzzerInterval = 1000;
const int buzzerDuration = 100;
bool buzzerActive = false;

// Servo control
Servo continuousServo;  // FS90R on D12
Servo positionServo;    // MG90S on D11

// Pot 1 (A4) - FS90R
const int potCenter = 891;
const int deadzone = 50;

// Joystick settings
const int joyCenter = 512;
const int joyDeadzone = 50;

// Exponential smoothing 
float potSmoothed = 512.0;
float pot2Smoothed = 512.0;
const float alpha = 0.15;  // Smoothing factor (0.1-0.3 works well)

// Acceleration ramping variables
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
const int maxAcceleration = 20;
const int directionChangeDelay = 10;
unsigned long lastDirectionChange = 0;
int lastLeftDirection = 0;
int lastRightDirection = 0;

void setup() {
  Serial.begin(9600);
  pinMode(buzzerPin, OUTPUT);
  
  // Motor driver pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  // Enable motor driver
  digitalWrite(STBY, HIGH);
  
  continuousServo.attach(servoPin);
  continuousServo.write(SERVO_CENTER);
  
  positionServo.attach(servo2Pin);
  positionServo.write(SERVO_CENTER);
  
  // Initialize smoothed values with first readings
  delay(100);
  potSmoothed = analogRead(potPin);
  pot2Smoothed = analogRead(pot2Pin);
}

inline int getDirection(int speed) {
  if (speed > 0) return 1;
  if (speed < 0) return -1;
  return 0;
}

int rampSpeed(int currentSpeed, int targetSpeed, int maxChange) {
  int difference = targetSpeed - currentSpeed;
  
  if (abs(difference) <= maxChange) {
    return targetSpeed;
  }
  
  return (difference > 0) ? currentSpeed + maxChange : currentSpeed - maxChange;
}

void setMotor(int pwmPin, int in1Pin, int in2Pin, int speed) {
  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void loop() {
  // Read inputs
  int xValue = analogRead(joystickXPin);
  int yValue = analogRead(joystickYPin);
  int potValueRaw = analogRead(potPin);
  int pot2ValueRaw = analogRead(pot2Pin);
  
  //Apply exponential smoothing
  potSmoothed = (alpha * potValueRaw) + ((1.0 - alpha) * potSmoothed);
  pot2Smoothed = (alpha * pot2ValueRaw) + ((1.0 - alpha) * pot2Smoothed);
  
  // Control FS90R
  int servoSpeed = SERVO_CENTER;
  int potSmoothedInt = (int)potSmoothed;
  
  if (potSmoothedInt > potCenter + deadzone) {
    servoSpeed = map(potSmoothedInt, potCenter + deadzone, ADC_MAX, SERVO_CENTER, SERVO_MAX_ANGLE);
    servoSpeed = constrain(servoSpeed, SERVO_CENTER, SERVO_MAX_ANGLE);
    
  } else if (potSmoothedInt < potCenter - deadzone) {
    servoSpeed = map(potSmoothedInt, 0, potCenter - deadzone, 0, SERVO_CENTER);
    servoSpeed = constrain(servoSpeed, 0, SERVO_CENTER);
  }
  
  continuousServo.write(servoSpeed);
  
  // Control MG90S
  int servoAngle = map((int)pot2Smoothed, 0, ADC_MAX, 0, SERVO_MAX_ANGLE);
  servoAngle = constrain(servoAngle, 0, SERVO_MAX_ANGLE);
  positionServo.write(servoAngle);
  
  // Motor control with joystick
  int motorLeftSpeed = 0;
  int motorRightSpeed = 0;
  
  // Calculate Y axis
  int yAxis = 0;
  if (yValue > joyCenter + joyDeadzone) {
    yAxis = map(yValue, joyCenter + joyDeadzone, ADC_MAX, 0, PWM_MAX);
    yAxis = constrain(yAxis, 0, PWM_MAX);
  } else if (yValue < joyCenter - joyDeadzone) {
    yAxis = map(yValue, 0, joyCenter - joyDeadzone, -PWM_MAX, 0);
    yAxis = constrain(yAxis, -PWM_MAX, 0);
  }
  
  // Calculate X axis
  int xAxis = 0;
  if (xValue > joyCenter + joyDeadzone) {
    xAxis = map(xValue, joyCenter + joyDeadzone, ADC_MAX, 0, PWM_MAX);
    xAxis = constrain(xAxis, 0, PWM_MAX);
  } else if (xValue < joyCenter - joyDeadzone) {
    xAxis = map(xValue, 0, joyCenter - joyDeadzone, -PWM_MAX, 0);
    xAxis = constrain(xAxis, -PWM_MAX, 0);
  }
  
  // Tank Steering motor control
  motorLeftSpeed = constrain(yAxis + xAxis, -PWM_MAX, PWM_MAX);
  motorRightSpeed = constrain(yAxis - xAxis, -PWM_MAX, PWM_MAX);
  
  // Detect direction changes
  int targetLeftDirection = getDirection(motorLeftSpeed);
  int targetRightDirection = getDirection(motorRightSpeed);
  bool directionChanged = false;
  
  if ((targetLeftDirection != 0 && lastLeftDirection != 0 && 
       targetLeftDirection != lastLeftDirection) ||
      (targetRightDirection != 0 && lastRightDirection != 0 && 
       targetRightDirection != lastRightDirection)) {
    directionChanged = true;
  }
  
  // Direction change handling
  if (directionChanged && (millis() - lastDirectionChange < directionChangeDelay)) {
    motorLeftSpeed = 0;
    motorRightSpeed = 0;
  } else if (directionChanged) {
    lastDirectionChange = millis();
    motorLeftSpeed = 0;
    motorRightSpeed = 0;
  }
  
  // Apply acceleration ramping
  currentLeftSpeed = rampSpeed(currentLeftSpeed, motorLeftSpeed, maxAcceleration);
  currentRightSpeed = rampSpeed(currentRightSpeed, motorRightSpeed, maxAcceleration);
  
  // Update direction tracking
  lastLeftDirection = getDirection(currentLeftSpeed);
  lastRightDirection = getDirection(currentRightSpeed);
  
  // Set motors with ramped speeds
  setMotor(PWMA, AIN1, AIN2, currentLeftSpeed);
  setMotor(PWMB, BIN1, BIN2, currentRightSpeed);
  
  // Reversing buzzer
  if (yValue < joyCenter - joyDeadzone) {
    unsigned long currentTime = millis();
    
    if (!buzzerActive && (currentTime - lastBuzzerTime >= buzzerInterval)) {
      digitalWrite(buzzerPin, HIGH);
      buzzerStartTime = currentTime;
      buzzerActive = true;
      lastBuzzerTime = currentTime;
    }
  }
  
  // Turn off buzzer after set time
  if (buzzerActive && (millis() - buzzerStartTime >= buzzerDuration)) {
    digitalWrite(buzzerPin, LOW);
    buzzerActive = false;
  }
  
  // Serial output 
  Serial.print("X:");
  Serial.print(xValue);
  Serial.print(" Y:");
  Serial.print(yValue);
  Serial.print(" ML:");
  Serial.print(currentLeftSpeed);
  Serial.print(" MR:");
  Serial.print(currentRightSpeed);
  Serial.print(" P1:");
  Serial.print(potSmoothedInt);
  Serial.print(" S1:");
  Serial.print(servoSpeed);
  Serial.print(" P2:");
  Serial.print((int)pot2Smoothed);
  Serial.print(" S2:");
  Serial.println(servoAngle);
  
  delay(10); 
}