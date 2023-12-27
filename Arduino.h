#include <Arduino.h>

// Motor control pins
const int pwmPin = 5;  // PWM pin connected to H-Bridge
const int in1Pin = 2;  // H-Bridge input 1
const int in2Pin = 4;  // H-Bridge input 2

// Encoder pins
const int encoderPinA = 14;  // Hall-effect encoder channel A
const int encoderPinB = 12;  // Hall-effect encoder channel B

// Serial communication
const int serialSpeed = 9600;

// Motor parameters
const int maxSpeed = 255;
const int minSpeed = 0;

// Variables
int motorSpeed = 0;
volatile long encoderCount = 0;  // Counter for encoder pulses

void setup() {
  // Motor setup
  pinMode(pwmPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  // Encoder setup
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);

  // Serial setup
  Serial.begin(serialSpeed);
}

void loop() {
  // Read user input
  if (Serial.available() > 0) {
    char command = Serial.read();
    int speedValue = 0;

    // Parse user command
    switch (command) {
      case 'F':
        speedValue = readSpeedFromSerial();
        setMotorSpeed(speedValue);
        break;
      case 'B':
        speedValue = readSpeedFromSerial();
        setMotorSpeed(-speedValue);  // Negative speed for backward
        break;
      case 'S':
        stopMotor();
        break;
    }
  }

  // Implement closed-loop motor control using encoder feedback
  // Adjust the PID control parameters as needed
  // For simplicity, proportional control is used here
  int targetEncoderCount = map(motorSpeed, -maxSpeed, maxSpeed, -500, 500);
  int error = targetEncoderCount - encoderCount;
  int pidOutput = constrain(error, -255, 255);

  analogWrite(pwmPin, abs(pidOutput));

  if (pidOutput >= 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }
}

int readSpeedFromSerial() {
  int speedValue = 0;
  while (Serial.available() > 0) {
    char digit = Serial.read();
    if (digit >= '0' && digit <= '9') {
      speedValue = speedValue * 10 + (digit - '0');
    }
  }
  return constrain(speedValue, minSpeed, maxSpeed);
}

void setMotorSpeed(int speed) {
  motorSpeed = constrain(speed, -maxSpeed, maxSpeed);
}

void stopMotor() {
  setMotorSpeed(0);
}

// Interrupt service routine for encoder counting
void updateEncoder() {
  if (digitalRead(encoderPinB) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}