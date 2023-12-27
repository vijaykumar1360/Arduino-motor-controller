#include <Arduino.h>

// Constants
const int motorPWM = 5;    // PWM pin connected to H-Bridge
const int encoderPinA = 2; // Hall effect encoder channel A
const int encoderPinB = 3; // Hall effect encoder channel B

const int encoderResolution = 4096; // Updated pulses per revolution
const float pulsesPerDegree = static_cast<float>(encoderResolution) / 360.0;

volatile int encoderCount = 0;  // Counter for encoder pulses
int targetSpeed = 0;

// Motor Specifications
const int noLoadSpeed = 300;   // RPM

// Function prototypes
void setMotorSpeed(int pwmDutyCycle);
void processSerialCommand();
int calculatePWMDutyCycle(int targetSpeed);
float calculatePulsesPerDegree();

void setup() {
  pinMode(motorPWM, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, RISING);

  Serial.begin(9600);
}

void loop() {
  processSerialCommand();
  int pwmDutyCycle = calculatePWMDutyCycle(targetSpeed);
  setMotorSpeed(pwmDutyCycle);
}

void updateEncoder() {
  if (digitalRead(encoderPinB) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setMotorSpeed(int pwmDutyCycle) {
  analogWrite(motorPWM, pwmDutyCycle);
}

void processSerialCommand() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    switch (command) {
      case 'F':
        targetSpeed = Serial.parseInt();
        break;
      case 'B':
        targetSpeed = -Serial.parseInt();
        break;
      case 'S':
        targetSpeed = 0;
        break;
      default:
        // Handle invalid command
        break;
    }
  }
}

int calculatePWMDutyCycle(int targetSpeed) {
  // Map target speed to PWM duty cycle using motor specifications
  float speedRatio = static_cast<float>(targetSpeed) / noLoadSpeed;
  int pwmDutyCycle = constrain(static_cast<int>(speedRatio * 255), 0, 255);
  return pwmDutyCycle;
}

float calculatePulsesPerDegree() {
  return pulsesPerDegree;
}
