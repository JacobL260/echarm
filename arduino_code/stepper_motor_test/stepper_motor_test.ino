#include <AccelStepper.h>

// === CONFIGURATION ===
const int stepPin = 8;  // Step pin for motor
const int dirPin = 9;   // Direction pin for motor

const float motorSpeed = 500.0;  // Speed in steps/second
const float motorAcceleration = 500.0;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// Timing variables
unsigned long previousMillis = 0;
const unsigned long interval = 10000; // 10 seconds
bool direction = true; // true = one direction, false = the other

void setup() {
  Serial.begin(115200);

  stepper.setMaxSpeed(motorSpeed);
  stepper.setAcceleration(motorAcceleration);

  // Set initial direction
  stepper.setSpeed(motorSpeed);  // Positive direction
}

void loop() {
  unsigned long currentMillis = millis();

  // Switch direction every 10 seconds
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    direction = !direction;

    // Change direction
    if (direction) {
      stepper.setSpeed(motorSpeed);  // Positive
    } else {
      stepper.setSpeed(-motorSpeed); // Negative
    }
  }

  // Keep motor running at set speed
  stepper.runSpeed();
}
