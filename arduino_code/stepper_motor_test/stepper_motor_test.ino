#include <AccelStepper.h>

// Define pin connections
#define DIR_PIN 8
#define STEP_PIN 9

// Create stepper object (driver mode)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Timing
unsigned long moveStartTime = 0;
bool movingForward = true;
const long runTime = 10000; // 10 seconds

void setup() {
  Serial.begin(9600);
  
  // Set max speed and acceleration
  stepper.setMaxSpeed(1000);       // steps per second
  stepper.setAcceleration(500);    // steps per second^2

  // Start moving in one direction
  stepper.moveTo(1000000); // Large number so it moves for a while
  moveStartTime = millis();
}

void loop() {
  stepper.run();

  // Check if 10 seconds have passed
  if (millis() - moveStartTime >= runTime) {
    // Change direction
    movingForward = !movingForward;
    
    if (movingForward) {
      stepper.moveTo(1000000);  // Forward
    } else {
      stepper.moveTo(-1000000); // Backward
    }
    
    moveStartTime = millis(); // Reset timer
  }
}
