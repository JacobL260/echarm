#include <PID_v1_bc.h>
#include <AccelStepper.h>
#include <math.h>

// Potentiometer angle calculation
int minDegree = 0;
int maxDegree = 270;
int minSignal = 0;
int maxSignal = 1023;
int pot_offset_angle = 0;
int pot_offset_signal = 0;
int outputToPotRatio = -1;

float m;
float b;
float upperLimit;
float lowerLimit;

// Potentiometer pin
#define POT_PIN A0

// Stepper motor pins
#define DIR_PIN 8
#define STEP_PIN 9

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
int maxSpeed = 1000;
int maxAcceleration = 500;

// PID variables
double Setpoint, Input, Output;
double serialSetpoint;  // User-defined target angle (before offset applied)

// PID tuning parameters
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);

  // Calculate potentiometer-to-angle ratio and offset
  m = (float(maxDegree) - minDegree)/(maxSignal-minSignal)*outputToPotRatio;
  b = pot_offset_angle - m * pot_offset_signal;

  lowerLimit = minSignal * m + b;
  upperLimit = maxSignal * m + b;

  // Stepper configuration
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(maxAcceleration);

  // Set initial target angle
  serialSetpoint = 90.0;
  Setpoint = serialSetpoint * outputToPotRatio + b;

  // Initialize PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-maxSpeed, maxSpeed);  // Output in steps/sec

  // Print setup info
  Serial.println("=== System Initialized ===");
  Serial.print("Ratio: "); Serial.println(m, 6);
  Serial.print("Offset (b): "); Serial.println(b, 6);
  Serial.print("Lower Limit: "); Serial.println(lowerLimit, 2);
  Serial.print("Upper Limit: "); Serial.println(upperLimit, 2);
  Serial.print("Serial Setpoint: "); Serial.println(serialSetpoint, 2);
  Serial.print("Adjusted Setpoint: "); Serial.println(Setpoint, 2);
  Serial.println("==========================");
}

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 100; // 100 ms

void loop() {
  // Read potentiometer and calculate angle
  int potValue = analogRead(POT_PIN);
  Input = potValue * m + b;

  // Compute PID output
  myPID.Compute();

  // Control stepper speed with PID output
  stepper.setSpeed(Output);
  stepper.runSpeed();

  // Debug print every 100 ms
  if (millis() - lastPrintTime >= printInterval) {
    Serial.print("Pot: ");
    Serial.print(potValue);
    Serial.print(" | Angle: ");
    Serial.print(Input, 2);
    Serial.print("° | Serial Setpoint: ");
    Serial.print(serialSetpoint, 2);
    Serial.print("° | Setpoint: ");
    Serial.print(Setpoint, 2);
    Serial.print("° | PID Output: ");
    Serial.print(Output, 2);
    Serial.print(" | Ratio: ");
    Serial.print(m, 6);
    Serial.print(" | Offset b: ");
    Serial.print(b, 6);
    Serial.print(" | Limits: [");
    Serial.print(lowerLimit, 2);
    Serial.print(", ");
    Serial.print(upperLimit, 2);
    Serial.println("]");
    lastPrintTime = millis();
  }
}
