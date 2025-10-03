#include <AccelStepper.h>
#include <PID_v1_bc.h>

// === CONFIGURATION ===

// Pins
const int potPin = A0;       // Analog pin connected to potentiometer
const int stepPin = 8;       // Step pin for motor speed
const int dirPin = 9;        // Direction control pin

// === Actuator Configuration ===
const float maxMotorSpeed = 1000.0;
const float maxMotorAcceration = 500.0;

// === Stepper Setup (DRIVER mode) ===
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// Constants
const float maxPotAngle = 270.0;
const int potMaxRaw = 1023;

// PID variables
double setpoint = 0;   // Target angle in degrees (0–270)
double input = 0;      // Current angle
double output = 0;     // PID output (stepper speed in steps/sec)

// PID tuning parameters
double Kp = 10, Ki = 1, Kd = 0.0;

// Create PID controller
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);

// Direction multiplier (for gear or pulley)
float direction_ratio = -1.0;  // Set to -1.0 if direction inverted

// === Setpoint Change Logic ===
const float positionThreshold = 2.0;           // Acceptable error in degrees
const unsigned long holdTimeRequired = 2000;   // Time to hold position before changing (ms)
unsigned long positionHoldStart = 0;           // When did we enter the "holding" state?
bool holding = false;

void setup() {
  Serial.begin(115200);

  stepper.setMaxSpeed(maxMotorSpeed);
  stepper.setAcceleration(maxMotorAcceration);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-maxMotorSpeed, maxMotorSpeed);

  // Random seed from unconnected analog pin (or from pot noise)
  randomSeed(analogRead(potPin));

  // Initialize with random setpoint
  setpoint = random(10, 261);
}

void loop() {
  // === Read Potentiometer and Convert to Angle ===
  int raw = analogRead(potPin);
  float potAngle = (raw / float(potMaxRaw)) * maxPotAngle;
  float angle = potAngle * direction_ratio;

  if (angle < 0) {
    angle += maxPotAngle * abs(direction_ratio);
  }

  input = angle;

  // === PID Control ===
  myPID.Compute();
  stepper.setSpeed(output);
  stepper.runSpeed();

  // === Check if setpoint is reached and held ===
  float error = abs(setpoint - input);

  if (error <= positionThreshold) {
    if (!holding) {
      holding = true;
      positionHoldStart = millis();
    } else {
      if (millis() - positionHoldStart >= holdTimeRequired) {
        // Pick a new random setpoint between 10 and 260 degrees
        setpoint = random(10, 261);
        holding = false;
      }
    }
  } else {
    holding = false;  // Reset if we're outside the threshold
  }

  // === Debug Output ===
  Serial.print("raw: "); Serial.print(raw);
  Serial.print(" | input: "); Serial.print(input, 1);
  Serial.print(" | setpoint: "); Serial.print(setpoint, 1);
  Serial.print(" | output: "); Serial.println(output, 1);
}
