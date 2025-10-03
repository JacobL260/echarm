#include <AccelStepper.h>
#include <PID_v1_bc.h>

// === Pin Definitions ===
const int potPin = A0;
const int stepPin = 9;
const int dirPin = 8;

// === Stepper Setup (DRIVER mode) ===
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// === Actuator Configuration ===
const float maxMotorSpeed = 1000.0; // Steps per something to bee determiend

// === PID variables ===
double setpoint;   // Target actuator angle in degrees
double rawSetpoint;
double input;      // Actual actuator angle in degrees (from pot)
double output;     // PID output (motor speed in steps/sec)

double Kp = 5.0, Ki = 0, Kd = 0;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// === Actuator Range (Degrees) ===
const float maxPotAngle = 270.0;    // Potentiometer full range
const float minPotAngle = 0.0;

// === Actuator Range (Degrees) ===
const float maxPotSignal = 1023.0;    // Potentiometer full range
const float minPotSignal = 0.0;

float motorMove;
const float actuatorToPotRatio = -1;
const float motorToActuatoDirection = -1;

void setup() {
  Serial.begin(115200);

  stepper.setMaxSpeed(maxMotorSpeed);     // steps/sec
  stepper.setAcceleration(500);  // not used with runSpeed()

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-maxMotorSpeed, maxMotorSpeed); // Speed (steps/sec)
}

void loop() {
  // === Read potentiometer position (feedback) ===
  int potValue = analogRead(potPin);  // 0 - 1023
  int potAngle = ((float)potValue / maxPotSignal) * maxPotAngle; // Potiometer angle in degrees, 0 signal is 0 degrees, 1023 is 270
  input = potAngle * actuatorToPotRatio;

  // === Desired actuator position (setpoint) ===
  rawSetpoint = 90 ; // Setpoint determined by big program
  setpoint = rawSetpoint;  // Setpoint changed to the ratio of actuator to potionometer

  // === Run PID and drive motor ===
  myPID.Compute();
  motorMove = output*motorToActuatoDirection;
  stepper.setSpeed(motorMove);     // PID output is in steps/sec
  stepper.runSpeed();

  // === Debug ===
  Serial.print("Pot: "); Serial.print(potValue);
  Serial.print(" Input (deg): "); Serial.print(int(input));
  Serial.print(" Stpnt: "); Serial.print(int(setpoint));
  Serial.print(" Otpt: "); Serial.println(int(output));
}

float wrapTo360(float angle) {
  // First, reduce angle by full rotations
  while (angle >= 360.0) {
    angle -= 360.0;
  }
  while (angle < 0.0) {
    angle += 360.0;
  }
  return angle;
}
