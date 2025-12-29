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
double serialCommand = 120;
double setpoint = 0;   // Target angle in degrees (0–270)
double input = 0;      // Current angle
double output = 0;     // PID output (stepper speed in steps/sec)

// PID tuning parameters
double Kp = 55, Ki = 0.1, Kd = 1;

// Create PID controller
// DIRECT is if MOTOR and ACTUATOR spin clockwise, REVERSED is Vise versea
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);

// Direction multiplier
float POT_TO_ACT_RATIO = -1.0;

// === Debug Timer ===
unsigned long lastDebugTime = 0;

double measuredPotAngle = 160;
double measuredActuatorAngle = 160;

void setup() {
  Serial.begin(115200);

  stepper.setMaxSpeed(maxMotorSpeed);
  stepper.setAcceleration(maxMotorAcceration);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-maxMotorSpeed, maxMotorSpeed);
}

void loop() {
  // === Read Potentiometer and convert to actuator angle ===
  int raw = analogRead(potPin); // Usuall 0 - 1023
  float potAngle = (raw / float(potMaxRaw)) *-1* maxPotAngle;
  float actuatorAngle = potAngle * POT_TO_ACT_RATIO;

  input = actuatorAngle;

  setpoint = serialCommand;

  // === PID Control ===
  myPID.Compute();
  stepper.setSpeed(output);
  stepper.runSpeed();

  // === Debug Output ===
  if (millis() - lastDebugTime >= 100) {
    lastDebugTime = millis();
    //Serial.print("raw:"); Serial.print(raw);
    Serial.print(" potangle:"); Serial.print(potAngle, 0);
    Serial.print(" actuatorangle:"); Serial.print(input, 0);
    Serial.print(" set:"); Serial.print(setpoint, 0);
    Serial.print(" out:"); Serial.println(output, 0);
  }
}
