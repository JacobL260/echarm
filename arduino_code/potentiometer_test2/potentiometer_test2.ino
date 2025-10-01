int pot_max_degree = 270;
int pot_max_signal = 1023;
int pot_offset_angle = 0;
int pot_offset_signal = 0;
int pot_ratio = 2;


float ratio;
float b;

void setup() {
  Serial.begin(9600);
  
  ratio = float(pot_max_degree) / pot_max_signal * pot_ratio;
  b = pot_offset_angle - ratio * pot_offset_signal;
}

float wrapAngle(float angle) {
  while (angle < 0) {
    angle += 360;
  }
  while (angle >= 360) {
    angle -= 360;
  }
  return angle;
}

void loop() {
  int potValue = analogRead(A0);

  float degree = potValue * ratio + b;

  degree = wrapAngle(degree);

  Serial.print("Potentiometer Value: ");
  Serial.print(potValue);
  Serial.print("  Degree Position: ");
  Serial.println(degree);

  delay(100);
}
