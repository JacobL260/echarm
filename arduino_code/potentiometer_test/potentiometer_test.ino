void setup() {
  Serial.begin(9600); // Start Serial Monitor at 9600 baud
}

void loop() {
  int potValue = analogRead(A0); // Read value from potentiometer (0–1023)
  
  // Map the potentiometer value (0-1023) to degrees (0-360)
  float degree = (potValue / 1023.0) * 270.0;

  Serial.print("Potentiometer Value: ");
  Serial.print(potValue);
  Serial.print("  Degree Position: ");
  Serial.println(degree);
  
  delay(100); // Small delay to avoid flooding the serial monitor
}