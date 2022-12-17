// EECS C106A - Final Project
// Team 17
// Authors: Grace Jung, Donald Le

const int flexPin = 4; 
int sensorValue; // Save analog value
float position;

void setup() {
  // Begin serial communication
  Serial.begin(9600);
}

void loop() {
  // Read and save analog value.
  sensorValue = analogRead(flexPin);
  float position = sensorValue * (0.0410 / 1023.0);
  Serial.println(sensorValue);
  // Serial.println("sensorValue: " + String(sensorValue));
  // Serial.println("");
  // Serial.println("position: " + String(position));
  
  // value = map(value, 700, 900, 0, 255);//Map value 0-1023 to 0-255 (PWM)
  // Serial.println(value); 
  delay(500);       
}
