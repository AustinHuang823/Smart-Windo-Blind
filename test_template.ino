// Sensor pins setup
const int phototransistorPin = A1; // phototransistor input pin
const int temperatureSensorPin = A2; // temperature sensor pin

// Timing variables for loop control
unsigned long previousTime = millis();
const int deltaT = 500;  

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read the sensor readings
  int lightLevel = analogRead(phototransistorPin);
  int temperature = analogRead(temperatureSensorPin);

  // Printing values:
  Serial.print("Current light reading: ");
  Serial.print(lightLevel);
  Serial.print(" ;   Current temperautre: ");
  Serial.println(temperature);

  // Timing control with while loop
  while (millis() - previousTime < deltaT) {}
  previousTime = millis();
}
