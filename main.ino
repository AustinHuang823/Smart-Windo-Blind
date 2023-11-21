// Define volatile variables to be used in the interrupt service routines
volatile int counterEncoder = 0;
volatile boolean aState, bState;

// Motor control pins
const int enablePin = 9; // H-bridge IC output for speed control
const int input1Pin = 8; // Digital output for motor direction
const int input2Pin = 7; // Digital output for motor direction
const int encoderAPin = 2; 
const int encoderBPin = 3; 

// Button pins setup
const int switchPin_ModeSwitch = 6; // Switch button pin for switching between Auto modes
const int switchPin_ManualMode = 5; // Switch button pin for switching between Auto and manual modes
const int potentiometer_ManualInputPin = A0; // Potentiometer pin for manual input

// Button pins state initialize
const boolean switchReleaseState = HIGH; // Initial switch release state (HIGH)

// Sensor pins setup
const int phototransistorPin = A1; // phototransistor input pin
const int temperatureSensorPin = A2; // temperature sensor pin

// Initialize state variable
enum State { INITIALIZE, MANUAL, SUMMER, FALL_SPRING, WINTER };
State currentState = INITIALIZE;

// Proportional control gain
const float kProp = 3.;  // Kprop value in V/rad

// Timing variables for loop control
unsigned long previousTime = millis();
const int deltaT = 10;  
const float delayTime_Switches = 300;

void setup() {
  // Set motor encoder pins as inputs in pullup mode
  pinMode(encoderAPin, INPUT_PULLUP);
  pinMode(encoderBPin, INPUT_PULLUP);
  // Set motor control pins as outputs
  pinMode(enablePin, OUTPUT);
  pinMode(input1Pin, OUTPUT);
  pinMode(input2Pin, OUTPUT);

  // Set button pins as inputs in pullup mode
  pinMode(switchPin_ModeSwitch, INPUT_PULLUP); 
  pinMode(switchPin_ManualMode, INPUT_PULLUP); 

  // Attach interrupts to motor encoder pins, both are set to trigger on CHANGE
  attachInterrupt(digitalPinToInterrupt(encoderAPin), aChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBPin), bChange, CHANGE);

  // Start serial communication
  Serial.begin(9600);
}

void loop() {
  // Read the current state of the switches
  boolean autoModeSwitch = digitalRead(switchPin_ModeSwitch);
  boolean manualModeSwitch = digitalRead(switchPin_ManualMode);

  if (manualModeSwitch != switchReleaseState) {
    manualModeSwitch();
    delay(delayTime_Switches);
  }

  if (autoModeSwitch != switchReleaseState) {
    autoModeSwitch();
    delay(delayTime_Switches);
  }
  
  // Read the sensor readings
  int potentiometerValue = analogRead(potentiometer_ManualInputPin);
  int lightValue = analogRead(phototransistorPin);
  int temperatureValue = analogRead(temperatureSensorPin);

  int counterDesired = counterDetermination(potentiometerValue, lightValue, temperatureValue);

  // Calculate error in counts and convert to radians
  int errorCounts = counterEncoder - counterDesired;
  float errorRadians = errorCounts * (2 * PI) / (12 * 4 * 4.4);

  // Calculate control signal using proportional control
  float controlSignal = kProp * errorRadians;

  // Determine motor direction based on the sign of the control signal
  if (controlSignal > 0) {
    digitalWrite(input1Pin, HIGH);
    digitalWrite(input2Pin, LOW);
  } else {
    digitalWrite(input1Pin, LOW);
    digitalWrite(input2Pin, HIGH);
  }

  // Scale and constrain control signal to 0-255 range
  int scaledControl = constrain(abs(controlSignal) * 255 / (2 * PI) * kProp, 0, 255);

  // Send control signal to the Enable1 pin on the H-Bridge using analogWrite(
  analogWrite(enablePin, scaledControl);

  // Timing control with while loop
  while (millis() - previousTime < deltaT) {}
  previousTime = millis();
}

void manualModeSwitch() {
  /*
  Mode switching between SUMMER, FALL_SPRING, and WINTER.
  Print out current mode.
  */
  if (currentState == MANUAL) currentState = INITIALIZE;
  else currentState = MANUAL;

  Serial.println("Current mode: " + String(currentState));
}

void autoModeSwitch() {
  /*
  Mode switching between SUMMER, FALL_SPRING, and WINTER.
  Print out current mode.
  */
  if (currentState == SUMMER) currentState = FALL_SPRING;
  else if (currentState == FALL_SPRING) currentState = WINTER;
  else if (currentState == WINTER) currentState = SUMMER;
  else currentState = INITIALIZE;

  Serial.println("Current mode: " + String(currentState));
}

int counterDetermination(int& potentiometerValue, int& lightValue, int& temperatureValue){
  /*
  Calculation of desired window blind angle.

  Args:
      direction_Stepper [bool]: The direction of the stepper motor rotation.
      currentIncrement [int]: Current phase index.

  Returns:
      nextPhase [int]: The index of the next phase.
  */
  int counterDesired;

  switch (currentState) {
    case INITIALIZE:
      // Initialize currentState to one of the states base on relation of lightValue and temperatureValue
      float stateFactor = lightValue * 0.1 + temperatureValue * 5;
      if (stateFactor > 100) currentState = SUMMER;
      else if (stateFactor > 50) currentState = FALL_SPRING;
      else currentState = WINTER;

      Serial.println("Current mode: " + String(currentState));
      break;
    
    case MANUAL:
      // Map value to encoder counts based on potentiometer value
      counterDesired = map(potentiometerValue, 0, 1023, 0, 12 * 4.4 * 4);
      break;

    case SUMMER:
      counterDesired = 0;
      break;
      
    case FALL_SPRING:
      counterDesired = 0;
      break;

    case WINTER:
      counterDesired = 0;
      break;
  }

  return counterDesired;
}


void aChange() {
  /*
  Function for tracking motor encoder
  */
  // Read the current state
  aState = digitalRead(encoderAPin);
  bState = digitalRead(encoderBPin);
  // If aState is different from bState, plus one to counterEncoder. Otherwise, minus one.
  if (aState != bState) {
    counterEncoder++;
  } else {
    counterEncoder--;
  }
}

void bChange() {
  /*
  Function for tracking motor encoder
  */
  // Read the current state
  aState = digitalRead(encoderAPin);
  bState = digitalRead(encoderBPin);
  // If aState is the same as bState, plus one to counterEncoder. Otherwise, minus one.
  if (aState == bState) {
    counterEncoder++;
  } else {
    counterEncoder--;
  }
}

