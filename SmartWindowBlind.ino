// Stepper motor setup
const int numPhases = 4; // Number of phases in the stepper motor
const float degreesPerStep_stepper = 7.5; // Number of degrees per step for stepper motor shaft itself
const float gearRation_stepper2Blind = 1; // Gear ratio between stepper motor output shaft and window blind shaft **** NEED UPDATE
const float degreePerStep_blind = degreesPerStep_stepper * gearRation_stepper2Blind; // Blind rotation degrees for one stepper motor step
const int incrementsPerStep = 1; // Number of increments per step
const int incrementsPerPhaseCycle = numPhases / incrementsPerStep; // Number of increments per phase cycle
const float delayTime_Stepper = 50; // Delay time in milliseconds

const int phasePins[numPhases] = {2, 3, 4, 5}; // Array of phase digital pin numbers

unsigned long SwitchedTime; // Recording time for stepper motor to idle
unsigned long idleStartTime = millis(); // Initialize the idle start time with the current time
const int idleTimeTurnOff = 5000; // Time in milliseconds to turn off the motor after inactivity

const boolean polesOnOff[incrementsPerPhaseCycle][numPhases] = {
  {true, true, false, false},  // Step 1
  {false, true, true, false},  // Step 2
  {false, false, true, true},  // Step 3
  {true, false, false, true}   // Step 4
};

int currentIncrement = 0; // Initialize the current phaseIncrement variable
boolean direction_Stepper = false; // The direction of the stepper motor rotation ***NEED TEST TO DETERMINE INITIALIZING AS FALSE OR TRUE***


// Button variables setup
// const int switchPin_ = 6; // 
const int switchPin_ModeSwitch = 7; // 
const int switchPin_ManualMode = 8; // 

boolean switchReleaseState = HIGH; // Initial switch release state (HIGH)
boolean autoModeSwitch;
boolean manualModeSwitch;

// Sensor setup
const int encoderPinA = 9; // connect the A output of the absolute encoder to this pin
const int encoderPinB = 10; // connect the B output of the absolute encoder to this pin
Encoder absEncoder(encoderPinA, encoderPinB);

const int phototransistorPin = 11; // connect the phototransistor to this analog pin
const int temperatureSensorPin = 12; // connect the temperature sensor to this analog pin

// Initialize state variable
enum State { INITIALIZE, MANUAL, SUMMER, FALL_SPRING, WINTER };
State currentState = INITIALIZE;

const float delayTime_Switches = 500;

void setup() {
  Serial.begin(9600);
  // Loop for setting pins
  for (int pins = 0; pins < numPhases; pins++) {
    pinMode(phasePins[pins], OUTPUT); // Set phase pins as OUTPUT
    digitalWrite(phasePins[pins], LOW); // Initialize phase pins to LOW
  }
  pinMode(switchPin_360Rotation, INPUT_PULLUP); // 
  pinMode(switchPin_ModeSwitch, INPUT_PULLUP); // 
  pinMode(switchPin_ManualMode, INPUT_PULLUP); // 
}

void loop() {
  // Read the current state of the switches
  autoModeSwitch = digitalRead(switchPin_ModeSwitch);
  manualModeSwitch = digitalRead(switchPin_ManualMode);

  // Read the readings from the sensors
  int angleCurrent = map(absEncoder.read(), 0, 1023, 0, 360);
  int lightLevel = analogRead(phototransistorPin);
  int temperature = map(analogRead(temperatureSensorPin), 0, 1023, 0, 100);

  int angleDesired = angleDetermination(angleCurrent, lightLevel, temperature);

  if (manualModeSwitch != switchReleaseState) {
    currentState = MANUAL;
    delay(delayTime_Switches);
  }

  if (autoModeSwitch != switchReleaseState) {
    modeSwitch();
    delay(delayTime_Switches);
  }

  // Rotate the stepper motor if current angle isn't the desired angle
  if (angleCurrent != angleDesired) stepperMotorRotation(angleDesired, angleCurrent);

  // // Turn off all phase pins if idle time exceeds the threshold
  // if (millis() - idleStartTime > idleTimeTurnOff) {
  //   // Loop through to turn off all phase digital pins
  //   for (int phase = 0; phase < numPhases; phase++) {
  //     digitalWrite(phasePins[phase], LOW); 
  //   }
  // }
}

int phaseIncrement(bool direction_Stepper, int currentIncrement) {
  /*
  Calculation of increment for phase increment.

  Args:
      direction_Stepper [bool]: The direction of the stepper motor rotation.
      currentIncrement [int]: Current phase index.

  Returns:
      nextPhase [int]: The index of the next phase.
  */
  if (direction_Stepper) currentIncrement += 3; // Increment by 3 when the direction switch is active
  else currentIncrement++;
  int nextPhase = currentIncrement % numPhases;
  return nextPhase; // Ensure the phaseIncrement wraps around within the phase count
}

void modeSwitch() {
  /*
  Mode switching between SUMMER, FALL_SPRING, and WINTER.
  Print out current mode.
  */
  if (currentState == SUMMER) currentState = FALL_SPRING;
  else if (currentState == FALL_SPRING) currentState = WINTER;
  else if (currentState == WINTER) currentState = SUMMER;
  else currentState = MANUAL;

  Serial.println("Current mode set to " + String(currentState));
}

int angleDetermination(int angleCurrent, int lightLevel, int temperature){
  /*
  Calculation of desired window blind angle.

  Args:
      direction_Stepper [bool]: The direction of the stepper motor rotation.
      currentIncrement [int]: Current phase index.

  Returns:
      nextPhase [int]: The index of the next phase.
  */
  int angleDesired;

  switch (currentState) {
    case INITIALIZE:
      // Initialize currentState to one of the states base on relation of lightLevel and temperature
      float stateFactor = lightLevel * 0.1 + temperature * 5;
      if (stateFactor > 100) currentState = SUMMER;
      else if (stateFactor > 50) currentState = FALL_SPRING;
      else currentState = WINTER;
      break;
    
    case MANUAL:
      angleDesired = angleCurrent;
      break;

    case SUMMER:
      angleDesired = 0;
      break;
      
    case FALL_SPRING:
      angleDesired = 0;
      break;

    case WINTER:
      angleDesired = 0;
      break;
  }

  return angleDesired
}

void stepperMotorRotation(int angleDesired, int angleCurrent){
  /*
  Stepper motor rotation base on desired angle and current angle.

  Args:
      angleDesired [int]: Desired angle of blinds.
      angleCurrent [int]: Current angle of blinds.
  */
  if (angleDesired > angleCurrent) direction_Stepper = false;
  else direction_Stepper = true;
  // for (int i = 0; i < 360 / degreesPerStep_stepper; i++) { 
  for (int i = 0; i < abs(angleCurrent - angleDesired) / degreesPerStep_stepper; i++) { 
    for (int phase = 0; phase < numPhases; phase++) {
      digitalWrite(phasePins[phase], polesOnOff[currentIncrement][phase]); // Set phase pins according to the current phaseIncrement
    }
    delay(delayTime_Stepper); // Delay for a specified time
    // Increment the current phaseIncrement and wrap around when it reaches the cycle length
    currentIncrement = phaseIncrement(direction_Stepper, currentIncrement); 
  }
  idleStartTime = millis(); // Reset the idle start time when the switch is pressed
}