#include <Arduino.h>
#include <SpeedyStepper.h>
#include <FlexyStepper.h>

  ////// Creating and assigning variables ///////////

#define BEAM_BUTTON_LIGHT 24
#define BEAM_BUTTON 25
#define BEAM_LEDS 10

#define BACKLIGHT_BUTTON_LIGHT 26
#define BACKLIGHT_BUTTON 27
#define BACKLIGHT_LEDS 9

#define LID_SENSOR 28

#define MOTOR_BUTTON_LIGHT 8
#define MOTOR_BUTTON 23
#define MOTOR_SPEED_POT A0
#define MOTOR_PORT 1

// be careful when messing with these 4 varibles below because they've been tweaked to work with
// the current set up. Changing them may result in loss of RPM or unusual noises.

#define MICROSTEPPING 4
#define TRANSMISSION 5
#define BASE_STEPS_PER_REV 200
#define MIN_SPEED 0.2

bool beamState = LOW;
bool lastBeamState = LOW;
bool beamCount = false;

int backlightState = LOW;
int lastBacklightState = LOW;
bool backlightCount = false;

int motorState = LOW;
int lastMotorState = LOW;
bool motorCount = false;

int potVal = 0;
int lastPotVal = 0;

int prevPotVal = analogRead(MOTOR_SPEED_POT);

int outputVal = 0;
int maxOutputVal = 100;

int overflowThreshold = 1023 / 2;
int filterThreshold = 3;


unsigned long startTime;

FlexyStepper stepper;

void setup() {
  ////// Pinmode all ports ///////////

  //Serial.begin(9600);

  pinMode(MOTOR_BUTTON_LIGHT, OUTPUT);
  pinMode(MOTOR_BUTTON, INPUT_PULLUP);
  pinMode(MOTOR_SPEED_POT, INPUT_PULLUP);

  pinMode(BEAM_BUTTON_LIGHT, OUTPUT);
  pinMode(BEAM_BUTTON, INPUT_PULLUP);
  pinMode(BEAM_LEDS, OUTPUT);

  pinMode(BACKLIGHT_BUTTON_LIGHT, OUTPUT);
  pinMode(BACKLIGHT_BUTTON, INPUT_PULLUP);
  pinMode(BACKLIGHT_LEDS, OUTPUT);

  pinMode(LID_SENSOR, INPUT_PULLUP);

  ////////// Initialize Stepper //////////

  stepper.connectToPort(MOTOR_PORT);
  stepper.setStepsPerRevolution(BASE_STEPS_PER_REV * TRANSMISSION * MICROSTEPPING);
  stepper.setSpeedInRevolutionsPerSecond(3);
  stepper.setAccelerationInRevolutionsPerSecondPerSecond(0.2);
  stepper.setCurrentPositionInRevolutions(0.0);
  stepper.setTargetPositionRelativeInRevolutions(100000.0);
  stepper.disableStepper();

  ///////// Initialize Lights and Button Lights //////////

  digitalWrite(MOTOR_BUTTON_LIGHT, motorState);
  digitalWrite(BEAM_BUTTON_LIGHT, beamState);
  digitalWrite(BEAM_LEDS, beamState);
  digitalWrite(BACKLIGHT_BUTTON_LIGHT, backlightState);
  digitalWrite(BACKLIGHT_LEDS, backlightState);

  startTime = millis();
}

void loop() {

  ////////// Non-Latching Buttons require Button States /////////
  stepper.processMovement();

  if (millis() - startTime > 100) {
    updateBeamState();
    updateBacklightState();
    updateMotorState();
    startTime = millis();
  }

}

void updateBeamState() {
  beamState = !digitalRead(BEAM_BUTTON);

  if (beamState != lastBeamState && beamState == HIGH) {
    beamCount = !beamCount;

    if (beamCount == false) {
      digitalWrite(BEAM_BUTTON_LIGHT, LOW);
      digitalWrite(BEAM_LEDS, LOW);
    }
    else {
      digitalWrite(BEAM_BUTTON_LIGHT, HIGH);
      digitalWrite(BEAM_LEDS, HIGH);
    }
  }

  lastBeamState = beamState;
  //stepper.processMovement();
}

void updateBacklightState() {
  backlightState = !digitalRead(BACKLIGHT_BUTTON);

  if (backlightState != lastBacklightState && backlightState == HIGH) {
    backlightCount = !backlightCount;

    if (backlightCount == false) {
      digitalWrite(BACKLIGHT_BUTTON_LIGHT, LOW);
      digitalWrite(BACKLIGHT_LEDS, LOW);
    }
    else {
      digitalWrite(BACKLIGHT_BUTTON_LIGHT, HIGH);
      digitalWrite(BACKLIGHT_LEDS, HIGH);
    }
  }
  lastBacklightState = backlightState;
  //stepper.processMovement();
}

void updateMotorState() {
  motorState = !digitalRead(MOTOR_BUTTON);
  potVal = analogRead(MOTOR_SPEED_POT);
  filterPotVal();
  if (digitalRead(LID_SENSOR)) { // if lid is open disable shit
    motorCount = false;
    digitalWrite(MOTOR_BUTTON_LIGHT, LOW);
    stepper.disableStepper();
  }

  else if (motorState != lastMotorState && motorState == HIGH) {
    motorCount = ! motorCount;

    if (motorCount == false) {
      digitalWrite(MOTOR_BUTTON_LIGHT, LOW);
      stepper.disableStepper();
    }
    else {
      digitalWrite(MOTOR_BUTTON_LIGHT, HIGH);
      stepper.resetVelocity();
      stepper.setTargetPositionRelativeInRevolutions(1000.0);
      stepper.enableStepper();
    }
    delay(50); // debouncing delay
  }

  if (motorCount && potVal != lastPotVal) {
    stepper.setSpeedInRevolutionsPerSecond(MIN_SPEED + (2 * (float)outputVal / 100.0));
  }
  lastMotorState = motorState;
  lastPotVal = potVal;
  //stepper.processMovement();
}



void updateStepperMovement() {
  stepper.processMovement();
}

void filterPotVal() {  // allows the limitless potentiometer to work without "overflowing" back to zero
  int newPotVal = analogRead(A0);

  if (abs(newPotVal - prevPotVal) > overflowThreshold) {
    if (prevPotVal > overflowThreshold) outputVal = min(outputVal + 1, maxOutputVal);
    else outputVal = max(outputVal - 1, 0);
  }

  else if (abs(newPotVal - prevPotVal) > filterThreshold) {
    if (newPotVal > prevPotVal) outputVal = min(outputVal + 1, maxOutputVal);
    else outputVal = max(outputVal - 1, 0);
  }

  prevPotVal = newPotVal;
}
