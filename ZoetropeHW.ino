#include <Arduino.h>
#include <SpeedyStepper.h>
#include <FlexyStepper.h>

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

#define MICROSTEPPING 8
#define TRANSMISSION 5
#define BASE_STEPS_PER_REV 200
#define MIN_SPEED 0.075

int beamState = LOW;
int lastBeamState = LOW;
bool beamCount = false;
int backlightState = LOW;
int lastBacklightState = LOW;
bool backlightCount = false;
int motorState = LOW;
int lastMotorState = LOW;
bool motorCount = false;

int potVal = 0;
int lastPotVal = 0;

FlexyStepper stepper;

void setup() {
  ////// Pinmode all ports ///////////

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
  stepper.setSpeedInRevolutionsPerSecond(1.0);
  stepper.setAccelerationInRevolutionsPerSecondPerSecond(0.2);
  stepper.setCurrentPositionInRevolutions(0.0);
  stepper.setTargetPositionRelativeInRevolutions(1000.0);
  stepper.disableStepper();

  ///////// Initialize Lights and Button Lights //////////

  digitalWrite(MOTOR_BUTTON_LIGHT, motorState);
  digitalWrite(BEAM_BUTTON_LIGHT, beamState);
  digitalWrite(BEAM_LEDS, beamState);
  digitalWrite(BACKLIGHT_BUTTON_LIGHT, backlightState);
  digitalWrite(BACKLIGHT_LEDS, backlightState);

}

void loop() {

  //////////// Non-Latching Buttons require Button States /////////
  beamState = !digitalRead(BEAM_BUTTON);
  backlightState = !digitalRead(BACKLIGHT_BUTTON);
  motorState = !digitalRead(MOTOR_BUTTON);
  potVal = analogRead(MOTOR_SPEED_POT);

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

  if (motorCount && potVal != lastPotVal) stepper.setSpeedInRevolutionsPerSecond(MIN_SPEED + (float)potVal / 1500.0);

  stepper.processMovement();

  ///////////// Save Last States For Next Loop Iteration /////////////
  lastMotorState = motorState;
  lastBeamState = beamState;
  lastBacklightState = backlightState;
  lastPotVal = potVal;

}
