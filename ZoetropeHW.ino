#include <SpeedyStepper.h>

#include <FlexyStepper.h>

#include <Arduino.h>

#include <TimerOne.h>

#include <TimerThree.h>

////// Creating and assigning variables ///////////
#define TIMER_US 100000
#define TICK_COUNTS 100
#define TICK_COUNTS1 300

volatile long tick_count = TICK_COUNTS;
volatile long tick_count1 = TICK_COUNTS1;
volatile bool in_long_isr = false;

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

int i = 0;
int g = 0;

unsigned long startTime;

FlexyStepper stepper;

void setup() {    //issues: overlapping timers. turning off beam while motor is on. turning on beam - motor time stays the same, doesn't restart -- solution: turn on beam when motor is on; turn off beam when motor is off
  ////// Pinmode all ports ///////////

  Serial.begin(9600);
  Serial.println("[debug] setup()");   

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

  digitalWrite(MOTOR_BUTTON_LIGHT, motorState); //sets motor button light to LOW
  digitalWrite(BEAM_BUTTON_LIGHT, beamState); //sets beam button light to LOW
  digitalWrite(BEAM_LEDS, beamState); //sets beam LEDs to LOW
  digitalWrite(BACKLIGHT_BUTTON_LIGHT, backlightState); //sets backlight button lights to LOW
  digitalWrite(BACKLIGHT_LEDS, backlightState); //sets backlight LEDs to LOW

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

void timerIsr()
{
  Serial.print("[debug] time for beam ");     //prints each 0.1 second, e.g. at "[debug] time 100", 10 seconds have passed. 
  Serial.println(i);
  i++;

  if (!(--tick_count))                             // Count to 10S
  {
    tick_count = TICK_COUNTS;                      // Reload
    tick_beam_isr();                                 // Call the beam routine 
  }

}

void timerIsr1()
{
  Serial.print("[debug] time for motor ");     //prints each 0.1 second, e.g. at "[debug] time 100", 10 seconds have passed. 
  Serial.println(g);
  g++;

  if (!(--tick_count1))                             // Count to 30S
  {
    tick_count1 = TICK_COUNTS1;                      // Reload
    tick_motor_isr();                                 // Call the motor routine
  }
}

void tick_beam_isr()
{
  if (in_long_isr)
    return;

  in_long_isr = true;

  volatile long i;

  interrupts();  

  if (motorCount == false) {  //don't turn the beam off when the motor is on
    turnOffBeam();
    beamState = LOW;
    beamCount = false;
  }

  noInterrupts();
  in_long_isr = false;
}

void tick_motor_isr()
{
  if (in_long_isr)
    return;

  in_long_isr = true;

  volatile long i;

  interrupts();

  if (motorCount == true) {
    turnOffMotor();
    motorState = LOW;
    motorCount = false;
  }

  noInterrupts();
  in_long_isr = false;
}

void updateBeamState() {
  beamState = !digitalRead(BEAM_BUTTON);

  if (beamState != lastBeamState && beamState == HIGH) {   //if beamState is HIGH      //beamState is whether beam button is on or off
    beamCount = !beamCount;   //make beamCount TRUE

    if (beamCount == false) {      //if beamCount is FALSE, make beam button light & LEDs LOW
      turnOffBeam();
    }
    else {    //if beamCount is TRUE, make beam button light & LEDs HIGH.
      turnOnBeam();
    }
  }

  lastBeamState = beamState;
  //stepper.processMovement();
}

void turnOffBeam() {
  //turns off beam light
  digitalWrite(BEAM_BUTTON_LIGHT, LOW);
  digitalWrite(BEAM_LEDS, LOW);
  Serial.println("[debug] updateBeamState() off");
  //stops timer
  Timer1.stop();
    //Timer1.detachInterrupt();
  Serial.println("[debug] beam timer stops");
  i = 0;
  g = 0;
  tick_count = TICK_COUNTS;  //this restarts the beam timer to TICK_COUNTS
  tick_count1 = TICK_COUNTS1;  //this restarts the motor timer to TICK_COUNTS1
}
void turnOnBeam () {
  //turns on beam light
  digitalWrite(BEAM_BUTTON_LIGHT, HIGH);
  digitalWrite(BEAM_LEDS, HIGH);
  Serial.println("[debug] updateBeamState() on");
  //starts timer
  Serial.println("[debug] beam timer starts");
  tick_count = TICK_COUNTS;  //this restarts the beam timer to TICK_COUNTS
  tick_count1 = TICK_COUNTS1;  //this restarts the motor timer to TICK_COUNTS1
  i = 0;
  g = 0;
  Timer1.initialize(TIMER_US);
  Timer1.attachInterrupt(timerIsr);
//  Timer1.resume();
}

void updateBacklightState() {
  backlightState = !digitalRead(BACKLIGHT_BUTTON);

  if (backlightState != lastBacklightState && backlightState == HIGH) {   //if backlightState is HIGH
    backlightCount = !backlightCount;   //make backlightCount TRUE

    if (backlightCount == false) {     //if backlightCount is FALSE, make backlight button light and LEDs LOW
      digitalWrite(BACKLIGHT_BUTTON_LIGHT, LOW);
      digitalWrite(BACKLIGHT_LEDS, LOW);
    }
    else {     //if backlightCount is TRUE, make backlight button light and LEDs HIGH
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
  if (digitalRead(LID_SENSOR)) { // if lid is open disable stuff
    motorCount = false;
    digitalWrite(MOTOR_BUTTON_LIGHT, LOW);
    stepper.disableStepper();
  }

  else if (motorState != lastMotorState && motorState == HIGH) {    //if motorState is HIGH      //motorState is whether motor button is turned on or off
    motorCount = !motorCount;     //make motorCount TRUE

    if (motorCount == false) {     //if motorCount is FALSE, make motor button light LOW
      turnOffMotor();
    }
    else {
      turnOnMotor();
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

void turnOffMotor() {
  //turn motor off
  digitalWrite(MOTOR_BUTTON_LIGHT, LOW);
  stepper.disableStepper();
  Serial.println("[debug] turnOffMotor() off");
  //stop timer
  Timer3.stop();
  Serial.println("[debug] motor timer stops");
  //resets encoder
  g = 0;
  i = 0;
  tick_count = TICK_COUNTS;  //this restarts the beam timer to TICK_COUNTS
  tick_count1 = TICK_COUNTS1;  //this restarts the motor timer to TICK_COUNTS1
  potVal = 0;
  lastPotVal = 0;
  prevPotVal = 0;
  outputVal = 0;
}

void turnOnMotor() {
  //turn on motor
  digitalWrite(MOTOR_BUTTON_LIGHT, HIGH);    //if motorCount is TRUE, make motor button light HIGH
  stepper.resetVelocity();
  stepper.setTargetPositionRelativeInRevolutions(1000.0);
  stepper.enableStepper();
  Serial.println("[debug] turnOnMotor() on");
  //starts timer
  Serial.println("[debug] motor timer starts");
  tick_count = TICK_COUNTS;  //this restarts the beam timer to TICK_COUNTS
  tick_count1 = TICK_COUNTS1;  //this restarts the motor timer to TICK_COUNTS1
  i = 0;
  g = 0;
  Timer3.initialize(TIMER_US);
  Timer3.attachInterrupt(timerIsr1);
//  Timer3.resume();
}

void updateStepperMovement() {
  stepper.processMovement();
}

void filterPotVal() {  // allows the limitless potentiometer to work without "overflowing" back to zero
  int newPotVal = analogRead(A0);

  if (abs(newPotVal - prevPotVal) > overflowThreshold) {
    if (prevPotVal > overflowThreshold)
      outputVal = min(outputVal + 1, maxOutputVal);
    else
      outputVal = max(outputVal - 1, 0);
  }

  else if (abs(newPotVal - prevPotVal) > filterThreshold) {
    Serial.println("[debug] potentiometer2");
    //resets timer
    tick_count = TICK_COUNTS;
    tick_count1 = TICK_COUNTS1;
    i = 0;
    g = 0;
    if (newPotVal > prevPotVal)
      outputVal = min(outputVal + 1, maxOutputVal);
    else
      outputVal = max(outputVal - 1, 0);

    prevPotVal = newPotVal;
  }
}


