// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
// Requires the Adafruit_Motorshield v2 library 
//   https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library
// And AccelStepper with AFMotor support 
//   https://github.com/adafruit/AccelStepper

// This tutorial is for Adafruit Motorshield v2 only!
// Will not work with v1 shields

// Why won't my stepper motor go any faster?
// Since the shield is controlled by i2c, the maximum step rate is limited by the i2c bus speed. 
//The default bus speed is 100KHz and can be increased to 400KHz 
//by editing the library file in your Arduino installation folder. 
//The file can be found in hardware/libraries/wire/utility/twi.h.
// Find the line with: "#define TWI_FREQ 100000L"
// and change it to "#define TWI_FREQ 400000L"

#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *stepperMotor1_table = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *stepperMotor2_head = AFMS.getStepper(200, 2);

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!

void forwardstep1_table() {  
  stepperMotor1_table->onestep(FORWARD, DOUBLE);
}
void backwardstep1_table() {  
  stepperMotor1_table->onestep(BACKWARD, DOUBLE);
}

void forwardstep2_head() {  
  stepperMotor2_head->onestep(FORWARD, MICROSTEP);
}
void backwardstep1_head() {  
  stepperMotor2_head->onestep(BACKWARD, MICROSTEP);
}

AccelStepper aStepper1_table(forwardstep1_table, backwardstep1_table); // use functions to step

AccelStepper aStepper2_head(forwardstep2_head, backwardstep1_head); // use functions to step


int speed_aStepper1_table = 1;
int speed_aStepper2_head = 255;

long duration_tip120_open_ms = 100;
long duration_tip120_closed_ms = 900;

long duration_head_rest = 2000;

int outOfBoundSteps = 200;
int behindHomeSteps = 200;

long position_end_aStepper2_head = 100 * 65;


// connect setup_microswitch from 5v pin 3 and 10k resistor ground
const int setupPin = 3;
int setupPinState = 0;
boolean setupPhase = true;

// connect out_of_bound_microswitch from 5v to pin 2 and 10k resistor ground
const int outOfBoundPin = 2;
int outOfBoundState = 0;

const int headPin = 4;

long lastTime = 0;

int setupPinTriggered = false;

//long time_headStarted = 0;
long time_headLastSeenRunning = 0;
long time_tip120_lastOpen = 0;
long time_tip120_lastClosed = 0;

void setup()
{  
   
   pinMode (outOfBoundPin, INPUT);
   pinMode (setupPin, INPUT);
   pinMode (headPin, OUTPUT);
   Serial.begin(9600);           // set up Serial library at 9600 bps
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  aStepper1_table.setSpeed(speed_aStepper1_table);
  aStepper2_head.setSpeed(speed_aStepper2_head);
  
  lastTime = millis();
  
  aStepper2_head.moveTo(position_end_aStepper2_head);
}

void loop()
{ 
   outOfBoundState = digitalRead(outOfBoundPin);
   setupPinState = digitalRead(setupPin);
   
  if (outOfBoundState == HIGH || setupPinState == HIGH)  
  {
     setupPhase = true;
  }
  
  if (setupPhase == true)
  {
    if (aStepper_head.currentPosition() != 0)
    {
      if (setupPin == HIGH && setupPinTriggered == false) // we're touching the home "out of bounds" switch
      {
        setupPinTriggered = true;
        aStepper2_head.setCurrentPosition(0 - behindHomeSteps);
        aStepper2_head.moveTo(0);
        // TODO setPosition(0) ... set its position to 0
      }
      else {
        if (setupPinTriggered == false)
        {
          aStepper2_head.moveTo(-behindHomeSteps);
        }
      }
    }
    setupPhase = false; // and end the setup phase
    time_headLastSeenRunning = time_tip120_lastOpen = millis();
    time_tip120_lastClosed = millis() - duration_tip120_open_ms;
  } 
  else {
    aStepper1_table.runSpeed();
    
    aStepper2_head.run();
    boolean headIsStopped_checkHead = aStepper2_head.distanceToGo() == 0;
    
    // do stuff
    if (headIsStopped_checkHead == true)
    {
      // TODO: close tip120
      digitalWrite (headPin, LOW);
      time_tip120_lastClosed = millis();
      
      long timeSinceHeadLastStopped = millis() - time_headLastSeenRunning;
      
      boolean headShouldStartMoving = (timeSinceHeadLastStopped >= duration_head_rest);
      
      if (headShouldStartMoving == true) {
        boolean headShouldMoveForward = true; // 1 is forward (and -1 is backward)
        if (aStepper2_head.currentPosition() == position_end_aStepper2_head)
        {
          headShouldMoveForward = false; // false is equivalent to 0 - we make use of this in the direction multiplication
        }
        aStepper2_head.moveTo(position_end_aStepper2_head * headShouldMoveForward);
        // aStepper2_head.run(); // we'll actually wait until the stepper.run() call in the next loop
      }
    } else { // head is running!

      if (time_tip120_lastOpen >= time_tip120_lastClosed)
      {
        // it's open - how long has it been open?
        // if we've been open for long enough, we want to close
        if (time_tip120_lastOpen >= duration_tip120_open_ms)
        {
          // TODO: close - send low signal to transistor pin
          digitalWrite (headPin, LOW);
          time_tip120_lastClosed = millis();
        }
      } else {
        // tip120 is closed, check duration and open it if delay is elapsed
        if (time_tip120_lastClosed >= duration_tip120_closed_ms)
        {
          // TODO: open - send high signal to transistor pin
          digitalWrite (headPin, HIGH);
          time_tip120_lastClosed = millis();
        }
      }
      time_headLastSeenRunning = millis();
    }
  }
  lastTime = millis();
}