// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2009 Mike McCauley
// $Id: ConstantSpeed.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>

// For RAMPS 1.4
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

AccelStepper stepperA(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepperB(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperC(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

void initStepper(AccelStepper *stepper, int enablePin) {
   stepper->setEnablePin(enablePin);
   stepper->setPinsInverted(false, false, true); // invert ENABLE
   stepper->setMaxSpeed(4000);

   stepper->setAcceleration(4000);
   stepper->enableOutputs();
      
   stepper->moveTo(10000);
}

void runStepper(AccelStepper *stepper, int limitPin) {
  // If at the end of travel go to the other end
  if (stepper->distanceToGo() == 0)
    stepper->moveTo(-stepper->currentPosition());

  if (digitalRead(limitPin) == 0) {  
    stepper->run();
  } else {
    stepper->stop();
  }
}

void setup()
{ 
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);
  pinMode(Z_MIN_PIN, INPUT_PULLUP);
  
  initStepper(&stepperA, X_ENABLE_PIN);
  initStepper(&stepperB, Y_ENABLE_PIN);
  initStepper(&stepperC, Z_ENABLE_PIN);
}

void loop()
{  
  runStepper(&stepperA, X_MIN_PIN);
  runStepper(&stepperB, Y_MIN_PIN);
  runStepper(&stepperC, Z_MIN_PIN);
}

