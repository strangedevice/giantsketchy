
#include "StepperWrapper.h" 


#define CYCLOID_RATIO 21 //TODO
#define MICROSTEPS 4 //TODO
#define STEPS_PER_REV 200 //TODO

// Give up if we haven't hit the home switch after this many steps
#define HOME_STEPS ((MICROSTEPS * STEPS_PER_REV * CYCLOID_RATIO) / 2)

#define SAFE_LIMIT ((MICROSTEPS * STEPS_PER_REV * CYCLOID_RATIO) / 4)


StepperWrapper::StepperWrapper(int _stepPin, int _dirPin, int _enablePin, int _limitPin) : 
            stepper(AccelStepper::DRIVER, _stepPin, _dirPin),
            limitPin( _limitPin ),
            homing( true )
    {
      pinMode(limitPin, INPUT_PULLUP); // active LOW
      
      stepper.setEnablePin(_enablePin);
      stepper.setPinsInverted(false, false, true); // ENABLE is inverted on Pololu driver
   
      stepper.setAcceleration(4000); // arbitrary
 
    }
    
   
   void StepperWrapper::loop()
   {
     stepper.run();
     
     if( homing )
     {     
        if( digitalRead(limitPin) == LOW ) // read switch - have we arrived ?
        {
          // Serial.print("StepperWrapper::loop homing ");
          // Serial.println(stepper.currentPosition());
          
          // move toward switch a little bit
          // stepper.move(-1);
        }
        else
        {         
          Serial.print ("StepperWrapper::loop homed\n");

          // we are home, set current position as zero, stop homing
          stepper.setCurrentPosition( 0 );
          stepper.moveTo(0); 
          homing = false;
          stepper.setMaxSpeed(4000);
        }
     }
   }
   
   
   void StepperWrapper::enable( bool enable )
   {
      if (enable) {
        stepper.enableOutputs();
      } else {
        stepper.disableOutputs();
      }
   }
 
   void StepperWrapper::home()
   {
      Serial.print ("StepperWrapper::home\n");
      homing = true;
      stepper.setMaxSpeed(400);
      stepper.setCurrentPosition(0);
      stepper.moveTo(-HOME_STEPS);
      // work continues in loop
   }
   
   boolean StepperWrapper::isHome() {
     return !homing;
   } 
   
   long stepsForAngle( float theta )
  {
    return theta * CYCLOID_RATIO * MICROSTEPS * STEPS_PER_REV / 360.0 ;
  }

  void StepperWrapper::setSpeedAndPosition( float degPerMillisec, float targetAngle )
  {
      // Serial.println("StepperWrapper::set speed and position");
      
      long targetSteps = stepsForAngle( targetAngle );
      
      if (targetSteps < 0) 
          targetSteps = 0; // safety, do not pass 0
      else if (targetSteps > SAFE_LIMIT)
          targetSteps = SAFE_LIMIT;
          
      long stepsPerSec = stepsForAngle( degPerMillisec * 1000.0 );
      stepper.setSpeed(stepsPerSec);
      stepper.moveTo( targetSteps );
  }
  
   
