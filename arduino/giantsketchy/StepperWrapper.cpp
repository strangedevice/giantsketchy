
#include "StepperWrapper.h"
#include "robot.h"

StepperWrapper::StepperWrapper(int anAxis) : 
    axis(anAxis),
    homed(false)
  {
  }

  void StepperWrapper::loop()
  { 
    // probably better done in the ISR    
    if (!homed) {
      if (isHome()) {
        homed = true;
      }
    }
  }

void StepperWrapper::enable( bool enable )
  {
    if (enable) {
      switch (axis) {
        case AXIS_0: enableX(); break;
        case AXIS_1: enableY(); break;
        case AXIS_2: enableZ(); break;
      }
    } 
    else {
      switch (axis) {
      case AXIS_0: disableX(); break;
      case AXIS_1: disableY(); break;
      case AXIS_2: disableZ(); break;
      }
    }
  }

  void StepperWrapper::home()
  { 
    homed = false;
    
    switch (axis) {
      case AXIS_0: homeX(); break;
      case AXIS_1: homeY(); break;
      case AXIS_2: homeZ(); break;
    }
  }

  bool StepperWrapper::isHome() {
    switch (axis) {
      case AXIS_0: return isHomeX();
      case AXIS_1: return isHomeY();
      case AXIS_2: return isHomeZ();
    }
    
    return false;
  }
  
  bool StepperWrapper::wasHomed() {
    return homed;
  }

  int stepsForAngle(float theta)
  {
    return theta * (float)STEPS_PER_REV / 360.0;
  }

  void StepperWrapper::setSpeedAndPosition(float degPerMillisec, float targetAngle)
  {
    if (!homed) return; // safety
  
    // Serial.println("StepperWrapper::set speed and position");

    int targetSteps = stepsForAngle(targetAngle);

    if (targetSteps < 0) 
      targetSteps = 0; // safety, do not pass 0
    else if (targetSteps > SAFE_LIMIT)
      targetSteps = SAFE_LIMIT;

    int stepsPerSec = stepsForAngle(degPerMillisec * 1000.0);
  
    switch (axis) {
      case AXIS_0:
        setMaxSpeedX(stepsPerSec);
        moveToX(targetSteps);
        break;
      case AXIS_1:
        setMaxSpeedY(stepsPerSec);
        moveToY(targetSteps);
        break;
      case AXIS_2:
        setMaxSpeedZ(stepsPerSec);
        moveToZ(targetSteps);
        break;
    }
  }



