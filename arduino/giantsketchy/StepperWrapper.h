#include <AccelStepper.h> 

class StepperWrapper
{
  
   public:
     StepperWrapper( int stepPin, int dirPin, int enablePin, int limitPin );
     void loop();
     void home();
     boolean isHome();
     void enable( bool enable );
     void setSpeedAndPosition( float degPerMillisec, float a1 );
 
   
   private:
     AccelStepper stepper;
     int limitPin;
     boolean homing;
  
};
