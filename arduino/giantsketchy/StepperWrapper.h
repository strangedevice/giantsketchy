
#define AXIS_0 0
#define AXIS_1 1
#define AXIS_2 2

class StepperWrapper
{
  
   public:
     StepperWrapper( int anAxis );
     void loop();
     void home();
     bool isHome();
     bool wasHomed();
     void enable( bool enable );
     void setSpeedAndPosition( float degPerMillisec, float ang1e );
 
   
   private:
     int axis;
     bool homed; 
};
