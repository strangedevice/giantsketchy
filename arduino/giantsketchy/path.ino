
// path.pde

// Manage creation and replay of paths for delta bot

struct point
{
  byte x,y,z;
};

#define MAX_POINTS 2048
struct point points[MAX_POINTS];

// Scale the entire drawing by a fixed amount
#define SCALE_FACTOR 1.25
//#define SCALE(p) (SCALE_FACTOR * (float)(p))
#define SCALE(p) (p)

int nextPoint = -1;

boolean loopPath()
{
  if( motionWorking )
  {
        #ifdef DO_LOGGING
   //Serial.println ("motionWorking");

  #endif
    return true; // still going to our last position
  }
    
  if( ! pathWorking )
  {
  #ifdef DO_LOGGING
   //Serial.println ("! pathWorking");

  #endif
    return false;
  }
  
  if( nextPoint+1 >= numPoints )
  {
    pathWorking = false;  // done all our work, will never run again
    
    logSummary();
    return false;
  }
  
  nextPoint ++;
  
  logPoint();
  
  boolean stopAtEnd = true;
  
  if (nextPoint+1 < numPoints)
  {
    if (points[nextPoint].z == points[nextPoint+1].z)
      stopAtEnd = false;
  }
   
  moveTo(SCALE(points[nextPoint].x - 128), SCALE(points[nextPoint].y - 128), points[nextPoint].z - 128, stopAtEnd);
  
  pathWorking = true;
  return true;  
}

void stopPath()
{
  nextPoint = numPoints;
  pathWorking = false; 
}

void clearPath()
{
  numPoints = 0;
}

void startPath()
{
#ifdef DO_LOGGING
  Serial.println ("startPath");
  Serial.println (numPoints);
#endif
   
  nextPoint = 0;
  pathWorking = true; 
}

boolean addPoint(int x, int y, int z, int overhead)  // in range -127, +127
{
  if (numPoints + overhead + 1 >= MAX_POINTS)
    return false;
       
   points[numPoints].x = x + 128;
   points[numPoints].y = y + 128;
   points[numPoints].z = z + 128;
   
   ++numPoints;   
   return true;
}

void logPoint()
{
#ifdef DO_LOGGING
    Serial.print ("Point ");
    Serial.print (nextPoint);
    Serial.print (": ");
    Serial.print (points[nextPoint].x-128);
    Serial.print (", ");
    Serial.print (points[nextPoint].y-128);
    Serial.print (", ");
    Serial.print (points[nextPoint].z-128);
    Serial.print ("\n");
#endif
}

void logSummary()
{
#ifdef DO_LOGGING
    Serial.print ("numPoints: ");
    Serial.print (numPoints);
    Serial.print (", totalDistance: ");
    Serial.print (totalDistance);
    Serial.print ("mm, totalSteps: ");
    Serial.print (totalSteps);
    Serial.print (", totalMillis: ");
    Serial.print (totalMillis);
    Serial.print ("\n");
    
    Serial.print ("msec/step: ");
    Serial.print (totalMillis/totalSteps);
    Serial.print (", mm/step: ");
    Serial.print (totalDistance/totalSteps);
    Serial.print ("\n");
    
    Serial.print ("max servo speed (s3003 does 0.2): ");
    Serial.print ( 60.0 / (max_servo_speed * 1000));
    Serial.print (" secs/60 deg");
    Serial.print ("\n");
    
    Serial.print ("mean servo speed: ");
    Serial.print ( 60.0 / ((sum_servo_speed/num_servo_speed) * 1000));
    Serial.print (" secs/60 deg");
    Serial.print ("\n");    
#endif
}

