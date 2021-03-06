
// position.pde

// Manages xyz->theta? transformation and servo drive for delta robots
// Maths from http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/

// trigonometric constants
const float sqrt3 = sqrt(3.0);
const float pi = 3.141592653;
const float sin120 = sqrt3 / 2.0;   
const float cos120 = -0.5;        
const float tan60 = sqrt3;
const float sin30 = 0.5;
const float tan30 = 1.0 / sqrt3;
 
// robot geometry for our rig
// (look at pics in link above for explanation)
const float side_from_radius = 2.0 / tan30; // we want the sides of the base and effector triangles, 
                                            // but what we can easily measure is the distance from the center to the midpoint of the side
                                            // so multiply that by 2/tan(30)
                                            
// const float e = side_from_radius * 25.0;    // end effector
// const float f = side_from_radius * 48;      // base
// const float re = 220.0;                     // effector arm length
// const float rf = 37.0;                      // base arm length

const float e = side_from_radius * 40.0;    // end effector
const float f = side_from_radius * 90.0;    // base
const float re = 475.0;                     // effector arm length
const float rf = 150.0;                     // base arm length
 
 // for other dimensional setup, see the top of giantsketchy.ino
 
// limits of servo motion for our rig (see also SAFE_LIMIT in robot.h)
const int minServoPos = 0;
const int maxServoPos = 135;

// see transformToServoAngle for a description of how we use angles
#define HORIZONTAL_ANGLE 52.0

// Retain previous position so deltas can be calculated.
static float last_theta1 = 0.0;
static float last_theta2 = 0.0;
static float last_theta3 = 0.0;

#define STATUS_LED 13 // lights up for bad positions

StepperWrapper s0 = StepperWrapper(AXIS_0);
StepperWrapper s1 = StepperWrapper(AXIS_1);
StepperWrapper s2 = StepperWrapper(AXIS_2);

boolean servosAttached = false;
  
int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3);
int delta_calcAngleYZ(float x0, float y0, float z0, float &theta);
boolean transformToServoAngle(float &theta);

void setupPosition()
{
  pinMode(STATUS_LED, OUTPUT);   
  turnOnServos();
}

void loopPosition()
{
  s0.loop();
  s1.loop();
  s2.loop();
}

boolean anyHoming()
{
  return !s0.wasHomed() || !s1.wasHomed() || !s2.wasHomed();
}

void homePosition()
{
  s0.home();
  s1.home();
  s2.home();
   
  // Servo angle is zero at home position
  last_theta1 = 0.0;
  last_theta2 = 0.0;
  last_theta3 = 0.0;
}

float getHomeZ() {
/*
  float x0, y0, z0;
  int status = delta_calcForward(HORIZONTAL_ANGLE, HORIZONTAL_ANGLE, HORIZONTAL_ANGLE, x0, y0, z0);
  
  if (status == 0) {
#ifdef DO_LOGGING
      Serial.print("Calculated home z: ");
      Serial.println(z0);
#endif
     return z0;
  }
*/

  return 345.0;
}

void turnOnServos()
{
  if(!servosAttached )
  {
    s0.enable(true);
    s1.enable(true);
    s2.enable(true);  
  }
  servosAttached = true;
}

void turnOffServos()
{
  if(servosAttached)
  {
    s0.enable(false);
    s1.enable(false);
    s2.enable(false);
 
  }
  servosAttached = false;
}

int goTo( float x0, float y0, float z0 )
{
  float theta1;
  float theta2;
  float theta3;
      
  static float tLast = 0;
       
  if(0 != delta_calcInverse(x0, y0, z0 + baseZ, theta1, theta2, theta3))
  {  
#ifdef DO_LOGGING
    Serial.print ("Unreachable pos: ");
    Serial.print (x0);
    Serial.print (", ");
    Serial.print (y0);
    Serial.print (", ");
    Serial.print (z0 + baseZ);
    Serial.print ("\n");
#endif
     
    digitalWrite(STATUS_LED,true); //Status LED...
    return 0; // no pos
  }
  
#ifdef DO_LOGGING
/*
  Serial.print ("Pos: ");
  Serial.print (x0);
  Serial.print (", ");
  Serial.print (y0);
  Serial.print (", ");
  Serial.print (z0);
  Serial.print ("\n");
  Serial.print ("Raw angles: ");
  Serial.print (theta1);
  Serial.print (", ");
  Serial.print (theta2);
  Serial.print (", ");
  Serial.print (theta3);
  Serial.print ("\n");
*/
#endif

  if (!transformToServoAngle(theta1) ||
      !transformToServoAngle(theta2) ||
      !transformToServoAngle(theta3))
  {
#ifdef DO_LOGGING
    Serial.print ("Unreachable angles: ");
    Serial.print (theta1);
    Serial.print (", ");
    Serial.print (theta2);
    Serial.print (", ");
    Serial.print (theta3);
    Serial.print ("\n");
#endif

    digitalWrite(STATUS_LED, true);
    return 0;    
  }

#ifdef DO_LOGGING  
  Serial.print ("Angles: ");
  Serial.print (theta1);
  Serial.print (", ");
  Serial.print (theta2);
  Serial.print (", ");
  Serial.print (theta3);
  Serial.print ("\n");
#endif
  
  digitalWrite(STATUS_LED, false);
  
  float tNow = millis();
 
  setServoSpeedAndPosition(&s0, tLast, tNow, theta1, last_theta1);
  setServoSpeedAndPosition(&s1, tLast, tNow, theta2, last_theta2);
  setServoSpeedAndPosition(&s2, tLast, tNow, theta3, last_theta3);
  
  last_theta1 = theta1;
  last_theta2 = theta2;
  last_theta3 = theta3;
  tLast = tNow;
  
  return 1;
}

void setServoSpeedAndPosition(StepperWrapper *stepper, float t1, float t2, float a1, float a2)
{
  if(t1 == 0)
    return;
    
  float degPerMillisec = fabs((a1-a2) / (t1-t2)); // degrees per millisec
  
  stepper->setSpeedAndPosition(degPerMillisec, a1);
  
  if(degPerMillisec > max_servo_speed) // only needed for logging?
    max_servo_speed = degPerMillisec;
    
  sum_servo_speed += degPerMillisec;
  num_servo_speed += 1.0;
}

boolean transformToServoAngle(float &theta)
{
  // Giant Sketchy's arms go from 0 (max retract) to about 140 (full extend, arm in line with servo body)  
  // Theta from the geometry maths has 0 with the arm horizontal, -90 at full extend
  
  theta = -theta;
  theta = theta + HORIZONTAL_ANGLE; // Giant Sketchy's arms are horizontal at about 50 degrees
  
  boolean success = true;
  
  if(theta < minServoPos )
  {
     success = false;
     theta = minServoPos;
  }
    
  if(theta > maxServoPos)
  {
    success = false;
    theta = maxServoPos; 
  }
    
  return success;
}

// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
// returned status: 0=OK, -1=non-existing position
int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
     float t = (f-e)*tan30/2;
     float dtr = pi/(float)180.0;
 
     theta1 *= dtr;
     theta2 *= dtr;
     theta3 *= dtr;
 
     float y1 = -(t + rf*cos(theta1));
     float z1 = -rf*sin(theta1);
 
     float y2 = (t + rf*cos(theta2))*sin30;
     float x2 = y2*tan60;
     float z2 = -rf*sin(theta2);
 
     float y3 = (t + rf*cos(theta3))*sin30;
     float x3 = -y3*tan60;
     float z3 = -rf*sin(theta3);
 
     float dnm = (y2-y1)*x3-(y3-y1)*x2;
 
     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;
     
     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
  
     // discriminant
     float d = b*b - (float)4.0*a*c;
     if (d < 0) return -1; // non-existing point
 
     z0 = -(float)0.5*(b+sqrt(d))/a;
     x0 = (a1*z0 + b1)/dnm;
     y0 = (a2*z0 + b2)/dnm;
     
     return 0;
}

// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-pane)
int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
  float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
  y0 -= 0.5 * 0.57735    * e;    // shift center to edge
  // z = a + b*y
  float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1) / (2*z0);
  float b = (y1 - y0) / z0;
  
  // discriminant
  float d = -(a + b*y1) * (a + b* y1) + rf*(b*b*rf + rf); 
  if (d < 0) return -1; // non-existing point
  
  float yj = (y1 - a*b - sqrt(d)) / (b*b + 1); // choosing outer point
  float zj = a + b*yj;
  theta = 180.0*atan(-zj / (y1 - yj)) / pi + ((yj > y1) ? 180.0 : 0.0);
  
  return 0;
}
 
// inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
// returned status: 0=OK, -1=non-existing position
int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
  theta1 = theta2 = theta3 = 0;
     
  int status = delta_calcAngleYZ(x0, y0, z0, theta1);
  if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
  if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
     
  return status;
}
