
boolean bluetoothOn = false;

#define BT_STATUS_WAITING 0
#define BT_STATUS_NUMPOINTS 1
#define BT_STATUS_READING 2
#define BT_STATUS_DONE 3

int btStatus = BT_STATUS_WAITING;
unsigned int numPointsComing = 0;
int axis = 0;
int axes[3];
long startT = 0;

void setupBluetooth()
{
  // Nothing to do.
}

void turnOnBluetooth()
{ 
  if (bluetoothOn)
    return;

#ifdef DO_LOGGING
  Serial.println("turnOnBluetooth");
#endif
  
  Serial2.begin(9600);
      
  startT = 0;  
  bluetoothOn = true;
  btStatus = BT_STATUS_WAITING;
}

void turnOffBluetooth()
{
  if (!bluetoothOn)
    return;
    
  Serial2.end();
  bluetoothOn = false;

#ifdef DO_LOGGING
  Serial.println("turnOffBluetooth");
#endif
}


void loopBluetooth()
{
  if (!bluetoothOn)
    return;
  
  if(btStatus == BT_STATUS_READING)
  {
    long elapsed = startT - millis();
    
    if (elapsed > 5000)
    {
#ifdef DO_LOGGING
      Serial.print("Timeout - numPointsComing: ");    
      Serial.print(numPointsComing);
      Serial.print(", numPoints: ");
      Serial.print(numPoints);
      Serial.print("\n");
#endif
      stopReading();
      return;
    }
  } 
 
  while (Serial2.available()) 
  {
    unsigned char c = (char)Serial2.read();
          
    switch( btStatus )
    {
      case BT_STATUS_WAITING:
        numPointsComing = c & 0xFF; // least significant byte of num points
#ifdef DO_LOGGING
        Serial.print("LS Byte: ");
        Serial.print((byte)c); 
#endif
        // clearPath();
        btStatus = BT_STATUS_NUMPOINTS;
        startT = millis();           
        break;
        
      case BT_STATUS_NUMPOINTS:
        numPointsComing |= (c << 8); // most significant byte of num points
#ifdef DO_LOGGING
        Serial.print(", MS Byte: ");
        Serial.println((byte)c);
        Serial.print("Expected number of points: ");
        Serial.println(numPointsComing);   
#endif
        btStatus = BT_STATUS_READING;
        break;
         
        case BT_STATUS_READING:
          axes[axis++] = c;
          
          if( axis == 3 )
          {
            int z = axes[2] - 127;
            
            if( z < -20 || z > 20 )
              numPointsComing = 1; // abort on bad z             
            else if (!addPoint(axes[0] - 127, axes[1] - 127, z, 2))
              numPointsComing = 1; // abort
              
            axis = 0;
            numPointsComing --;
            
            if( numPointsComing == 0 )
             {
               stopReading();
             }  
          }
          break;
         
        case BT_STATUS_DONE:
          break;         
     }
  }

/* 
   #ifdef DO_LOGGING
    Serial.print ("numPointsComing ");  
    Serial.print (numPointsComing);
    Serial.print ("\n");
     #endif
*/
}

boolean isDoneBluetooth() {
  return (btStatus == BT_STATUS_DONE);
}

void stopReading()
{
   addPoint(0.0, 0.0, -10.0, 1); // park pen
  
   #ifdef DO_LOGGING
      Serial.print ("stopReading numPointsComing: ");    
      Serial.print (numPointsComing);
      Serial.print (" numPoints: ");
      Serial.print (numPoints);
      Serial.print ("\n");
       #endif
       
   btStatus = BT_STATUS_DONE;
  
   // startRun();
}

/*
void echo()
{
  while (Serial2.available()) 
  {
    unsigned char c = (char)mySerial.read();
    Serial.print (c);
  }
}
*/
