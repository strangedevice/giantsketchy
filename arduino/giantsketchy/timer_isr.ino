
#include "robot.h"

// #define DEBUG

// Maximum step rate
#define INTERRUPT_FREQUENCY 8000

// For RAMPS 1.4, mapped to ATMeta2560
// See ./hardware/arduino/variants/mega/pins_arduino.h

/*****************************************************
#define X_STEP_PIN         54 // A0 = PF0
#define X_DIR_PIN          55 // A1 = PF1
#define X_ENABLE_PIN       38 // D38 = PD7
#define X_MIN_PIN           3 // PWM3 = PE5 
#define X_MAX_PIN           2 // PWM2 = PE4

#define Y_STEP_PIN         60 // A6 = PF6
#define Y_DIR_PIN          61 // A7 = PF7
#define Y_ENABLE_PIN       56 // A2 = PF2
#define Y_MIN_PIN          14 // USART3_TX = PJ1
#define Y_MAX_PIN          15 // USART3_RX = PJ0

#define Z_STEP_PIN         46 // D46 = PL3
#define Z_DIR_PIN          48 // D48 = PL1
#define Z_ENABLE_PIN       62 // A8 = PK0
#define Z_MIN_PIN          18 // USART1_TX = PD3
#define Z_MAX_PIN          19 // USART1_RX = PD2
*******************************************************/

#define SET_X_STEP() PORTF |= _BV(PF0)
#define CLR_X_STEP() PORTF &= ~_BV(PF0)
#define SET_Y_STEP() PORTF |= _BV(PF6)
#define CLR_Y_STEP() PORTF &= ~_BV(PF6)
#define SET_Z_STEP() PORTL |= _BV(PL3)
#define CLR_Z_STEP() PORTL &= ~_BV(PL3)

#define STEP_X() do { SET_X_STEP(); CLR_X_STEP(); } while (0)
#define STEP_Y() do { SET_Y_STEP(); CLR_Y_STEP(); } while (0)
#define STEP_Z() do { SET_Z_STEP(); CLR_Z_STEP(); } while (0)

#define SET_X_DIR() PORTF |= _BV(PF1)
#define CLR_X_DIR() PORTF &= ~_BV(PF1)
#define SET_Y_DIR() PORTF |= _BV(PF7)
#define CLR_Y_DIR() PORTF &= ~_BV(PF7)
#define SET_Z_DIR() PORTL |= _BV(PL1)
#define CLR_Z_DIR() PORTL &= ~_BV(PL1)

#define FORWARD_X() CLR_X_DIR()
#define FORWARD_Y() CLR_Y_DIR()
#define FORWARD_Z() CLR_Z_DIR()

#define BACKWARD_X() SET_X_DIR()
#define BACKWARD_Y() SET_Y_DIR()
#define BACKWARD_Z() SET_Z_DIR()

// Enables are active low on Pololu
#define ENABLE_X_MOTOR() PORTD &= ~_BV(PD7)
#define DISABLE_X_MOTOR() PORTD |= _BV(PD7)
#define ENABLE_Y_MOTOR() PORTF &= ~_BV(PF2)
#define DISABLE_Y_MOTOR() PORTF |= _BV(PF2)
#define ENABLE_Z_MOTOR() PORTK &= ~_BV(PK0)
#define DISABLE_Z_MOTOR() PORTK |= _BV(PK0)

// Switches normally closed with pullup, so high when switch pressed
#define AT_X_MIN() ((PINE & _BV(PE5)) != 0)
#define AT_Y_MIN() ((PINJ & _BV(PJ1)) != 0)
#define AT_Z_MIN() ((PIND & _BV(PD3)) != 0)

// Position bit that is written to the STEP output
// Bit 0 steps at half the timer frequency, etc.
#define STEP_BIT (1 << 0)

// Call setup only once...
void setupTimerISR() {
  
  // Set step, direction and enable pins as output for 3 axes (9 total)
  DDRF |= _BV(PF0) | _BV(PF1) | _BV(PF6) | _BV(PF7) | _BV(PF2);
  DDRD |= _BV(PD7);
  DDRL |= _BV(PL3) | _BV(PL1);
  DDRK |= _BV(PK0);
  
  // Set min and max pins as inputs with pullup for 3 axes (6 total)
  DDRE &= ~(_BV(PE5) | _BV(PE4)); PORTE |= _BV(PE5) | _BV(PE4);
  DDRJ &= ~(_BV(PJ1) | _BV(PJ0)); PORTJ |= _BV(PJ1) | _BV(PJ0);
  DDRD &= ~(_BV(PD3) | _BV(PD2)); PORTD |= _BV(PD3) | _BV(PD2);
  
  cli(); // disable interrupts

  // Use 8-bit timer2 to drive steppers (timer 0 is used by millis())
  
  // Enable power to timer2
  PRR0 &= ~(1 << PRTIM2);
  
  // CTC mode: clear counter and interrupt when OCR0A is matched
  TCCR2A = (1 << WGM21);
  
  // prescaler divides clk by 64 (250 kHz tick) (timer0 differs here)
  TCCR2B = (1 << CS22);
  
  // set compare match register for frequency (value must be <256)
  OCR2A = (uint8_t)(16000000L / ((uint32_t) INTERRUPT_FREQUENCY * 64L) - 1L);
  
  TCNT2 = 0; //initialize counter value
  
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei(); // enable interrupts
}

volatile uint16_t xPosition = SAFE_LIMIT;
volatile uint16_t yPosition = SAFE_LIMIT;
volatile uint16_t zPosition = SAFE_LIMIT;
volatile uint16_t xTarget = xPosition;
volatile uint16_t yTarget = yPosition;
volatile uint16_t zTarget = zPosition;

// Table approximates a square root function (at constant acceleration, elapsed time goes as square root of distance).
// Acceleration increases towards the far end (non-linear), so that we reach max speed in a reasonable time.
#define ACCEL_STEPS 75
const uint8_t accelTab[] = {71,29,22,19,17,15,14,13,12,11,11,10,10,10,9,9,9,9,8,8,8,8,8,7,7,7,7,7,7,6,6,6,6,6,6,6,5,5,5,5,5,5,5,5,4,4,4,4,4,4,4,4,4,3,3,3,3,3,3,3,3,3,3,2,2,2,2,2,2,2,2,2,2,2,1};

// Accessed only within ISR, so no need for volatile
uint8_t xAccelStep = 0; // the acceleration step we are on, index into accelTab[]
uint8_t xTicksRemaining = 1; // number of ticks before next step is issued
uint8_t xTicksMin = 1; // minimum ticks before next step, use to set maximum speed
uint8_t yAccelStep = 0;
uint8_t yTicksRemaining = 1; 
uint8_t yTicksMin = 1;
uint8_t zTicksMin = 1;
uint8_t zAccelStep = 0; 
uint8_t zTicksRemaining = 1;

ISR(TIMER2_COMPA_vect) {
  
  if (--xTicksRemaining == 0) {
    uint16_t stepsToGo = xPosition > xTarget ? xPosition - xTarget : xTarget - xPosition;
    
    if (stepsToGo > xAccelStep) {
      // We still have enough room to decelerate, so continue to accelerate.
      if (xAccelStep < ACCEL_STEPS - 1) ++xAccelStep;
    } else {
      // Decelerate. Set the accelStep from stepsToGo (rather than decrementing), because
      // the target may have been changed.
      xAccelStep = stepsToGo > 0 ? stepsToGo - 1 : 0;
    }
    
    xTicksRemaining = accelTab[xAccelStep];
    if (xTicksRemaining < xTicksMin) {
      xTicksRemaining = xTicksMin; // respect the speed limit
    }
    
    if (xPosition < xTarget) {
      FORWARD_X();
      if (xPosition < SAFE_LIMIT) {
        ++xPosition;
        STEP_X();
      }
    } else if (xPosition > xTarget) {
      BACKWARD_X();
      if (!AT_X_MIN()) {
        --xPosition;
        STEP_X();
      } else { // we hit the home switch, so stop
        xPosition = 0;
        xTarget = 0;
      }
    }  
  }
  
  if (--yTicksRemaining == 0) {
    uint16_t stepsToGo = yPosition > yTarget ? yPosition - yTarget : yTarget - yPosition;
    
    if (stepsToGo > yAccelStep) {
      if (yAccelStep < ACCEL_STEPS - 1) ++yAccelStep;
    } else {
      yAccelStep = stepsToGo > 0 ? stepsToGo - 1 : 0;
    }
    
    yTicksRemaining = accelTab[yAccelStep];
    if (yTicksRemaining < yTicksMin) {
      yTicksRemaining = yTicksMin;
    }
    
    if (yPosition < yTarget) {
      FORWARD_Y();
      if (yPosition < SAFE_LIMIT) {
        ++yPosition;
        STEP_Y();
      }
    } else if (yPosition > yTarget) {
      BACKWARD_Y();
      if (!AT_Y_MIN()) {
        --yPosition;
        STEP_Y();
      } else {
        yPosition = 0;
        yTarget = 0;
      }
    }  
  }
  
  if (--zTicksRemaining == 0) {
    uint16_t stepsToGo = zPosition > zTarget ? zPosition - zTarget : zTarget - zPosition;
    
    if (stepsToGo > zAccelStep) {
      if (zAccelStep < ACCEL_STEPS - 1) ++zAccelStep;
    } else {
      zAccelStep = stepsToGo > 0 ? stepsToGo - 1 : 0;
    }
    
    zTicksRemaining = accelTab[zAccelStep];
    if (zTicksRemaining < zTicksMin) {
      zTicksRemaining = zTicksMin;
    }
    
    if (zPosition < zTarget) {
      FORWARD_Z();
      if (zPosition < SAFE_LIMIT) {
        ++zPosition;
        STEP_Z();
      }
    } else if (zPosition > zTarget) {
      BACKWARD_Z();
      if (!AT_Z_MIN()) {
        --zPosition;
        STEP_Z();
      } else {
        zPosition = 0;
        zTarget = 0;
      }
    }  
  } 
}

// Public API below here
// ---------------------

void enableX() {
  ENABLE_X_MOTOR();
}

void enableY() {
  ENABLE_Y_MOTOR();
}

void enableZ() {
  ENABLE_Z_MOTOR(); 
}

void disableX() {
  DISABLE_X_MOTOR();
}

void disableY() {
  DISABLE_Y_MOTOR();
}

void disableZ() {
  DISABLE_Z_MOTOR(); 
}

// The maximum speed available is INTERRUPT_FREQUENCY

void setMaxSpeedX(float stepsPerSec) {
  xTicksMin = ((float)INTERRUPT_FREQUENCY / stepsPerSec) + 0.5; // NB: truncation
}

void setMaxSpeedY(float stepsPerSec) {
  yTicksMin = ((float)INTERRUPT_FREQUENCY / stepsPerSec) + 0.5;
}

void setMaxSpeedZ(float stepsPerSec) {
  zTicksMin = ((float)INTERRUPT_FREQUENCY / stepsPerSec) + 0.5;
}

// Note: target updates that cause an instant change of direction
// will not accelerate or decelerate. Wait for motion to finish before doing this.

void moveToX(uint16_t target) {
  if (target > SAFE_LIMIT) target = SAFE_LIMIT;
  cli(); // ensure update is atomic
  xTarget = target;
  sei();
}

void moveToY(uint16_t target) {
  if (target > SAFE_LIMIT) target = SAFE_LIMIT;
  cli();
  yTarget = target;
  sei();
}

void moveToZ(uint16_t target) {
  if (target > SAFE_LIMIT) target = SAFE_LIMIT;
  cli();
  zTarget = target;
  sei();
}

uint16_t getPositionX() {
  uint16_t result;
  cli();
  result = xPosition; // atomic access
  sei();
  return result;
}

uint16_t getPositionY() {
  uint16_t result;
  cli();
  result = yPosition;
  sei();
  return result;
}

uint16_t getPositionZ() {
  uint16_t result;
  cli();
  result = zPosition;
  sei();
  return result;
}

bool isMotionDoneX() {
  boolean result;
  cli();
  result = (xPosition == xTarget);
  sei();
  return result;
}

bool isMotionDoneY() {
  boolean result;
  cli();
  result = (yPosition == yTarget);
  sei();
  return result;
}

bool isMotionDoneZ() {
  boolean result;
  cli();
  result = (zPosition == zTarget);
  sei();
  return result;
}

void homeX() {
  setMaxSpeedX(INTERRUPT_FREQUENCY / 8); // go slowly
  cli();
  xPosition = SAFE_LIMIT;
  xTarget = 0; // motion stops automatically at the limit switch
  sei();
}

void homeY() {
  setMaxSpeedY(INTERRUPT_FREQUENCY / 8); // go slowly
  cli();
  yPosition = SAFE_LIMIT;
  yTarget = 0;
  sei();
}

void homeZ() {
  setMaxSpeedZ(INTERRUPT_FREQUENCY / 8); // go slowly
  cli();
  zPosition = SAFE_LIMIT;
  zTarget = 0;
  sei();
}

bool isHomeX() {
  return AT_X_MIN();
}

bool isHomeY() {
  return AT_Y_MIN();
}

bool isHomeZ() {
  return AT_Z_MIN();
}

