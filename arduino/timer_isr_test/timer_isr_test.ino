
// #define DEBUG

// Robot parameters.

#define CYCLOID_RATIO 21
#define MICROSTEPS 4
#define STEPS_PER_REV 200

#define SAFE_LIMIT ((MICROSTEPS * STEPS_PER_REV * CYCLOID_RATIO) / 4)

// Maximum step rate
#define INTERRUPT_FREQUENCY 4000

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

void setup() {
  
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

  // Use 8-bit timer0 to drive steppers
  
  // CTC mode: clear counter and interrupt when OCR0A is matched
  TCCR0A = (1 << WGM01);
  
  // prescaler divides clk by 64 (250 kHz tick)
  TCCR0B = (1 << CS01) | (1 << CS00);
  
  // set compare match register for frequency (value must be <256)
  OCR0A = (uint8_t)(16000000L / ((uint32_t) INTERRUPT_FREQUENCY * 64L) - 1L);
  
  TCNT0  = 0; //initialize counter value
  
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);

  sei(); // enable interrupts
  
  ENABLE_X_MOTOR(); // for now
  ENABLE_Y_MOTOR();
  ENABLE_Z_MOTOR();
  
  Serial.begin(9600);
}

volatile uint16_t xPosition = 0;
volatile uint16_t yPosition = 0;
volatile uint16_t zPosition = 0;
volatile uint16_t xTarget = 0;
volatile uint16_t yTarget = 0;
volatile uint16_t zTarget = 0;

// Table approximates a square root function (at constant acceleration, elapsed time goes as square root of distance).
// Acceleration increases towards the far end (non-linear), so that we reach max speed in a reasonable time.
#define ACCEL_STEPS 75
const uint8_t accelTab[] = {71,29,22,19,17,15,14,13,12,11,11,10,10,10,9,9,9,9,8,8,8,8,8,7,7,7,7,7,7,6,6,6,6,6,6,6,5,5,5,5,5,5,5,5,4,4,4,4,4,4,4,4,4,3,3,3,3,3,3,3,3,3,3,2,2,2,2,2,2,2,2,2,2,2,1};

// Accessed only within ISR, so no need for volatile
uint8_t xAccelStep = 0; // the acceleration step we are on, index into accelTab[]
uint8_t xTicksRemaining = 1; // number of ticks before next step is issued
uint8_t yAccelStep = 0;
uint8_t yTicksRemaining = 1; 
uint8_t zAccelStep = 0; 
uint8_t zTicksRemaining = 1;

ISR(TIMER0_COMPA_vect) {
  
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
    
    if (xPosition < xTarget) {
      SET_X_DIR();
      if (xPosition < SAFE_LIMIT) {
        ++xPosition;
        STEP_X();
      }
    } else if (xPosition > xTarget) {
      CLR_X_DIR();
      if (!AT_X_MIN()) {
        --xPosition;
        STEP_X();
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
    
    if (yPosition < yTarget) {
      SET_Y_DIR();
      if (yPosition < SAFE_LIMIT) {
        ++yPosition;
        STEP_Y();
      }
    } else if (yPosition > yTarget) {
      CLR_Y_DIR();
      if (!AT_Y_MIN()) {
        --yPosition;
        STEP_Y();
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
    
    if (zPosition < zTarget) {
      SET_Z_DIR();
      if (zPosition < SAFE_LIMIT) {
        ++zPosition;
        STEP_Z();
      }
    } else if (zPosition > zTarget) {
      CLR_Z_DIR();
      if (!AT_Z_MIN()) {
        --zPosition;
        STEP_Z();
      }
    }  
  } 
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

boolean isMotionDoneX() {
  boolean result;
  cli();
  result = (xPosition == xTarget);
  sei();
  return result;
}

boolean isMotionDoneY() {
  boolean result;
  cli();
  result = (yPosition == yTarget);
  sei();
  return result;
}

boolean isMotionDoneZ() {
  boolean result;
  cli();
  result = (zPosition == zTarget);
  sei();
  return result;
}

boolean isHomeX() {
  return AT_X_MIN();
}

boolean isHomeY() {
  return AT_Y_MIN();
}

boolean isHomeZ() {
  return AT_Z_MIN();
}

void loop() {
  
#ifdef DEBUG
  Serial.print("x: "); Serial.println(getPositionX());
  Serial.print("y: "); Serial.println(getPositionY());
  Serial.print("z: "); Serial.println(getPositionZ());
#endif
  
  if (isMotionDoneX()) {
    moveToX(xTarget == 0 ? 4000 : 0);
  }
  
  if (isMotionDoneY()) {
    moveToY(yTarget == 0 ? 3000 : 0);
  }
  
  if (isMotionDoneZ()) {
    moveToZ(zTarget == 0 ? 2000 : 0);
  }
}
