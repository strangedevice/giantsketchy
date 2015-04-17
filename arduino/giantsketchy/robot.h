// Robot parameters.

#include <stdint.h>

#define CYCLOID_RATIO 21
#define MICROSTEPS 4
#define MOTOR_STEPS 200

#define STEPS_PER_REV (MOTOR_STEPS * MICROSTEPS * CYCLOID_RATIO)

#define SAFE_LIMIT (3 * (STEPS_PER_REV / 8))

void enableX();
void enableY();
void enableZ();

void disableX();
void disableY();
void disableZ();

void setMaxSpeedX(float stepsPerSec);
void setMaxSpeedY(float stepsPerSec);
void setMaxSpeedZ(float stepsPerSec);

void moveToX(uint16_t target); 
void moveToY(uint16_t target);
void moveToZ(uint16_t target);

uint16_t getPositionX();
uint16_t getPositionY();
uint16_t getPositionZ();

bool isMotionDoneX();
bool isMotionDoneY();
bool isMotionDoneZ();

void homeX();
void homeY();
void homeZ();

bool isHomeX();
bool isHomeY();
bool isHomeZ();

