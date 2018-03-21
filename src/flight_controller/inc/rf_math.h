#pragma once

#define PIf       3.14159265358f
#define HALF_PI_F 1.57079632679f
#define IPIf      0.31830988618f //inverse of Pi
#define I180      0.00555555555f
#define PI180f    0.01745329251f
#define d180PIf   57.2957795131f


// a=target variable, b=bit number to act upon 0-n
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) ((a) & (1<<(b)))

// x=target variable, y=mask
#define BITMASK_SET(x,y) ((x) |= (y))
#define BITMASK_CLEAR(x,y) ((x) &= (~(y)))
#define BITMASK_FLIP(x,y) ((x) ^= (y))
#define BITMASK_CHECK(x,y) (((x) & (y)) == (y))


#ifndef SQUARE
  #define SQUARE(x) ((x)*(x))
#endif

//These were defined in a lib file?
#ifndef MIN
  #define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
  #define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef ABS
  #define ABS(x) ((x) > 0 ? (x) : -(x))
#endif

#ifndef CONSTRAIN
  #define CONSTRAIN(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

extern uint8_t  BitReverse8(uint8_t byteToConvert);
extern uint8_t  SmCrc8(const uint8_t * ptr, uint8_t len);
extern uint8_t  CRC4_12(uint16_t crcData);
extern uint32_t BigToLittleEndian32(uint32_t numberIn);
extern float    InlineConstrainf(float amt, float low, float high);
extern uint32_t InlineConstrainui(uint32_t amt, uint32_t low, uint32_t high);
extern float    Powerf(float base, uint32_t exp);
extern float    InlineChangeRangef(float oldValue, float oldMax, float oldMin, float newMax, float newMin);
extern float    CalculateSD(float data[]);
extern float    CalculateSDSize(float data[], uint32_t size);
extern uint8_t  GetChecksum8(const uint8_t *config, uint32_t len);
extern float    InlineDegreesToRadians(float degrees);
extern float    InlineRadiansToDegrees(float radians);
extern float    Atan2fast( float y, float x );