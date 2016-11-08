#ifndef STEREO_MATH_H
#define STEREO_MATH_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define NormRadAngle(x) { \
    while (x > M_PI) x -= 2 * M_PI; \
    while (x < -M_PI) x += 2 * M_PI; \
  }
#define DegOfRad(x) ((x) * (180. / M_PI))
#define DeciDegOfRad(x) ((x) * (1800./ M_PI))
#define RadOfDeg(x) ((x) * (M_PI/180.))
#define RadOfDeciDeg(x) ((x) * (M_PI/1800.))

#define MOfCm(_x) (((float)(_x))/100.)
#define MOfMm(_x) (((float)(_x))/1000.)

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

#ifndef ABS
#define ABS(val) ((val) < 0 ? -(val) : (val))
#endif

#define BoundUpper(_x, _max) { if (_x > (_max)) _x = (_max);}


#define Bound(_x, _min, _max) { if (_x > (_max)) _x = (_max); else if (_x < (_min)) _x = (_min); }
#define BoundInverted(_x, _min, _max) {           \
    if ((_x < (_min)) && (_x > (_max))) {         \
      if (abs(_x - (_min)) < abs(_x - (_max)))    \
        _x = (_min);                              \
      else                                        \
        _x = (_max);                              \
    }                                             \
  }
#define BoundWrapped(_x, _min, _max) {            \
    if ((_max) > (_min))                          \
      Bound(_x, _min, _max)                       \
      else                                        \
        BoundInverted(_x, _min, _max)             \
      }
#define BoundAbs(_x, _max) Bound(_x, -(_max), (_max))
#define Chop(_x, _min, _max) ( (_x) < (_min) ? (_min) : (_x) > (_max) ? (_max) : (_x) )
#define ChopAbs(x, max) Chop(x, -(max), (max))

#define DeadBand(_x, _v) {            \
    if (_x > (_v))                    \
      _x = _x -(_v);                  \
    else if  (_x < -(_v))             \
      _x = _x +(_v);                  \
    else                              \
      _x = 0;                         \
  }

#define Blend(a, b, rho) (((rho)*(a))+(1-(rho))*(b))

/* bounduint8: bound value outside of bounds of uint8, to prevent overflow
 * \param value to be bounded
 * \return bounded value to uint8_t
 * */
static uint8_t bounduint8(int32_t value)
{
  uint8_t value_uint8;
  if (value > 255) {
    value_uint8 = 255;
  } else if (value < 0) {
    value_uint8 = 0;
  } else {
    value_uint8 = (uint8_t) value;
  }

  return value_uint8;
}

static int8_t boundint8(int32_t value)
{
  int8_t value_uint8;
  if (value > 127) {
    value_uint8 = 127;
  } else if (value < -128) {
    value_uint8 = -128;
  } else {
    value_uint8 = (uint8_t) value;
  }

  return value_uint8;
}

#endif
