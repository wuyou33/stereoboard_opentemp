#ifndef STEREO_MATH_H
#define STEREO_MATH_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Some useful constants.  */
#ifndef M_PI
# define M_E    2.7182818284590452354 /* e */
# define M_LOG2E  1.4426950408889634074 /* log_2 e */
# define M_LOG10E 0.43429448190325182765  /* log_10 e */
# define M_LN2    0.69314718055994530942  /* log_e 2 */
# define M_LN10   2.30258509299404568402  /* log_e 10 */
# define M_PI   3.14159265358979323846  /* pi */
# define M_PI_2   1.57079632679489661923  /* pi/2 */
# define M_PI_4   0.78539816339744830962  /* pi/4 */
# define M_1_PI   0.31830988618379067154  /* 1/pi */
# define M_2_PI   0.63661977236758134308  /* 2/pi */
# define M_2_SQRTPI 1.12837916709551257390  /* 2/sqrt(pi) */
# define M_SQRT2  1.41421356237309504880  /* sqrt(2) */
# define M_SQRT1_2  0.70710678118654752440  /* 1/sqrt(2) */
#endif

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
#define BoundLower(_x, _min) { if (_x < (_min)) _x = (_min);}

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
  int8_t value_int8;
  if (value > 127) {
    value_int8 = 127;
  } else if (value < -128) {
    value_int8 = -128;
  } else {
    value_int8 = (int8_t) value;
  }

  return value_int8;
}

/* getMedian: finds maximum value in array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return median of array
 * */
static uint32_t getMedian(int32_t *a, int32_t n)
{
  // Allocate an array of the same size and sort it.
  int32_t i, j;

  int dpSorted[n];
  for (i = 0; i < n; ++i) {
    dpSorted[i] = a[i];
  }
  for (i = n - 1; i > 0; --i) {
    for (j = 0; j < i; ++j) {
      if (dpSorted[j] > dpSorted[j + 1]) {
        int32_t dTemp = dpSorted[j];
        dpSorted[j] = dpSorted[j + 1];
        dpSorted[j + 1] = dTemp;
      }
    }
  }

  // Middle or average of middle values in the sorted array.
  int32_t dMedian = 0;
  if ((n % 2) == 0) {
    dMedian = (dpSorted[n / 2] + dpSorted[(n / 2) - 1]) / 2.0;
  } else {
    dMedian = dpSorted[n / 2];
  }
  return dMedian;
}
/* getMean: calculate mean of array
 * \param a is an array containing the values
 * \param n is size of the array
 * \return mean of array
 * */
static uint32_t getMean(int32_t *a, int32_t n)
{
  int32_t dSum = a[0];
  int32_t i;
  for (i = 1; i < n; ++i) {
    dSum += a[i];
  }
  return dSum / n;
}

/* ipow: calculates the power of the value
 * \param base
 * \param exp (the power of the base)
 * \return result
 *
 * */
static int ipow(int base, int exp)
{
  int result = 1;
  while (exp) {
    if (exp & 1) {
      result *= base;
    }
    exp >>= 1;
    base *= base;
  }

  return result;
}

static float get_random_number_f(void)
{
  int rand_num = rand() % 1000;
  float r = (float)rand_num / 1000.0f;
  return r;
}

static float get_minimum_f(float *nums, uint16_t n_elements, uint16_t *index)
{
  (*index) = 0;
  float min = nums[0];
  uint16_t i;
  for (i = 1; i < n_elements; i++) {
    if (nums[i] < min) {
      (*index) = i;
      min = nums[i];
    }
  }
  return min;
}

static uint32_t get_sum(uint8_t *nums, uint16_t n_elements)
{
  uint32_t sum = nums[0];
  uint16_t i;
  for (i = 1; i < n_elements; i++) {
    sum += nums[i];
  }
  return sum;
}

#endif
