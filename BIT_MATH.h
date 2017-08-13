/*
 * AP_MATH.h
 *
 *  Created on: 2017-8-1
 *      Author: wangbo
 */

#ifndef AP_MATH_H_
#define AP_MATH_H_

#include <cmath>
#include <limits>
#include <stdint.h>
//#include <type_traits>

#include "vector2.h"
#include "vector3.h"
#include "matrix3.h"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)
#endif

#define M_2PI         (M_PI * 2)

/*
 * A variant of asin() that checks the input ranges and ensures a valid angle
 * as output. If nan is given as input then zero is returned.
 */
template <typename T>
float safe_asin(const T v);

/*
 * A variant of sqrt() that checks the input ranges and ensures a valid value
 * as output. If a negative number is given then 0 is returned.  The reasoning
 * is that a negative number for sqrt() in our code is usually caused by small
 * numerical rounding errors, so the real input should have been zero
 */
template <typename T>
float safe_sqrt(const T v);

// invOut is an inverted 4x4 matrix when returns true, otherwise matrix is Singular
bool inverse3x3(float m[], float invOut[]);

// invOut is an inverted 3x3 matrix when returns true, otherwise matrix is Singular
bool inverse4x4(float m[],float invOut[]);

// matrix multiplication of two NxN matrices
float *mat_mul(float *A, float *B, uint8_t n);

// matrix algebra
bool inverse(float x[], float y[], uint16_t dim);

/*
 * Constrain an angle to be within the range: -180 to 180 degrees. The second
 * parameter changes the units. Default: 1 == degrees, 10 == dezi,
 * 100 == centi.
 */
template <typename T>
float wrap_180(const T angle, float unit_mod = 1);

/*
 * Wrap an angle in centi-degrees. See wrap_180().
 */
//template <typename T>
//auto wrap_180_cd(const T angle) -> decltype(wrap_180(angle, 100.f));

/*
 * Constrain an euler angle to be within the range: 0 to 360 degrees. The
 * second parameter changes the units. Default: 1 == degrees, 10 == dezi,
 * 100 == centi.
 */
template <typename T>
float wrap_360(const T angle, float unit_mod = 1);

/*
 * Wrap an angle in centi-degrees. See wrap_360().
 */
//template <typename T>
//auto wrap_360_cd(const T angle) -> decltype(wrap_360(angle, 100.f));

/*
  wrap an angle in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
template <typename T>
float wrap_PI(const T radian);

/*
 * wrap an angle in radians to 0..2PI
 */
template <typename T>
float wrap_2PI(const T radian);

/*
 * Constrain a value to be within the range: low and high
 */
template <typename T>
T constrain_value(const T amt, const T low, const T high);

inline float constrain_float(const float amt, const float low, const float high)
{
    return constrain_value(amt, low, high);
}

inline int16_t constrain_int16(const int16_t amt, const int16_t low, const int16_t high)
{
    return constrain_value(amt, low, high);
}

inline int32_t constrain_int32(const int32_t amt, const int32_t low, const int32_t high)
{
    return constrain_value(amt, low, high);
}

// degrees -> radians
static inline float radians(float deg)
{
    return deg * DEG_TO_RAD;
}

// radians -> degrees
static inline float degrees(float rad)
{
    return rad * RAD_TO_DEG;
}

template<typename T>
float sq(const T val)
{
    return powf(static_cast<float>(val), 2);
}

/*
 * Variadic template for calculating the square norm of a vector of any
 * dimension.
 */
template<typename T, typename... Params>
float sq(const T first, const Params... parameters)
{
    return sq(first) + sq(parameters...);
}

/*
 * Variadic template for calculating the norm (pythagoras) of a vector of any
 * dimension.
 */
//template<typename T, typename U, typename... Params>
//float norm(const T first, const U second, const Params... parameters)
//{
  //  return sqrtf(sq(first, second, parameters...));
//}

//template<typename A, typename B>
//static inline auto MIN(const A &one, const B &two) -> decltype(one < two ? one : two)
//{
  //  return one < two ? one : two;
//}
#if 0
template<typename A, typename B>
static inline auto MAX(const A &one, const B &two) -> decltype(one > two ? one : two)
{
    return one > two ? one : two;
}
#endif

/*
  linear interpolation based on a variable in a range
 */
float linear_interpolate(float low_output, float high_output,
                         float var_value,
                         float var_low, float var_high);

/* simple 16 bit random number generator */
uint16_t get_random16(void);

// generate a random float between -1 and 1, for use in SITL
float rand_float(void);

// generate a random Vector3f of size 1
Vector3f rand_vec3f(void);

#endif /* AP_MATH_H_ */
