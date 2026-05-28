/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Limits.hpp
 * @brief Basic mathematical limits, clamping, conversion, and range checking.
 *
 * This header provides constexpr template functions for min/max, value
 * constraint, range checking, degree/radian conversion, and safe floating‑point
 * zero tests. All functions are defined in the `math` namespace.
 */

#pragma once

#include <float.h>
#include <math.h>
#include <stdint.h>

#ifndef MATH_PI
/**
 * @def MATH_PI
 * @brief Mathematical constant π (pi) with high precision.
 */
#define MATH_PI		3.141592653589793238462643383280
#endif

namespace math
{

/**
 * @brief Returns the smaller of two values.
 *
 * @tparam _Tp Type supporting `operator<`.
 * @param a First value.
 * @param b Second value.
 * @return The minimum of a and b.
 */
template<typename _Tp>
constexpr _Tp min(_Tp a, _Tp b)
{
	return (a < b) ? a : b;
}

/**
 * @brief Returns the smallest of three values.
 *
 * @tparam _Tp Type supporting `operator<`.
 * @param a First value.
 * @param b Second value.
 * @param c Third value.
 * @return The minimum of a, b, and c.
 */
template<typename _Tp>
constexpr _Tp min(_Tp a, _Tp b, _Tp c)
{
	return min(min(a, b), c);
}

/**
 * @brief Returns the larger of two values.
 *
 * @tparam _Tp Type supporting `operator>`.
 * @param a First value.
 * @param b Second value.
 * @return The maximum of a and b.
 */
template<typename _Tp>
constexpr _Tp max(_Tp a, _Tp b)
{
	return (a > b) ? a : b;
}

/**
 * @brief Returns the largest of three values.
 *
 * @tparam _Tp Type supporting `operator>`.
 * @param a First value.
 * @param b Second value.
 * @param c Third value.
 * @return The maximum of a, b, and c.
 */
template<typename _Tp>
constexpr _Tp max(_Tp a, _Tp b, _Tp c)
{
	return max(max(a, b), c);
}

/**
 * @brief Clamps a value between a minimum and maximum.
 *
 * If `val` is less than `min_val`, returns `min_val`. If greater than `max_val`,
 * returns `max_val`. Otherwise returns `val`.
 *
 * @tparam _Tp Type supporting comparison and copy.
 * @param val Value to constrain.
 * @param min_val Lower bound.
 * @param max_val Upper bound (must be ≥ min_val).
 * @return Constrained value in the range [min_val, max_val].
 */
template<typename _Tp>
constexpr _Tp constrain(_Tp val, _Tp min_val, _Tp max_val)
{
	return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

/**
 * @brief Constrains a float value to the range of `int16_t` and casts to `int16_t`.
 *
 * The input float is clamped to [INT16_MIN, INT16_MAX] before being cast.
 * This is useful for converting sensor or control outputs to integer types.
 *
 * @param value Floating‑point value to constrain and convert.
 * @return Clamped value as `int16_t`.
 */
constexpr int16_t constrainFloatToInt16(float value)
{
	return (int16_t)math::constrain(value, (float)INT16_MIN, (float)INT16_MAX);
}

/**
 * @brief Checks if a value lies within a closed interval.
 *
 * @tparam _Tp Type supporting comparison operators.
 * @param val Value to test.
 * @param min_val Lower bound (inclusive).
 * @param max_val Upper bound (inclusive).
 * @return true if `min_val <= val <= max_val`, false otherwise.
 */
template<typename _Tp>
constexpr bool isInRange(_Tp val, _Tp min_val, _Tp max_val)
{
	return (min_val <= val) && (val <= max_val);
}

/**
 * @brief Converts degrees to radians.
 *
 * @tparam T Floating‑point or arithmetic type.
 * @param degrees Angle in degrees.
 * @return Angle in radians.
 */
template<typename T>
constexpr T radians(T degrees)
{
	return degrees * (static_cast<T>(MATH_PI) / static_cast<T>(180));
}

/**
 * @brief Converts radians to degrees.
 *
 * @tparam T Floating‑point or arithmetic type.
 * @param radians Angle in radians.
 * @return Angle in degrees.
 */
template<typename T>
constexpr T degrees(T radians)
{
	return radians * (static_cast<T>(180) / static_cast<T>(MATH_PI));
}

/**
 * @brief Checks whether a float value is approximately zero.
 *
 * Uses `FLT_EPSILON` as the absolute tolerance.
 *
 * @param val Floating‑point value.
 * @return true if `fabsf(val - 0.0f) < FLT_EPSILON`, false otherwise.
 */
inline bool isZero(float val)
{
	return fabsf(val - 0.0f) < FLT_EPSILON;
}

/**
 * @brief Checks whether a double value is approximately zero.
 *
 * Uses `DBL_EPSILON` as the absolute tolerance.
 *
 * @param val Double‑precision value.
 * @return true if `fabs(val - 0.0) < DBL_EPSILON`, false otherwise.
 */	
inline bool isZero(double val)
{
	return fabs(val - 0.0) < DBL_EPSILON;
}

}
