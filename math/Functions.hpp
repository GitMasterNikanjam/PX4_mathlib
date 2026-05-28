/****************************************************************************
 *
 *   Copyright (c) 2017-2022 PX4 Development Team. All rights reserved.
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
 * @file Functions.hpp
 * @brief Collection of simple, reusable mathematical functions.
 *
 * This header provides frequently used utility functions for signal shaping,
 * interpolation, saturation, and bit operations. All functions are templated
 * or overloaded to work with common numeric and vector types.
 */

#pragma once

#include "Limits.hpp"

#if defined(__WIN32)
	// #include "../../px4_platform_common/defines.h"
	#include "../../matrix/matrix/math.hpp"
#else
	#include <px4_platform_common/defines.h>
	#include <matrix/matrix/math.hpp>
#endif

namespace math
{

/**
 * @brief Type‑safe signum function (zero treated as positive).
 *
 * Returns -1 for negative values, 0 or 1 for zero and positive values.
 *
 * @tparam T Numeric type (must support comparison with 0).
 * @param val Input value.
 * @return -1 if val < 0, otherwise 0 or 1.
 */
template<typename T>
int signNoZero(T val)
{
	return (T(0) <= val) - (val < T(0));
}

/**
 * @brief Returns +1 or -1 based on a boolean.
 *
 * @param positive If true, returns 1; otherwise returns -1.
 * @return 1 or -1.
 */
inline int signFromBool(bool positive)
{
	return positive ? 1 : -1;
}

/**
 * @brief Returns the square of a value.
 *
 * @tparam T Numeric type.
 * @param val Input value.
 * @return val * val.
 */
template<typename T>
T sq(T val)
{
	return val * val;
}

/**
 * @brief Exponential curve function – blends linear and cubic behaviour.
 *
 * Given an input in [-1, 1] and a parameter e in [0, 1], the output is
 * `(1-e)*x + e*x³`. This is commonly used for RC input shaping.
 *
 * @tparam T Floating‑point type.
 * @param value Input in range [-1, 1].
 * @param e Blending factor: 0 = pure linear, 1 = pure cubic.
 * @return Shaped output (same range as input).
 */
template<typename T>
const T expo(const T &value, const T &e)
{
	T x = constrain(value, (T) - 1, (T) 1);
	T ec = constrain(e, (T) 0, (T) 1);
	return (1 - ec) * x + ec * x * x * x;
}

/**
 * @brief SuperExpo function – further shapes the curve using a rational term.
 *
 * Output = expo(x, e) * (1-g) / (1 - |x|*g). This creates a very steep curve
 * near the ends when g is close to 1, while keeping the output in [-1, 1].
 *
 * @tparam T Floating‑point type.
 * @param value Input in range [-1, 1].
 * @param e Expo blending factor (see `expo`).
 * @param g SuperExpo factor in [0, 0.99]. 0 = pure expo, 0.99 = very aggressive.
 * @return Shaped output.
 */
template<typename T>
const T superexpo(const T &value, const T &e, const T &g)
{
	T x = constrain(value, (T) - 1, (T) 1);
	T gc = constrain(g, (T) 0, (T) 0.99);
	return expo(x, e) * (1 - gc) / (1 - fabsf(x) * gc);
}

/**
 * @brief Deadzone function – linear outside a central zero region.
 *
 * The function is continuous and piecewise linear. For input in [-dz, dz]
 * the output is zero; outside that region the output is rescaled to reach ±1.
 *
 * @verbatim
 * 1                ------
 *                /
 *             --
 *           /
 * -1 ------
 *        -1 -dz +dz 1
 * @endverbatim
 *
 * @tparam T Floating‑point type.
 * @param value Input in range [-1, 1].
 * @param dz Deadzone size (0 = no deadzone, 0.5 = half the span, <1).
 * @return Output after deadzone application.
 */
template<typename T>
const T deadzone(const T &value, const T &dz)
{
	T x = constrain(value, (T) - 1, (T) 1);
	T dzc = constrain(dz, (T) 0, (T) 0.99);
	// Rescale the input such that we get a piecewise linear function that will be continuous with applied deadzone
	T out = (x - matrix::sign(x) * dzc) / (1 - dzc);
	// apply the deadzone (values zero around the middle)
	return out * (fabsf(x) > dzc);
}

/**
 * @brief Applies deadzone then expo shaping.
 *
 * Convenience function that chains `deadzone` and `expo`.
 *
 * @param value Input in [-1, 1].
 * @param e Expo factor.
 * @param dz Deadzone size.
 * @return Shaped output.
 */
template<typename T>
const T expo_deadzone(const T &value, const T &e, const T &dz)
{
	return expo(deadzone(value, dz), e);
}

/**
 * @brief Constant‑linear‑constant interpolation with two breakpoints.
 *
 * For input ≤ x_low, output = y_low.
 * For input ≥ x_high, output = y_high.
 * Otherwise linear interpolation between (x_low, y_low) and (x_high, y_high).
 *
 * @tparam T Floating‑point type.
 * @param value Input.
 * @param x_low Lower breakpoint x‑coordinate.
 * @param x_high Upper breakpoint x‑coordinate.
 * @param y_low Output at x_low.
 * @param y_high Output at x_high.
 * @return Interpolated value.
 */
template<typename T>
const T interpolate(const T &value, const T &x_low, const T &x_high, const T &y_low, const T &y_high)
{
	if (value <= x_low) {
		return y_low;

	} else if (value > x_high) {
		return y_high;

	} else {
		/* linear function between the two points */
		T a = (y_high - y_low) / (x_high - x_low);
		T b = y_low - (a * x_low);
		return (a * value) + b;
	}
}

/*
 * Constant, piecewise linear, constant function with 1/N size intervalls and N corner points as parameters
 * y[N-1]               -------
 *                     /
 *                   /
 * y[1]            /
 *               /
 *              /
 *             /
 * y[0] -------
 *        0 1/(N-1) 2/(N-1) ... 1
 */

 /**
 * @brief Interpolation with N equally spaced breakpoints on [0, 1].
 *
 * The x‑coordinates are `i/(N-1)` for i = 0 … N-1. The output is piecewise
 * constant outside the interval [0,1] and piecewise linear inside.
 *
 * @tparam T Floating‑point type.
 * @tparam N Number of breakpoints (array size).
 * @param value Input (expected in [0,1]).
 * @param y Array of output values at the breakpoints.
 * @return Interpolated value.
 */
template<typename T, size_t N>
const T interpolateN(const T &value, const T(&y)[N])
{
	size_t index = constrain((int)(value * (N - 1)), 0, (int)(N - 2));
	return interpolate(value, (T)index / (T)(N - 1), (T)(index + 1) / (T)(N - 1), y[index], y[index + 1]);
}

/*
 * Constant, piecewise linear, constant function with N corner points as parameters
 * y[N-1]               -------
 *                     /
 *                   /
 * y[1]            /
 *               /
 *              /
 *             /
 * y[0] -------
 *          x[0] x[1] ... x[N-1]
 * Note: x[N] corner coordinates have to be sorted in ascending order
 */

/**
 * @brief Interpolation with arbitrary, sorted breakpoints.
 *
 * The arrays `x` (sorted ascending) and `y` define N breakpoints. Output is
 * constant below x[0] and above x[N-1], linear between.
 *
 * @tparam T Floating‑point type.
 * @tparam N Number of breakpoints.
 * @param value Input.
 * @param x Array of x‑coordinates (must be sorted).
 * @param y Array of corresponding output values.
 * @return Interpolated value.
 */
template<typename T, size_t N>
const T interpolateNXY(const T &value, const T(&x)[N], const T(&y)[N])
{
	size_t index = 0;

	while ((value > x[index + 1]) && (index < (N - 2))) {
		index++;
	}

	return interpolate(value, x[index], x[index + 1], y[index], y[index + 1]);
}

/*
 * Squareroot, linear function with fixed corner point at intersection (1,1)
 *                     /
 *      linear        /
 *                   /
 * 1                /
 *                /
 *      sqrt     |
 *              |
 * 0     -------
 *             0    1
 */

/**
 * @brief Square‑root followed by linear for inputs > 1.
 *
 * For input < 0 returns 0.
 * For 0 ≤ input ≤ 1 returns sqrt(input).
 * For input > 1 returns input (linear).
 *
 * @tparam T Floating‑point type.
 * @param value Input.
 * @return Shaped output.
 */
template<typename T>
const T sqrt_linear(const T &value)
{
	if (value < static_cast<T>(0)) {
		return static_cast<T>(0);

	} else if (value < static_cast<T>(1)) {
		return sqrtf(value);

	} else {
		return value;
	}
}

/**
 * @brief Linear interpolation between a and b.
 *
 * @tparam T Numeric type.
 * @param a Value at s=0.
 * @param b Value at s=1.
 * @param s Interpolation factor (any real number).
 * @return (1-s)*a + s*b.
 */
template<typename T>
const T lerp(const T &a, const T &b, const T &s)
{
	return (static_cast<T>(1) - s) * a + s * b;
}

/**
 * @brief Generic negation (default: unary minus).
 *
 * Specialised for int16_t to handle the asymmetric range (-32768 … 32767)
 * by mapping INT16_MIN to INT16_MAX and vice versa.
 *
 * @tparam T Numeric type (default implementation uses unary -).
 * @param value Value to negate.
 * @return Negated value with overflow protection for int16_t.
 */
template<typename T>
constexpr T negate(T value)
{
	static_assert(sizeof(T) > 2, "implement for T");
	return -value;
}

/**
 * @brief Specialisation for int16_t: safe negation with symmetric handling.
 *
 * Since INT16_MIN = -32768 and INT16_MAX = 32767, simple `-value` would overflow
 * for INT16_MIN. This implementation returns INT16_MAX when given INT16_MIN,
 * and vice versa.
 *
 * @param value int16_t value.
 * @return Symmetric negation.
 */
template<>
constexpr int16_t negate<int16_t>(int16_t value)
{
	if (value == INT16_MAX) {
		return INT16_MIN;

	} else if (value == INT16_MIN) {
		return INT16_MAX;
	}

	return -value;
}

/*
 * This function calculates the Hamming weight, i.e. counts the number of bits that are set
 * in a given integer.
 */

 /**
 * @brief Counts the number of set bits (Hamming weight) in an integer.
 *
 * @tparam T Unsigned integer type (or any integral type).
 * @param n Input value.
 * @return Number of 1‑bits.
 */
template<typename T>
int countSetBits(T n)
{
	int count = 0;

	while (n) {
		count += n & 1;
		n >>= 1;
	}

	return count;
}

/**
 * @brief Checks if a float is finite (not NaN or infinite).
 *
 * @param value Floating‑point value.
 * @return true if finite, false otherwise.
 */
inline bool isFinite(const float &value)
{
	return PX4_ISFINITE(value);
}

/**
 * @brief Checks if all elements of a 2D vector are finite.
 *
 * @param value Vector2f.
 * @return true if all components are finite.
 */
inline bool isFinite(const matrix::Vector2f &value)
{
	return value.isAllFinite();
}

/**
 * @brief Checks if all elements of a 3D vector are finite.
 *
 * @param value Vector3f.
 * @return true if all components are finite.
 */
inline bool isFinite(const matrix::Vector3f &value)
{
	return value.isAllFinite();
}

} /* namespace math */
