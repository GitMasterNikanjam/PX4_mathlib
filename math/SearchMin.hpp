/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file SearchMin.hpp
 * @brief One‑dimensional minimization algorithms (golden section search).
 *
 * This header provides a type‑generic golden section search for finding
 * the minimum of a unimodal function on an interval. A binary search
 * routine is planned but not yet implemented.
 */

#pragma once

namespace math
{
/**
 * @brief Golden ratio constant φ = (1 + √5)/2 ≈ 1.6180339887.
 *
 * Used in the golden section search to optimally reduce the bracketing interval.
 */
static constexpr double GOLDEN_RATIO = 1.6180339887; //(sqrt(5)+1)/2

/**
 * @brief Type‑safe absolute value function.
 *
 * Works for any numeric type that supports comparison with zero and unary minus.
 *
 * @tparam _Tp Numeric type (e.g., float, double, int).
 * @param val Input value.
 * @return Absolute value of val.
 */
template<typename _Tp>
_Tp abs_t(_Tp val)
{
	return ((val > (_Tp)0) ? val : -val);
}

/**
 * @brief Golden section search for the minimum of a unimodal function.
 *
 * Given a function `fun` that is unimodal (has a single minimum) on the interval
 * [arg1, arg2], this routine iteratively narrows the interval using the golden
 * ratio until the width is less than `tol`. It returns the midpoint of the final
 * interval as an approximation of the minimizer.
 *
 * The method evaluates the function at two interior points, `c` and `d`, and
 * discards the sub‑interval that does not contain the minimum based on the
 * function values. The golden ratio ensures optimal reduction of the interval
 * per function evaluation.
 *
 * @tparam _Tp Floating‑point type (should support arithmetic and comparison).
 * @param arg1 Left endpoint of the initial search interval.
 * @param arg2 Right endpoint of the initial search interval (must be > arg1).
 * @param fun Pointer to the function to minimize (signature: `_Tp fun(_Tp)`).
 * @param tol Absolute tolerance for the interval width (must be positive).
 *
 * @return Approximation of the point where `fun` attains its minimum on [arg1, arg2].
 *
 * @note The function must be unimodal (strictly decreasing then increasing) on the interval.
 * @note The tolerance should be chosen relative to the expected scale of x and desired precision.
 * @note For functions that are flat near the minimum, the algorithm may converge more slowly.
 *
 * @see https://en.wikipedia.org/wiki/Golden-section_search
 */
template<typename _Tp>
inline const _Tp goldensection(const _Tp &arg1, const _Tp &arg2, _Tp(*fun)(_Tp), const _Tp &tol)
{
	_Tp a = arg1;
	_Tp b = arg2;
	_Tp c = b - (b - a) / GOLDEN_RATIO;
	_Tp d = a + (b - a) / GOLDEN_RATIO;

	while (abs_t(c - d) > tol) {

		if (fun(c) < fun(d)) {
			b = d;

		} else {
			a = c;
		}

		c = b - (b - a) / GOLDEN_RATIO;
		d = a + (b - a) / GOLDEN_RATIO;

	}

	return ((b + a) / (_Tp)2);
}
}
