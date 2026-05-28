/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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
 * @file WelfordMean.hpp
 * @brief Online mean and variance estimation using Welford's algorithm.
 *
 * This header provides a class template `WelfordMean` that incrementally updates
 * the mean, variance, and standard deviation of a data stream. It uses Welford's
 * numerically stable one‑pass algorithm and Kahan summation to minimise floating‑point
 * error. The implementation also handles count overflow and invalid values gracefully.
 *
 * @see https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
 */

#pragma once

#include <lib/mathlib/mathlib.h>

namespace math
{
/**
 * @brief Online mean and variance estimator using Welford's algorithm.
 *
 * This class maintains the running mean and variance of a sequence of values.
 * It supports incremental updates and provides access to the current mean,
 * variance, and standard deviation. The algorithm is numerically stable and
 * uses Kahan summation to reduce floating‑point drift.
 *
 * @tparam Type Floating‑point type (default `float`). Must support arithmetic
 *         operations and be compatible with `math::max` and `PX4_ISFINITE`.
 */
template <typename Type = float>
class WelfordMean
{
public:
	/**
     * @brief Updates the statistics with a new sample.
     *
     * Incorporates a new value into the running mean and variance.
     * - On first sample, initialises the estimator.
     * - On count overflow (`UINT16_MAX`), preserves the current mean and variance
     *   but resets the count to 1 to avoid overflow.
     * - Uses Kahan summation for both mean and M2 updates.
     *
     * @param new_value The new data sample.
     * @return `true` if the update succeeded and the statistics are valid
     *         (at least 3 samples accumulated), `false` otherwise.
     *
     * @note After overflow reset, the count restarts from 1, which may
     *       temporarily make `valid()` return `false` until enough samples arrive.
     */
	bool update(const Type &new_value)
	{
		if (_count == 0) {
			reset();
			_count = 1;
			_mean = new_value;
			return false;

		} else if (_count == UINT16_MAX) {
			// count overflow
			//  reset count, but maintain mean and variance
			_M2 = _M2 / _count;
			_M2_accum = 0;

			_count = 1;

		} else {
			_count++;
		}

		// mean accumulates the mean of the entire dataset
		// delta can be very small compared to the mean, use algorithm to minimise numerical error
		const Type delta{new_value - _mean};
		const Type mean_change = delta / _count;
		_mean = kahanSummation(_mean, mean_change, _mean_accum);

		// M2 aggregates the squared distance from the mean
		// count aggregates the number of samples seen so far
		const Type M2_change = delta * (new_value - _mean);
		_M2 = kahanSummation(_M2, M2_change, _M2_accum);

		// protect against floating point precision causing negative variances
		_M2 = math::max(_M2, 0.f);

		if (!PX4_ISFINITE(_mean) || !PX4_ISFINITE(_M2)) {
			reset();
			return false;
		}

		return valid();
	}

	/**
     * @brief Checks whether the current statistics are valid.
     *
     * Statistics are considered valid after at least 3 samples have been accumulated.
     * This ensures that the sample variance (`_M2 / (count-1)`) is defined.
     *
     * @return `true` if the sample count is greater than 2, otherwise `false`.
     */
	bool valid() const { return _count > 2; }

	/**
     * @brief Returns the number of samples processed so far.
     *
     * @return Sample count (may be reset to 1 after overflow).
     */	
	auto count() const { return _count; }

	/**
     * @brief Resets the estimator to its initial state.
     *
     * Clears all accumulated statistics and sets the count to zero.
     */
	void reset()
	{
		_count = 0;
		_mean = 0;
		_M2 = 0;

		_mean_accum = 0;
		_M2_accum = 0;
	}

	/**
     * @brief Returns the current sample mean.
     *
     * @return Running mean of all samples seen so far.
     */
	Type mean() const { return _mean; }

	/**
     * @brief Returns the current sample variance (unbiased estimate).
     *
     * Variance is computed as `M2 / (count - 1)`. If fewer than 2 samples have been
     * processed, the result is undefined (may be zero or NaN). Use `valid()` to check.
     *
     * @return Sample variance.
     */
	Type variance() const { return _M2 / (_count - 1); }

	/**
     * @brief Returns the current sample standard deviation.
     *
     * Standard deviation is the square root of the variance.
     *
     * @return Sample standard deviation.
     */
	Type standard_deviation() const { return std::sqrt(variance()); }

private:

	/**
     * @brief Kahan summation helper to reduce floating‑point error.
     *
     * Adds `input` to `sum_previous` while keeping a running compensation
     * in `accumulator`. The caller must preserve `accumulator` between calls.
     *
     * @param sum_previous The accumulated sum before adding `input`.
     * @param input The value to add.
     * @param accumulator The Kahan compensation variable (modified).
     * @return The corrected sum (`sum_previous + input` with reduced error).
     */
	inline Type kahanSummation(Type sum_previous, Type input, Type &accumulator)
	{
		const Type y = input - accumulator;
		const Type t = sum_previous + y;
		accumulator = (t - sum_previous) - y;
		return t;
	}

	Type _mean{};			///< Current mean estimate.
	Type _M2{};				///< Sum of squared differences from the mean (scaled by count-1 when used for variance).

	Type _mean_accum{};  	///< kahan summation algorithm accumulator for mean
	Type _M2_accum{};    	///< kahan summation algorithm accumulator for M2

	uint16_t _count{0};		///< Number of samples processed (resets to 1 after overflow).
};

} // namespace math
