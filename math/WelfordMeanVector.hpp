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
 * @file WelfordMeanVector.hpp
 * @brief Online mean and covariance estimation for vectors using Welford's algorithm.
 *
 * This header provides a class template `WelfordMeanVector` that incrementally
 * updates the mean vector and covariance matrix of a stream of vector samples.
 * It extends the scalar Welford algorithm to multiple dimensions, using Kahan
 * summation for numerical stability and handling count overflow gracefully.
 *
 * @see https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
 */

#pragma once

namespace math
{
/**
 * @brief Online mean and covariance estimator for fixed‑size vectors.
 *
 * Maintains running mean and covariance of vector samples (e.g., `Vector<float, 3>`)
 * using Welford's algorithm. The implementation:
 * - Uses Kahan summation to reduce floating‑point error.
 * - Stores only the upper triangle of the covariance matrix internally,
 *   then makes it symmetric for querying.
 * - Handles count overflow by scaling the accumulated M2 matrix and resetting
 *   the count while preserving the estimates.
 * - Checks for finite values and resets on invalid data.
 *
 * @tparam Type Floating‑point type (e.g., `float`, `double`).
 * @tparam N Dimension of the vector (size of the state).
 */
template <typename Type, size_t N>
class WelfordMeanVector
{
public:
	/**
     * @brief Updates the statistics with a new vector sample.
     *
     * Incorporates a new sample into the running mean and covariance.
     * - On first sample, initialises the estimator.
     * - On count overflow (`UINT16_MAX`), preserves the current mean and
     *   covariance but resets the count to 1 to avoid overflow.
     * - Uses Kahan summation for both mean and covariance updates.
     *
     * @param new_value The new sample vector (e.g., `matrix::Vector<Type, N>`).
     * @return `true` if the update succeeded and the statistics are valid
     *         (at least 3 samples accumulated), `false` otherwise.
     *
     * @note After overflow reset, the count restarts from 1, which may
     *       temporarily make `valid()` return `false` until enough samples arrive.
     */
	bool update(const matrix::Vector<Type, N> &new_value)
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
			_M2_accum.zero();

			_count = 1;

		} else {
			_count++;
		}

		// mean
        //  accumulates the mean of the entire dataset
        //  use Kahan summation because delta can be very small compared to the mean
		const matrix::Vector<Type, N> delta{new_value - _mean};
		{
			const matrix::Vector<Type, N> y = (delta / _count) - _mean_accum;
			const matrix::Vector<Type, N> t = _mean + y;
			_mean_accum = (t - _mean) - y;
			_mean = t;
		}

		if (!_mean.isAllFinite()) {
			reset();
			return false;
		}

		// covariance
        //  Kahan summation (upper triangle only)
		{
			// eg C(x,y) += dx * (y - mean_y)
			matrix::SquareMatrix<Type, N> m2_change{};

			for (size_t r = 0; r < N; r++) {
				for (size_t c = r; c < N; c++) {
					m2_change(r, c) = delta(r) * (new_value(c) - _mean(c));
				}
			}

			for (size_t r = 0; r < N; r++) {
				for (size_t c = r; c < N; c++) {

					const Type y = m2_change(r, c) - _M2_accum(r, c);
					const Type t = _M2(r, c) + y;
					_M2_accum(r, c) = (t - _M2(r, c)) - y;

					_M2(r, c) = t;
				}

				// protect against floating point precision causing negative variances
				if (_M2(r, r) < 0) {
					_M2(r, r) = 0;
				}
			}

			// make symmetric
			for (size_t r = 0; r < N; r++) {
				for (size_t c = r + 1; c < N; c++) {
					_M2(c, r) = _M2(r, c);
				}
			}
		}

		if (!_M2.isAllFinite()) {
			reset();
			return false;
		}

		return valid();
	}

	/**
     * @brief Checks whether the current statistics are valid.
     *
     * Statistics are considered valid after at least 3 samples have been accumulated.
     * This ensures that the sample covariance (`_M2 / (count-1)`) is defined.
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
		_mean.zero();
		_M2.zero();

		_mean_accum.zero();
		_M2_accum.zero();
	}

	/**
     * @brief Returns the current mean vector.
     *
     * @return Running mean of all samples seen so far (zero vector if no samples).
     */
	matrix::Vector<Type, N> mean() const { return _mean; }

	/**
     * @brief Returns the variance vector (diagonal entries of the covariance matrix).
     *
     * Each component is the unbiased sample variance for that dimension.
     *
     * @return Vector of variances (length N).
     */
	matrix::Vector<Type, N> variance() const { return _M2.diag() / (_count - 1); }

	/**
     * @brief Returns the full sample covariance matrix.
     *
     * The covariance matrix is symmetric and positive semi‑definite.
     *
     * @return Covariance matrix (N×N).
     */
	matrix::SquareMatrix<Type, N> covariance() const { return _M2 / (_count - 1); }

	/**
     * @brief Returns the covariance between two dimensions.
     *
     * @param x Index of the first dimension (0‑based).
     * @param y Index of the second dimension (0‑based).
     * @return Sample covariance Cov(x, y).
     */
	Type covariance(int x, int y) const { return _M2(x, y) / (_count - 1); }

private:

	matrix::Vector<Type, N> _mean{};			///< Current mean vector.
	matrix::Vector<Type, N> _mean_accum{};  	///< kahan summation algorithm accumulator for mean

	matrix::SquareMatrix<Type, N> _M2{};		///< Scaled sum of outer products of deviation
	matrix::SquareMatrix<Type, N> _M2_accum{};  ///< kahan summation algorithm accumulator for M2

	uint16_t _count{0};							///< Number of samples processed.
};

} // namespace math
