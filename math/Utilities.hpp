/****************************************************************************
 *
 *   Copyright (c) 2020-2024 PX4 Development Team. All rights reserved.
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
 * @file Utilities.hpp
 * @brief Mathematical utilities for rotations, Euler angles, and quaternion conversions.
 *
 * This header provides helper functions for converting between rotation representations,
 * extracting yaw angles while avoiding gimbal lock, and updating rotation matrices with
 * new yaw values. It supports both 3-2-1 (yaw-pitch-roll) and 3-1-2 (yaw-roll-pitch)
 * Tait‑Bryan sequences.
 */

#ifndef MATH_UTILITIES_H
#define MATH_UTILITIES_H

#include <matrix/math.hpp>

namespace math
{

/**
 * @namespace Utilities
 * @brief Collection of rotation-related utility functions.
 */
namespace Utilities
{

/**
 * @brief Returns the square of a floating-point number.
 * @param var Input value.
 * @return var * var.
 */
static constexpr float sq(float var) { return var * var; }

/**
 * @brief Converts a 3-1-2 Tait‑Bryan rotation sequence to a rotation matrix.
 *
 * The sequence applies rotations in the order: first about Z, then about X, then about Y.
 * The resulting matrix rotates vectors from frame 2 to frame 1.
 * See http://www.atacolorado.com/eulersequences.doc
 * 
 * @tparam T Floating-point type (default float).
 * @param rot312 Vector containing (yaw, roll, pitch) in radians:
 *               rot312(0) = first rotation about Z (yaw)
 *               rot312(1) = second rotation about X (roll)
 *               rot312(2) = third rotation about Y (pitch)
 * @return Direction cosine matrix (rotation matrix) from frame 2 to frame 1.
 */
template<typename T = float>
inline matrix::Dcm<T> taitBryan312ToRotMat(const matrix::Vector3<T> &rot312)
{
	// Calculate the frame2 to frame 1 rotation matrix from a 312 Tait-Bryan rotation sequence
	const T c2 = cosf(rot312(2)); // third rotation is pitch
	const T s2 = sinf(rot312(2));
	const T s1 = sinf(rot312(1)); // second rotation is roll
	const T c1 = cosf(rot312(1));
	const T s0 = sinf(rot312(0)); // first rotation is yaw
	const T c0 = cosf(rot312(0));

	matrix::Dcm<T> R;
	R(0, 0) = c0 * c2 - s0 * s1 * s2;
	R(1, 1) = c0 * c1;
	R(2, 2) = c2 * c1;
	R(0, 1) = -c1 * s0;
	R(0, 2) = s2 * c0 + c2 * s1 * s0;
	R(1, 0) = c2 * s0 + s2 * s1 * c0;
	R(1, 2) = s0 * s2 - s1 * c0 * c2;
	R(2, 0) = -s2 * c1;
	R(2, 1) = s1;

	return R;
}

/**
 * @brief Converts a quaternion to the corresponding inverse rotation matrix.
 *
 * Computes the direction cosine matrix that represents the same orientation as the
 * quaternion, but with the transpose (inverse) relationship.
 *
 * @tparam T Floating-point type (default float).
 * @param quat Quaternion (w, x, y, z) representing rotation from frame 1 to frame 2.
 * @return Dcm matrix rotating from frame 2 to frame 1 (inverse of quaternion's matrix).
 */
template<typename T = float>
inline matrix::Dcm<T> quatToInverseRotMat(const matrix::Quaternion<T> &quat)
{
	const T q00 = quat(0) * quat(0);
	const T q11 = quat(1) * quat(1);
	const T q22 = quat(2) * quat(2);
	const T q33 = quat(3) * quat(3);
	const T q01 = quat(0) * quat(1);
	const T q02 = quat(0) * quat(2);
	const T q03 = quat(0) * quat(3);
	const T q12 = quat(1) * quat(2);
	const T q13 = quat(1) * quat(3);
	const T q23 = quat(2) * quat(3);

	matrix::Dcm<T> dcm;
	dcm(0, 0) = q00 + q11 - q22 - q33;
	dcm(1, 1) = q00 - q11 + q22 - q33;
	dcm(2, 2) = q00 - q11 - q22 + q33;
	dcm(1, 0) = 2.0f * (q12 - q03);
	dcm(2, 0) = 2.0f * (q13 + q02);
	dcm(0, 1) = 2.0f * (q12 + q03);
	dcm(2, 1) = 2.0f * (q23 - q01);
	dcm(0, 2) = 2.0f * (q13 - q02);
	dcm(1, 2) = 2.0f * (q23 + q01);

	return dcm;
}

/**
 * @brief Determines whether to use the 3-2-1 (yaw-pitch-roll) rotation sequence.
 *
 * The 3-2-1 sequence is preferred when roll tilt is larger than pitch tilt;
 * otherwise the 3-1-2 sequence is used to avoid gimbal lock.
 *
 * @tparam T Floating-point type (default float).
 * @param R Rotation matrix.
 * @return true if |R(2,0)| < |R(2,1)| (use 3-2-1), false otherwise (use 3-1-2).
 */
template<typename T = float>
inline bool shouldUse321RotationSequence(const matrix::Dcm<T> &R)
{
	return fabsf(R(2, 0)) < fabsf(R(2, 1));
}

/**
 * @brief Extracts yaw angle from a rotation matrix using the 3-2-1 sequence.
 * @tparam T Floating-point type.
 * @param R Rotation matrix.
 * @return Yaw angle in radians (range -π to π).
 */
template<typename T = float>
inline float getEuler321Yaw(const matrix::Dcm<T> &R)
{
	return atan2f(R(1, 0), R(0, 0));
}

/**
 * @brief Extracts yaw angle from a rotation matrix using the 3-1-2 sequence.
 * @tparam T Floating-point type.
 * @param R Rotation matrix.
 * @return Yaw angle in radians (range -π to π).
 */
template<typename T = float>
inline float getEuler312Yaw(const matrix::Dcm<T> &R)
{
	return atan2f(-R(0, 1), R(1, 1));
}

/**
 * @brief Extracts yaw angle from a quaternion using the 3-2-1 sequence.
 * @tparam T Floating-point type.
 * @param q Quaternion (w, x, y, z).
 * @return Yaw angle in radians.
 */
template<typename T = float>
inline T getEuler321Yaw(const matrix::Quaternion<T> &q)
{
	// Values from yaw_input_321.c file produced by
	// https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw321.m
	const T a = static_cast<T>(2.) * (q(0) * q(3) + q(1) * q(2));
	const T b = sq(q(0)) + sq(q(1)) - sq(q(2)) - sq(q(3));
	return atan2f(a, b);
}

/**
 * @brief Extracts yaw angle from a quaternion using the 3-1-2 sequence.
 * @tparam T Floating-point type.
 * @param q Quaternion (w, x, y, z).
 * @return Yaw angle in radians.
 */
template<typename T = float>
inline T getEuler312Yaw(const matrix::Quaternion<T> &q)
{
	// Values from yaw_input_312.c file produced by
	// https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw312.m
	const T a = static_cast<T>(2.) * (q(0) * q(3) - q(1) * q(2));
	const T b = sq(q(0)) - sq(q(1)) + sq(q(2)) - sq(q(3));
	return atan2f(a, b);
}

/**
 * @brief Extracts yaw angle from a rotation matrix, automatically selecting the best sequence.
 *
 * Chooses between 3-2-1 and 3-1-2 sequences based on `shouldUse321RotationSequence()` to
 * avoid gimbal lock.
 *
 * @tparam T Floating-point type.
 * @param R Rotation matrix.
 * @return Yaw angle in radians.
 */
template<typename T = float>
inline T getEulerYaw(const matrix::Dcm<T> &R)
{
	if (shouldUse321RotationSequence(R)) {
		return getEuler321Yaw(R);

	} else {
		return getEuler312Yaw(R);
	}
}

/**
 * @brief Extracts yaw angle from a quaternion, automatically selecting the best sequence.
 * @tparam T Floating-point type.
 * @param q Quaternion (w, x, y, z).
 * @return Yaw angle in radians.
 */
template<typename T = float>
inline T getEulerYaw(const matrix::Quaternion<T> &q)
{
	return getEulerYaw(matrix::Dcm<T>(q));
}

/**
 * @brief Updates a rotation matrix with a new yaw angle using the 3-2-1 sequence.
 *
 * The pitch and roll angles are preserved from the input matrix.
 *
 * @tparam T Floating-point type.
 * @param yaw New yaw angle in radians.
 * @param rot_in Input rotation matrix.
 * @return Rotation matrix with updated yaw.
 */
template<typename T = float>
inline matrix::Dcm<T> updateEuler321YawInRotMat(T yaw, const matrix::Dcm<T> &rot_in)
{
	matrix::Euler<T> euler321(rot_in);
	euler321(2) = yaw;
	return matrix::Dcm<T>(euler321);
}

/**
 * @brief Updates a rotation matrix with a new yaw angle using the 3-1-2 sequence.
 *
 * Roll and pitch are re-computed from the input matrix using the 3-1-2 convention.
 *
 * @tparam T Floating-point type.
 * @param yaw New yaw angle in radians.
 * @param rot_in Input rotation matrix.
 * @return Rotation matrix with updated yaw.
 */
template<typename T = float>
inline matrix::Dcm<T> updateEuler312YawInRotMat(T yaw, const matrix::Dcm<T> &rot_in)
{
	const matrix::Vector3<T> rotVec312(yaw,                                  // yaw
					   asinf(rot_in(2, 1)),                  // roll
					   atan2f(-rot_in(2, 0), rot_in(2, 2))); // pitch
	return taitBryan312ToRotMat(rotVec312);
}

/**
 * @brief Updates a rotation matrix with a new yaw angle, automatically selecting the sequence.
 *
 * Calls either `updateEuler321YawInRotMat` or `updateEuler312YawInRotMat` based on the
 * `shouldUse321RotationSequence` test.
 *
 * @tparam T Floating-point type.
 * @param yaw New yaw angle in radians.
 * @param rot_in Input rotation matrix.
 * @return Rotation matrix with updated yaw.
 */
template<typename T = float>
inline matrix::Dcm<T> updateYawInRotMat(T yaw, const matrix::Dcm<T> &rot_in)
{
	if (shouldUse321RotationSequence(rot_in)) {
		return updateEuler321YawInRotMat(yaw, rot_in);

	} else {
		return updateEuler312YawInRotMat(yaw, rot_in);
	}
}

} // namespace Utilities
} // namespace math

#endif // MATH_UTILITIES_H
