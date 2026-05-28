/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file TrajMath.hpp
 * @brief Mathematical utilities for trajectory generation and speed limiting.
 *
 * This header provides functions to compute maximum allowable speeds based on
 * kinematic constraints (jerk, acceleration, distance) and geometric waypoint
 * turning limits. It also includes helpers for braking distance and circle‑line
 * intersection used in path planning.
 */

#pragma once

namespace math
{

/**
 * @namespace trajectory
 * @brief Functions for trajectory generation and speed planning.
 */
namespace trajectory
{

/**
 * @brief Computes the maximum possible speed over a remaining distance under jerk and acceleration limits.
 *
 * The function solves the equation resulting from a constant‑acceleration braking profile
 * that includes a finite jerk‑limited transition time (2 * accel / jerk) to reverse acceleration.
 * The derived formula is:
 *   v_final² = v_initial² - 2 * accel * (distance - v_initial * 2 * accel / jerk)
 * Solved for v_initial (max_speed) given the braking distance, final speed, jerk, and accel.
 *
 * @param jerk Maximum jerk (positive, units: m/s³ or similar).
 * @param accel Maximum acceleration magnitude (positive, units: m/s²).
 * @param braking_distance Remaining distance to the target point (units: m).
 * @param final_speed Desired speed when reaching the target (units: m/s).
 *
 * @return Maximum allowed initial speed (≥ final_speed) that can be reduced to final_speed
 *         within the given distance respecting jerk and acceleration limits.
 */
inline float computeMaxSpeedFromDistance(const float jerk, const float accel, const float braking_distance,
		const float final_speed)
{
	auto sqr = [](float f) {return f * f;};
	float b =  4.0f * sqr(accel) / jerk;
	float c = - 2.0f * accel * braking_distance - sqr(final_speed);
	float max_speed = 0.5f * (-b + sqrtf(sqr(b) - 4.0f * c));

	// don't slow down more than the end speed, even if the conservative accel ramp time requests it
	return fmaxf(max_speed, final_speed);
}

/**
 * @brief Computes the maximum tangential speed through a V‑shaped waypoint under lateral acceleration limit.
 *
 * The waypoint is formed by two line segments of equal length `d` meeting at an angle `alpha`.
 * A circle is inscribed tangent to the ends of both segments. The maximum tangential speed
 * is derived from the centripetal acceleration constraint: a = v² / R, where R = d * tan(α/2).
 *
 * @verbatim
 *      \\
 *      | \ d
 *      /  \
 *  __='___a\
 *      d
 * @endverbatim
 *
 * @param alpha Opening angle between the two segments (radians).
 * @param accel Maximum lateral acceleration (units: m/s²).
 * @param d Length of each line segment (units: m).
 *
 * @return Maximum tangential speed (units: m/s) that keeps lateral acceleration ≤ accel.
 */
inline float computeMaxSpeedInWaypoint(const float alpha, const float accel, const float d)
{
	float tan_alpha = tanf(alpha / 2.0f);
	float max_speed_in_turn = sqrtf(accel * d * tan_alpha);

	return max_speed_in_turn;
}

/**
 * @brief Computes the braking distance needed to reduce from an initial velocity to zero,
 *        considering jerk‑limited acceleration reversal.
 *
 * The braking profile assumes constant deceleration `accel` after a delay time of
 * `accel_delay_max / jerk` during which acceleration ramps from +accel to -accel.
 * The derived distance is: distance = v * (v / (2*accel) + accel_delay_max / jerk).
 *
 * @param velocity Initial forward velocity (units: m/s).
 * @param jerk Maximum jerk (units: m/s³).
 * @param accel Maximum deceleration magnitude (positive, units: m/s²).
 * @param accel_delay_max The acceleration value that defines the delay (typically = accel).
 *
 * @return Minimum distance required to come to a stop (units: m).
 */
inline float computeBrakingDistanceFromVelocity(const float velocity, const float jerk, const float accel,
		const float accel_delay_max)
{
	return velocity * (velocity / (2.0f * accel) + accel_delay_max / jerk);
}

/**
 * @brief Finds the farthest distance from a point to a circle along a given direction.
 *
 * Given a point P, a circle (center C, radius R), and a direction vector D (pointing from P
 * towards the circle), this function computes the distance `t` such that
 * P + t * (D/|D|) lies on the circle. If the ray from P in direction D intersects the circle
 * at two points, the larger (farther) intersection distance is returned.
 *
 * @verbatim
 *                  _
 *               ,=' '=,               __
 *    P-->------/-------A   Distance = PA
 *       Dir   |    x    |
 *              \       /
 *               "=,_,="
 * @endverbatim
 *
 * @param pos Position of the starting point.
 * @param circle_pos Center of the circle.
 * @param radius Radius of the circle.
 * @param direction Vector pointing from the point towards the circle (need not be unit).
 *
 * @return The longest distance from the point to the circle along the given direction.
 *         Returns NAN if the ray does not intersect the circle (i.e., direction points away,
 *         or the line misses the circle entirely).
 */
inline float getMaxDistanceToCircle(const matrix::Vector2f &pos, const matrix::Vector2f &circle_pos, float radius,
				    const matrix::Vector2f &direction)
{
	matrix::Vector2f center_to_pos = pos - circle_pos;
	const float b = 2.f * center_to_pos.dot(direction.unit_or_zero());
	const float c = center_to_pos.norm_squared() - radius * radius;
	const float delta = b * b - 4.f * c;

	float distance_to_circle;

	if (delta >= 0.f && direction.longerThan(0.f)) {
		distance_to_circle = fmaxf((-b + sqrtf(delta)) / 2.f, 0.f);

	} else {
		// Never intersecting the circle
		distance_to_circle = NAN;
	}

	return distance_to_circle;
}

} /* namespace traj */
} /* namespace math */
