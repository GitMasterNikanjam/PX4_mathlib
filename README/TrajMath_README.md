# TrajMath – Trajectory Generation Mathematics

This header (`TrajMath.hpp`) provides a set of lightweight, standalone functions for trajectory planning and speed limiting. All functions are designed for real‑time embedded use (e.g., flight controllers, autonomous vehicles) and operate on SI units (meters, seconds, radians).

## Features

- **Jerk‑limited speed planning** – compute maximum speed over a remaining distance given jerk and acceleration limits.
- **Waypoint turning speed** – limit tangential speed through a V‑shaped corner based on lateral acceleration.
- **Braking distance** – estimate stopping distance with a realistic deceleration profile (including jerk delay).
- **Circle‑line intersection** – find the farthest intersection of a ray with a circle (useful for path follow‑around obstacles).

All functions are `inline` and live in the `math::trajectory` namespace.

## Function Reference

### `computeMaxSpeedFromDistance`

```cpp
float computeMaxSpeedFromDistance(float jerk, float accel, float braking_distance, float final_speed);
```

**Purpose**  
Given a remaining distance, determine the maximum starting speed that can be reduced to a target `final_speed` without exceeding jerk or acceleration limits.

**Kinematic model**  
The profile assumes a constant deceleration `accel` after a jerk‑limited ramp time of `2*accel/jerk`. The governing equation is:

```
v_final² = v_initial² - 2 * accel * (distance - v_initial * (2 * accel / jerk))
```

The function solves for `v_initial` (returned as `max_speed`).

**Parameters**  
- `jerk` – maximum jerk (positive, m/s³)  
- `accel` – maximum acceleration magnitude (positive, m/s²)  
- `braking_distance` – remaining distance to target (m)  
- `final_speed` – desired speed at target (m/s)  

**Returns** – maximum initial speed (≥ `final_speed`) that satisfies the constraints.

---

### `computeMaxSpeedInWaypoint`

```cpp
float computeMaxSpeedInWaypoint(float alpha, float accel, float d);
```

**Purpose**  
Compute the maximum tangential speed when traversing a sharp waypoint formed by two equal‑length segments.

**Geometry**  
Two line segments of length `d` meet at an opening angle `α`. An inscribed circle of radius `R = d * tan(α/2)` is tangent to the ends of both segments. The maximum speed is limited by the lateral (centripetal) acceleration:

```
v_max = sqrt(accel * d * tan(α/2))
```

**Parameters**  
- `alpha` – angle between the two segments (radians)  
- `accel` – maximum lateral acceleration (m/s²)  
- `d` – length of each segment (m)  

**Returns** – maximum tangential speed (m/s).

---

### `computeBrakingDistanceFromVelocity`

```cpp
float computeBrakingDistanceFromVelocity(float velocity, float jerk, float accel, float accel_delay_max);
```

**Purpose**  
Calculate the distance required to come to a complete stop from an initial velocity using a deceleration profile that includes a jerk‑limited delay.

**Profile**  
The deceleration ramps from `+accel` to `-accel` over a delay `accel_delay_max / jerk`, after which it stays at `-accel`. The resulting distance is:

```
distance = velocity * ( velocity / (2 * accel) + accel_delay_max / jerk )
```

**Parameters**  
- `velocity` – initial forward speed (m/s)  
- `jerk` – maximum jerk (m/s³)  
- `accel` – maximum deceleration magnitude (positive, m/s²)  
- `accel_delay_max` – acceleration value defining the delay (typically equal to `accel`)  

**Returns** – minimum braking distance (m).

---

### `getMaxDistanceToCircle`

```cpp
float getMaxDistanceToCircle(const matrix::Vector2f &pos, const matrix::Vector2f &circle_pos,
                             float radius, const matrix::Vector2f &direction);
```

**Purpose**  
Find the farthest distance from a point to a circle along a given direction vector. Equivalent to solving for the largest `t ≥ 0` such that:

```
|pos + t * (direction/|direction|) - circle_pos| = radius
```

If the ray intersects the circle twice, the larger (farther) intersection distance is returned.

**Parameters**  
- `pos` – starting point (2D vector)  
- `circle_pos` – centre of the circle (2D vector)  
- `radius` – circle radius (m)  
- `direction` – vector pointing from `pos` towards the circle (need not be unit)  

**Returns** – distance to the farthest intersection point, or `NAN` if the ray does not intersect the circle (e.g., direction points away, or the line misses the circle entirely).

---

## Example Usage

```cpp
#include "TrajMath.hpp"
#include <matrix/math.hpp>
#include <cmath>
#include <cstdio>

using namespace math::trajectory;

int main() {
    // 1. Speed limit over remaining distance
    float max_speed = computeMaxSpeedFromDistance(5.0f, 2.0f, 30.0f, 1.0f);
    printf("Max initial speed: %.2f m/s\n", max_speed);

    // 2. Turning speed at a 90° corner with 5 m segments
    float v_turn = computeMaxSpeedInWaypoint(M_PI_2, 3.0f, 5.0f);
    printf("Max turn speed: %.2f m/s\n", v_turn);

    // 3. Braking distance from 10 m/s
    float stop_dist = computeBrakingDistanceFromVelocity(10.0f, 4.0f, 2.5f, 2.5f);
    printf("Braking distance: %.2f m\n", stop_dist);

    // 4. Circle intersection (point outside, ray toward circle)
    matrix::Vector2f pos(0.0f, 0.0f);
    matrix::Vector2f circle_center(10.0f, 0.0f);
    matrix::Vector2f direction(1.0f, 0.0f);
    float dist = getMaxDistanceToCircle(pos, circle_center, 3.0f, direction);
    if (!std::isnan(dist)) {
        printf("Intersection distance: %.2f\n", dist);
    }

    return 0;
}
```

## Dependencies

- `matrix/math.hpp` – PX4 matrix library (for `Vector2f` and basic vector operations).  
- `<cmath>` – implicitly used for `sqrtf`, `tanf`, `NAN`.

## Notes

- All functions use `float` precision. For double precision, copy and adapt the code.
- The braking distance formula in `computeBrakingDistanceFromVelocity` assumes the delay time is `accel_delay_max / jerk`. If `accel_delay_max != accel`, the initial overshoot in acceleration is different; for most cases pass `accel_delay_max = accel`.
- `getMaxDistanceToCircle` requires `direction.longerThan(0.f)` to avoid division by zero; the `unit_or_zero()` method returns a unit vector or zero if the input is too short. If the direction has zero length, the function returns `NAN`.
- The V‑waypoint model in `computeMaxSpeedInWaypoint` assumes the vehicle follows the inscribed circle exactly; in practice a smoother (clothoid) transition may be used, but this provides a conservative upper bound.

## License

Same as the original PX4 code – BSD 3‑Clause. See header for full terms.
```