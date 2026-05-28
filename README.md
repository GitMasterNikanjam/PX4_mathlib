# PX4 Math Library

This is the **mathlib** component originally from the [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) firmware, extracted for use as an independent, header‑only C++ library. It provides lightweight mathematical utilities for signal shaping, trajectory planning, online statistics, and optimisation – all designed for real‑time embedded systems with no dynamic allocation.

---

## Overview

`mathlib` is a collection of template‑based and `constexpr` functions that work with basic numeric types (`float`, `double`, `int16_t`, etc.) and optionally with the [PX4 Matrix library](https://github.com/PX4/Matrix) for vector/matrix support.

When used standalone, the library requires only a C++11 compiler and a few standard C headers. The only optional external dependency is the `matrix` library for the vector‑based modules (`WelfordMeanVector.hpp`).

---

## File Structure

```
mathlib/
├── math/
│   ├── Limits.hpp             – min, max, constrain, rad/deg conversion, isZero
│   ├── Functions.hpp          – expo, deadzone, superexpo, interpolation, sign, negate, countSetBits, isFinite
│   ├── TrajMath.hpp           – speed planning, braking distance, circle‑ray intersection
│   ├── SearchMin.hpp          – golden‑section 1D minimisation
│   ├── WelfordMean.hpp        – online mean & variance (scalar)
│   ├── WelfordMeanVector.hpp  – online mean & covariance (vector, requires matrix)
│   └── matrix_alg.h           – legacy raw‑array matrix ops (optional)
└── mathlib.h                  – umbrella include (if desired)
```

**Note**: The original `mathlib.h` simply includes the individual headers; you can create your own or include the needed headers directly.

---

## Component Documentation

### 1. `Limits.hpp` – Basic Arithmetic

| Function | Description |
|----------|-------------|
| `min(a,b)`, `max(a,b)` | Two‑argument minimum/maximum (also three‑argument overloads) |
| `constrain(val, min, max)` | Clamp `val` to `[min, max]` |
| `constrainFloatToInt16(val)` | Clamp float to `int16_t` range and cast |
| `isInRange(val, min, max)` | Returns true if `min ≤ val ≤ max` |
| `radians(deg)`, `degrees(rad)` | Angle conversion |
| `isZero(float)` / `isZero(double)` | ε‑based zero test (`FLT_EPSILON` / `DBL_EPSILON`) |

### 2. `Functions.hpp` – Signal Shaping & Interpolation

| Function | Description |
|----------|-------------|
| `signNoZero(val)` | Returns -1 for negative, 0/1 otherwise |
| `signFromBool(bool)` | Returns 1 if true, -1 if false |
| `sq(val)` | Returns `val * val` |
| `expo(value, e)` | Blends linear (`e=0`) and cubic (`e=1`), input in `[-1,1]` |
| `superexpo(value, e, g)` | Adds rational term for steeper curve |
| `deadzone(value, dz)` | Linear outside `[-dz, dz]`, zero inside |
| `expo_deadzone(value, e, dz)` | Deadzone then expo |
| `interpolate(val, xl, xh, yl, yh)` | Two‑point linear interpolation with saturation |
| `interpolateN(val, y[])` | N equally spaced breakpoints on `[0,1]` |
| `interpolateNXY(val, x[], y[])` | Arbitrary sorted breakpoints |
| `sqrt_linear(val)` | 0 for negative, `sqrt` for [0,1], linear above 1 |
| `lerp(a, b, s)` | `(1-s)*a + s*b` |
| `negate(val)` | Generic negation, specialised for `int16_t` (safe overflow) |
| `countSetBits(n)` | Hamming weight |
| `isFinite(float/double)` | Returns `isfinite` (requires `PX4_ISFINITE` or C++11 `std::isfinite`) |

> **Standalone note**: `isFinite` uses the macro `PX4_ISFINITE`. If you don't have that, replace with `std::isfinite(value)` from `<cmath>`.

### 3. `TrajMath.hpp` – Trajectory & Motion Planning

All functions are in the `math::trajectory` namespace.

| Function | Description |
|----------|-------------|
| `computeMaxSpeedFromDistance(jerk, accel, dist, v_final)` | Maximum initial speed that can be reduced to `v_final` within `dist` under jerk/accel limits |
| `computeMaxSpeedInWaypoint(alpha, accel, d)` | Max tangential speed through a V‑shaped corner (centripetal limit) |
| `computeBrakingDistanceFromVelocity(v, jerk, accel, delay_accel)` | Distance to stop from velocity `v` with jerk‑limited deceleration |
| `getMaxDistanceToCircle(pos, center, radius, dir)` | Farthest ray‑circle intersection distance (returns `NAN` if none) |

### 4. `SearchMin.hpp` – Golden Section Search

| Function | Description |
|----------|-------------|
| `goldensection(a, b, fun, tol)` | Finds the minimum of a unimodal function `fun` on `[a,b]` using golden‑section search. Returns approximate minimiser. |

### 5. `WelfordMean.hpp` – Online Statistics (Scalar)

| Method | Description |
|--------|-------------|
| `update(x)` | Add a new sample |
| `mean()` | Current sample mean |
| `variance()` | Unbiased sample variance |
| `standard_deviation()` | √variance |
| `valid()` | True after ≥3 samples |
| `count()` | Number of samples (may wrap at 65535) |
| `reset()` | Clear all state |

Uses Welford’s algorithm and Kahan summation for numerical stability. Handles count overflow.

### 6. `WelfordMeanVector.hpp` – Online Statistics (Vector)

**Requires the [PX4 Matrix library](https://github.com/PX4/Matrix).**  
Provides the same interface but for fixed‑size vectors. Maintains a mean vector and a full covariance matrix.

| Method | Description |
|--------|-------------|
| `update(Vector<N>)` | Add a new sample |
| `mean()` | Mean vector |
| `variance()` | Per‑component variances |
| `covariance()` | Full covariance matrix |
| `covariance(i, j)` | Single covariance entry |
| `valid()`, `count()`, `reset()` | Same as scalar version |

---

## Using the Library in an Independent Project

### Minimal Setup (Scalar‑only)

If you only need scalar functions (`Limits.hpp`, `Functions.hpp`, `TrajMath.hpp`, `SearchMin.hpp`, `WelfordMean.hpp`), simply copy the `math/` folder into your project and include the required headers. No external dependencies are required.

Example `CMakeLists.txt` snippet:

```cmake
add_library(mathlib INTERFACE)
target_include_directories(mathlib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/math)
```

Then in your code:

```cpp
#include "math/Limits.hpp"
#include "math/Functions.hpp"
#include "math/TrajMath.hpp"
#include "math/SearchMin.hpp"
#include "math/WelfordMean.hpp"
```

### Full Setup (with Vector Statistics)

For `WelfordMeanVector.hpp` you also need the `matrix` library. The easiest way is to fetch it as a CMake dependency:

```cmake
include(FetchContent)
FetchContent_Declare(
    matrix
    GIT_REPOSITORY https://github.com/PX4/Matrix.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(matrix)

add_library(mathlib INTERFACE)
target_include_directories(mathlib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/math)
target_link_libraries(mathlib INTERFACE matrix)
```

Then include the vector header:

```cpp
#include "math/WelfordMeanVector.hpp"
```

### Handling `PX4_ISFINITE`

The original code uses `PX4_ISFINITE` in `Functions.hpp` and `WelfordMean.hpp`. To make the library standalone, either:

1. **Define the macro yourself** (e.g., in a config header) before including `mathlib`:
   ```cpp
   #ifndef PX4_ISFINITE
   #define PX4_ISFINITE(x) std::isfinite(x)
   #endif
   ```

2. **Patch the headers** by replacing `PX4_ISFINITE` with `std::isfinite` (requires `<cmath>`).

The same applies to `math::max` used in `WelfordMean.hpp` – it comes from `Limits.hpp`, which is part of this library, so no extra work is needed.

---

## Example Programs

### 1. Using RC Shaping

```cpp
#include "math/Functions.hpp"
#include <cstdio>

int main() {
    float stick = 0.8f;
    float expo_out = math::expo(stick, 0.4f);        // 0.4 linear, 0.6 cubic
    float dz_out = math::deadzone(stick, 0.1f);      // deadzone ±0.1
    printf("expo: %f, deadzone: %f\n", expo_out, dz_out);
    return 0;
}
```

### 2. Trajectory Speed Planning

```cpp
#include "math/TrajMath.hpp"
#include <cstdio>

int main() {
    float max_speed = math::trajectory::computeMaxSpeedFromDistance(5.0f, 2.0f, 30.0f, 1.0f);
    float brake_dist = math::trajectory::computeBrakingDistanceFromVelocity(10.0f, 4.0f, 2.5f, 2.5f);
    printf("Max initial speed: %.2f m/s, braking distance: %.2f m\n", max_speed, brake_dist);
    return 0;
}
```

### 3. Online Statistics (Vector)

```cpp
#include "math/WelfordMeanVector.hpp"
#include <matrix/math.hpp>
#include <cstdio>

int main() {
    math::WelfordMeanVector<float, 3> estimator;
    estimator.update(matrix::Vector3f(1.0f, 2.0f, 3.0f));
    estimator.update(matrix::Vector3f(1.1f, 2.1f, 2.9f));
    estimator.update(matrix::Vector3f(0.9f, 1.9f, 3.1f));

    if (estimator.valid()) {
        auto mean = estimator.mean();
        auto cov = estimator.covariance();
        printf("Mean: (%f, %f, %f)\n", mean(0), mean(1), mean(2));
        printf("Covariance(0,0): %f\n", cov(0,0));
    }
    return 0;
}
```

---

## Portability

- **Compiler**: C++11 or later.
- **Platforms**: Any platform with a standard C++ library (Linux, Windows, macOS, embedded RTOS).
- **Endianness**: Not relevant (no binary serialisation).
- **Thread safety**: Not guaranteed – the classes contain mutable state and are not thread‑safe by design.

---

## License

All files are licensed under the **BSD 3‑Clause License**. The full text is included in each header. This license allows both open‑source and proprietary use, provided the copyright notice and disclaimer are retained.

---

## Getting the Code

You can clone the original repository and extract the `mathlib` folder, or download the individual files from:

- [PX4-Autopilot/src/lib/mathlib](https://github.com/PX4/PX4-Autopilot/tree/main/src/lib/mathlib)

For the `matrix` library (optional), visit: [github.com/PX4/Matrix](https://github.com/PX4/Matrix)

---

## Acknowledgements

Originally developed by the PX4 Development Team. This standalone version is a derivative work under the same BSD 3‑Clause license.