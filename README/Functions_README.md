# Functions.hpp – Signal Shaping, Interpolation & Utility Functions

Header‑only C++ library providing a collection of simple, reusable mathematical functions. Used extensively in the PX4 flight stack for RC input shaping, deadzone handling, interpolation, bit operations, and safe finite checks.

## Overview

The file contains templated and overloaded functions that work with `float`, `double`, and integer types, as well as `matrix::Vector2f`/`Vector3f`. All functions are in the `math` namespace.

## Function Categories

### Sign & Basic Arithmetic

| Function | Description |
|----------|-------------|
| `signNoZero(val)` | Returns -1 for negative, 0 or 1 for zero/positive. |
| `signFromBool(positive)` | Returns 1 if true, -1 if false. |
| `sq(val)` | Returns `val * val`. |
| `negate(val)` | Generic negation (specialised for `int16_t` to avoid overflow). |
| `countSetBits(n)` | Hamming weight – counts set bits in an integer. |

### RC / Joystick Signal Shaping

| Function | Description |
|----------|-------------|
| `expo(value, e)` | Blends linear and cubic: `(1-e)*x + e*x³`. Range `[-1,1]`. |
| `superexpo(value, e, g)` | Adds rational steepening: `expo(x,e) * (1-g)/(1 - |x|*g)`. |
| `deadzone(value, dz)` | Linear outside `[-dz,dz]`, zero inside. |
| `expo_deadzone(value, e, dz)` | Applies deadzone first, then expo. |
| `sqrt_linear(value)` | `0` for negative, `sqrt(x)` for `0≤x≤1`, `x` for `x>1`. |

### Interpolation

| Function | Description |
|----------|-------------|
| `interpolate(value, x_low, x_high, y_low, y_high)` | Constant‑linear‑constant with two breakpoints. |
| `interpolateN(value, y_array)` | N equally spaced breakpoints on `[0,1]`. |
| `interpolateNXY(value, x_array, y_array)` | Arbitrary sorted breakpoints. |
| `lerp(a, b, s)` | Linear interpolation: `(1-s)*a + s*b`. |

### Finite Checks

| Function | Description |
|----------|-------------|
| `isFinite(float)` | Uses `PX4_ISFINITE`. |
| `isFinite(matrix::Vector2f)` | Checks all components. |
| `isFinite(matrix::Vector3f)` | Checks all components. |

## Example

```cpp
#include "Functions.hpp"
#include <cstdio>

int main() {
    // RC shaping
    float rc_input = 0.7f;
    float shaped = math::expo(rc_input, 0.3f);      // 0.3 linear, 0.7 cubic
    float with_deadzone = math::deadzone(rc_input, 0.1f);
    float super = math::superexpo(rc_input, 0.2f, 0.9f);

    // Interpolation
    float x = 0.5f;
    float y = math::interpolate(x, 0.0f, 1.0f, 10.0f, 20.0f); // 15.0

    const float knots[] = {0.0f, 0.5f, 1.0f};
    const float values[] = {0.0f, 1.0f, 0.0f};
    float interp = math::interpolateNXY(0.75f, knots, values); // 0.5

    // Safe negation for int16_t
    int16_t a = -32768;
    int16_t b = math::negate(a); // b = 32767

    // Finite check
    if (math::isFinite(1.0f/0.0f)) {
        // will not execute
    }

    return 0;
}
```

## Dependencies

- `Limits.hpp` – provides `constrain`, `min`, `max`, `isInRange`.
- `matrix/math.hpp` – for `matrix::sign`, `Vector2f`, `Vector3f`.
- `px4_platform_common/defines.h` – defines `PX4_ISFINITE`.

## Notes

- Most functions constrain inputs to their expected ranges (e.g., `expo` clamps to `[-1,1]`).
- `deadzone` and `superexpo` use `fabsf` – they are single‑precision. For `double`, replace with `fabs`.
- `negate<int16_t>` ensures symmetric overflow protection, useful for sensor/actuator limits.
- `interpolateN` expects `value` roughly in `[0,1]`; out‑of‑range is clamped.

## License

BSD 3‑Clause. Part of the PX4 project. See copyright header in the file.
```