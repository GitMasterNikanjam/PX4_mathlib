# Limits.hpp – Basic Mathematical Limits & Utilities

Header‑only C++ library providing constexpr templates for common operations: min/max, clamping, range checking, degree/radian conversion, and safe floating‑point zero tests. Designed for real‑time embedded systems (PX4 flight stack).

## Overview

This file contains lightweight, compile‑time evaluable functions that avoid C++ standard library overhead. All functions are in the `math` namespace.

## Functions

### Minimum / Maximum

| Function | Description |
|----------|-------------|
| `min(a, b)` | Smaller of two values |
| `min(a, b, c)` | Smallest of three values |
| `max(a, b)` | Larger of two values |
| `max(a, b, c)` | Largest of three values |

```cpp
int x = math::min(5, 10);        // x = 5
float y = math::max(1.2f, 3.4f); // y = 3.4f
```

### Constraint & Range

| Function | Description |
|----------|-------------|
| `constrain(val, min, max)` | Clamp `val` to interval `[min, max]` |
| `constrainFloatToInt16(val)` | Clamp float to `int16_t` range and cast |
| `isInRange(val, min, max)` | Check if `min ≤ val ≤ max` |

```cpp
float v = math::constrain(15.0f, 0.0f, 10.0f);   // v = 10.0f
int16_t i = math::constrainFloatToInt16(40000.0f); // i = 32767
bool inside = math::isInRange(5, 0, 10);          // inside = true
```

### Angle Conversion

| Function | Description |
|----------|-------------|
| `radians(deg)` | Degrees → radians |
| `degrees(rad)` | Radians → degrees |

```cpp
double rad = math::radians(180.0); // rad ≈ 3.14159
double deg = math::degrees(M_PI);   // deg = 180.0
```

### Floating‑Point Zero Test

| Function | Description |
|----------|-------------|
| `isZero(float)` | `fabsf(val) < FLT_EPSILON` |
| `isZero(double)` | `fabs(val) < DBL_EPSILON` |

```cpp
if (math::isZero(0.000001f)) { /* true if within float epsilon */ }
```

## Example

```cpp
#include "Limits.hpp"
#include <cstdio>

int main() {
    float angle_deg = 90.0f;
    float angle_rad = math::radians(angle_deg);
    
    float clamped = math::constrain(angle_rad, 0.0f, 1.0f);
    printf("rad = %f, clamped = %f\n", angle_rad, clamped);
    
    int16_t output = math::constrainFloatToInt16(123.456f);
    return 0;
}
```

## Dependencies

- `<float.h>` – for `FLT_EPSILON`, `DBL_EPSILON`
- `<math.h>` – for `fabsf`, `fabs`
- `<stdint.h>` – for `INT16_MIN`, `INT16_MAX`
- C++11 or later (`constexpr`)

## Notes

- All functions except `isZero` are `constexpr` – can be used at compile time.
- The `radians`/`degrees` templates work with integer types, but results are truncated (use floating‑point for precision).
- `isZero` uses absolute tolerance; for scale‑dependent checks, consider relative tolerance.
- Part of the PX4 project, BSD 3‑Clause licensed.
```