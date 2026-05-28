# SearchMin.hpp – Golden Section Search

Header-only C++ implementation of the **golden section search** algorithm for finding the minimum of a unimodal function on a closed interval. No derivatives required.

## Overview

Given a function `f(x)` that is **unimodal** (strictly decreasing then strictly increasing) on `[a, b]`, the golden section search iteratively narrows the interval containing the minimum. Each iteration evaluates `f` at two interior points, discarding the sub‑interval where the minimum cannot lie. The golden ratio `φ ≈ 1.618` is used to choose the interior points, ensuring the interval shrinks by a factor of `φ` per evaluation.

## API

```cpp
namespace math {
    template<typename _Tp>
    _Tp goldensection(const _Tp &arg1, const _Tp &arg2,
                      _Tp (*fun)(_Tp), const _Tp &tol);
}
```

### Parameters

| Parameter | Description |
|-----------|-------------|
| `arg1` | Left endpoint of the search interval (must be `< arg2`) |
| `arg2` | Right endpoint of the search interval |
| `fun`   | Function pointer: `_Tp f(_Tp)` – the objective to minimize |
| `tol`   | Absolute tolerance – iteration stops when interval width ≤ `tol` |

### Return Value

Approximation of the minimizer `x*` (the midpoint of the final interval).

## Example

```cpp
#include "SearchMin.hpp"
#include <cmath>
#include <iostream>

double my_function(double x) {
    // A simple parabola with minimum at x = 2.0
    return (x - 2.0) * (x - 2.0) + 1.0;
}

int main() {
    double a = 0.0;
    double b = 5.0;
    double tolerance = 1e-6;

    double minimum_x = math::goldensection(a, b, my_function, tolerance);

    std::cout << "Minimum found at x = " << minimum_x << std::endl;
    // Output: Minimum found at x ≈ 2.000000
    return 0;
}
```

## Dependencies

- Standard C++11 or later (templates, `constexpr`).
- No external libraries – pure header.

## Notes

- The function **must** be unimodal on `[arg1, arg2]` (one minimum, monotonic on each side). If multiple minima exist, the algorithm may converge to one of them but without guarantee.
- For flat regions near the minimum, convergence may be slower – tolerance should be chosen according to the scale of `x`.
- The implementation uses a type‑generic `abs_t` that works for any numeric type (float, double, int, etc.). For floating‑point types, `tol` should be positive and reasonable (e.g., `1e-6` for double).
- The golden ratio constant `GOLDEN_RATIO` is defined as `static constexpr double`.

## Algorithm Details

The interval `[a, b]` is reduced by evaluating `f` at:
```
c = b - (b - a) / φ
d = a + (b - a) / φ
```
If `f(c) < f(d)`, the minimum lies in `[a, d]` (set `b = d`), otherwise it lies in `[c, b]` (set `a = c`). The process repeats until `|c - d| ≤ tol`.

## License

BSD 3‑Clause – see copyright header in the file. Part of the PX4 project.
```