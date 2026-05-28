# WelfordMean – Online Mean & Variance (Scalar)

Header‑only C++ class for **online (streaming) mean and variance** estimation of scalar data using Welford's algorithm. Optimised for numerical stability with Kahan summation and handles count overflow gracefully.

## Features

- **Single‑pass algorithm** – processes each sample once, O(1) memory and time per update.
- **Numerically stable** – uses Welford's method plus Kahan summation to reduce floating‑point error.
- **Overflow protection** – when sample count reaches `UINT16_MAX`, the internal count resets while preserving statistics.
- **Safety checks** – detects non‑finite values and resets automatically.
- **Unbiased estimates** – returns sample variance (`÷ (n‑1)`) and standard deviation.

## Dependencies

- `lib/mathlib/mathlib.h` – provides `math::max` and `PX4_ISFINITE` (part of PX4).
- C++11 or later.

## API Reference

### Template Parameter

| Parameter | Description |
|-----------|-------------|
| `Type`    | Floating‑point type (`float`, `double`). Default = `float`. |

### Public Methods

| Method | Description |
|--------|-------------|
| `bool update(const Type& value)` | Add a new sample. Returns `true` if statistics are valid afterwards (≥3 samples). |
| `bool valid() const` | Returns `true` if `count() > 2` (sample variance defined). |
| `auto count() const` | Number of samples processed so far (may wrap to 1 after overflow). |
| `void reset()` | Clear all state (count=0, mean=0, M2=0). |
| `Type mean() const` | Current sample mean. |
| `Type variance() const` | Unbiased sample variance. |
| `Type standard_deviation() const` | Sample standard deviation (sqrt of variance). |

## Example

```cpp
#include "WelfordMean.hpp"
#include <cstdio>

int main() {
    math::WelfordMean<float> stats;

    // Feed some measurements
    stats.update(10.0f);
    stats.update(12.0f);
    stats.update(11.0f);
    stats.update(13.0f);

    if (stats.valid()) {
        printf("Count: %u\n", stats.count());
        printf("Mean: %.3f\n", stats.mean());
        printf("Variance: %.3f\n", stats.variance());
        printf("StdDev: %.3f\n", stats.standard_deviation());
    }

    return 0;
}
```

## Algorithm Details

Welford's online algorithm maintains:

- `n` = number of samples
- `μ` = running mean
- `M₂` = sum of squared differences from the mean (`n * variance` when using population variance, or `(n-1) * sample_variance`)

For each new value `x`:

```
δ = x - μ
μ ← μ + δ / n
M₂ ← M₂ + δ * (x - μ)
```

At any point, the **sample variance** = `M₂ / (n-1)` (unbiased).  
The **standard deviation** = `√variance`.

### Kahan Summation

Both `μ` and `M₂` are updated using Kahan summation to minimise floating‑point drift over many samples.

### Overflow Handling

Because the counter is `uint16_t`, when `_count == UINT16_MAX` (65535), the current `_M2` is divided by `_count` (preserving the variance estimate) and the counter resets to 1. This avoids overflow while keeping the statistics correct.

## Limitations

- Counter is `uint16_t` – after ~65535 updates it will wrap, but statistics are preserved. For applications needing exact counts beyond 65535, modify the type to `uint32_t`.
- Weighted samples are not supported.
- Assumes samples are independent and identically distributed (i.i.d.).

## License

BSD 3‑Clause. Part of the PX4 project. See copyright header in the file.
```