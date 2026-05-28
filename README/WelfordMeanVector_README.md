# WelfordMeanVector – Online Mean & Covariance for Vectors

Header‑only C++ class for **online (streaming) mean and covariance estimation** of fixed‑size vectors using Welford's algorithm. Optimised for numerical stability with Kahan summation and handles count overflow gracefully.

## Features

- **Single‑pass algorithm** – processes each sample once, O(1) memory and time per update.
- **Numerically stable** – uses Welford's method plus Kahan summation to reduce floating‑point error.
- **Multivariate** – maintains mean vector and full covariance matrix of size `N×N` (for any dimension `N`).
- **Overflow protection** – when sample count reaches `UINT16_MAX`, the internal count resets while preserving statistics.
- **Safety checks** – detects non‑finite values and resets automatically.

## Dependencies

- `matrix::Vector<Type, N>` and `matrix::SquareMatrix<Type, N>` – from the [PX4 matrix library](https://github.com/PX4/matrix).
- C++11 or later.

## API Reference

### Template Parameters

| Parameter | Description |
|-----------|-------------|
| `Type`    | Floating‑point type (`float`, `double`, etc.). |
| `N`       | Dimension of the vectors (must be known at compile time). |

### Public Methods

| Method | Description |
|--------|-------------|
| `bool update(const Vector<Type,N>& v)` | Add a new sample. Returns `true` if statistics are valid afterwards (≥3 samples). |
| `bool valid() const` | Returns `true` if `count() > 2` (covariance matrix defined). |
| `uint16_t count() const` | Number of samples processed so far (may wrap to 1 after overflow). |
| `void reset()` | Clear all state (count=0, zero mean, zero M2). |
| `Vector<Type,N> mean() const` | Current mean vector. |
| `Vector<Type,N> variance() const` | Sample variance for each dimension (diagonal of covariance). |
| `SquareMatrix<Type,N> covariance() const` | Full sample covariance matrix (symmetric). |
| `Type covariance(int x, int y) const` | Sample covariance between dimension `x` and `y`. |

## Example

```cpp
#include "WelfordMeanVector.hpp"
#include <matrix/math.hpp>

int main() {
    // 3D vector estimator
    math::WelfordMeanVector<float, 3> estimator;

    // Simulate some measurements
    estimator.update(matrix::Vector3f(1.0f, 2.0f, 3.0f));
    estimator.update(matrix::Vector3f(1.1f, 2.1f, 2.9f));
    estimator.update(matrix::Vector3f(0.9f, 1.9f, 3.1f));

    if (estimator.valid()) {
        auto mean = estimator.mean();
        auto cov = estimator.covariance();

        printf("Mean: %.2f, %.2f, %.2f\n", mean(0), mean(1), mean(2));
        printf("Covariance matrix:\n");
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                printf("%.4f ", cov(i,j));
            }
            printf("\n");
        }
    }

    return 0;
}
```

## Algorithm Notes

- **Welford's recurrence** for multivariate data:
  ```
  δ = x - μ
  μ ← μ + δ / n
  M₂ ← M₂ + δ ⊗ (x - μ_new)
  ```
  where `M₂` accumulates the scaled sum of outer products, and the unbiased sample covariance is `M₂ / (n-1)`.

- **Kahan summation** is applied to both `μ` and each element of `M₂` to maintain precision over many updates.

- **Overflow handling**: When `_count == UINT16_MAX`, the current `_M2` is scaled down by `_count` (preserving covariance), then `_count` resets to 1. This avoids a 16‑bit counter overflow while keeping estimates valid.

## Limitations

- The counter is `uint16_t`; after ~65535 updates it will wrap, but the statistics are preserved. For applications needing exact sample counts beyond 65535, modify the code to use `uint32_t`.
- The class does not support weighted samples.
- All samples are assumed independent and identically distributed (i.i.d.).

## License

BSD 3‑Clause. Part of the PX4 project. See copyright header in the file.
```