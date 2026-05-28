# Math Utilities for Rotations

This document describes the `Utilities.hpp` header, part of the PX4 math library. It provides a collection of efficient, gimbal‑lock‑aware functions for working with rotation matrices, quaternions, and Euler angles. The code is templated on floating‑point type (default `float`).

## Overview

The header solves common problems in 3D attitude representation:

- Converting between Tait‑Bryan rotation sequences (3‑1‑2 and 3‑2‑1) and rotation matrices.
- Extracting yaw angle from a rotation matrix or quaternion without risking gimbal lock.
- Updating a rotation matrix with a new yaw value while preserving roll/pitch (or recomputing them appropriately).

All functions are inlined and designed for real‑time embedded use (e.g., flight controllers).

## Rotation Conventions

Two Tait‑Bryan sequences are used:

| Sequence | Order of rotations          | Usage condition                     |
|----------|-----------------------------|-------------------------------------|
| 3‑2‑1    | Yaw (Z) → Pitch (Y) → Roll (X) | When `|R(2,0)| < |R(2,1)|` (more roll tilt) |
| 3‑1‑2    | Yaw (Z) → Roll (X) → Pitch (Y) | Otherwise (more pitch tilt)         |

The `shouldUse321RotationSequence()` function decides which sequence is numerically better to avoid singularities. The yaw extraction functions (`getEulerYaw`) and yaw update functions (`updateYawInRotMat`) automatically select the appropriate sequence.

## Function Reference

### Basic Utility

```cpp
static constexpr float sq(float var);
```
Returns `var * var`. Used internally for quaternion algebra.

### Conversions

#### `taitBryan312ToRotMat`

```cpp
template<typename T = float>
matrix::Dcm<T> taitBryan312ToRotMat(const matrix::Vector3<T> &rot312);
```
Converts a 3‑1‑2 Tait‑Bryan vector `(yaw, roll, pitch)` to a direction cosine matrix (DCM) that rotates vectors from frame 2 to frame 1.

#### `quatToInverseRotMat`

```cpp
template<typename T = float>
matrix::Dcm<T> quatToInverseRotMat(const matrix::Quaternion<T> &quat);
```
Converts a quaternion (stored as `w, x, y, z`) to the **inverse** rotation matrix (i.e., the transpose of the standard quaternion‑to‑DCM matrix).

### Yaw Extraction

| Function | Input | Sequence used |
|----------|-------|----------------|
| `getEuler321Yaw(R)` | DCM | 3‑2‑1 |
| `getEuler312Yaw(R)` | DCM | 3‑1‑2 |
| `getEuler321Yaw(q)` | Quaternion | 3‑2‑1 |
| `getEuler312Yaw(q)` | Quaternion | 3‑1‑2 |
| `getEulerYaw(R)` | DCM | Auto‑selected |
| `getEulerYaw(q)` | Quaternion | Auto‑selected |

All return yaw in radians (range `-π` to `π`).

### Yaw Update in a Rotation Matrix

Given a rotation matrix that represents a full attitude, replace only the yaw angle while keeping the effective tilt (roll & pitch) as close as possible to the original.

- `updateEuler321YawInRotMat(yaw, rot_in)` – uses 3‑2‑1 decomposition.
- `updateEuler312YawInRotMat(yaw, rot_in)` – uses 3‑1‑2 decomposition.
- `updateYawInRotMat(yaw, rot_in)` – automatically selects the best sequence.

## Why Two Sequences?

When pitch approaches ±90° (the “gimbal lock” region of 3‑2‑1 Euler angles), yaw becomes undefined. The 3‑1‑2 sequence avoids this singularity for the same attitude. By detecting which element of the DCM is larger (`R(2,0)` vs `R(2,1)`), the code switches to the sequence that gives a well‑conditioned yaw estimate.

## Example Usage

```cpp
#include "Utilities.hpp"
#include <matrix/math.hpp>

using namespace math::Utilities;

// Create a rotation matrix from a 3‑1‑2 Euler vector
matrix::Vector3f euler312(0.1f, 0.2f, 0.3f); // yaw, roll, pitch
matrix::Dcmf R = taitBryan312ToRotMat(euler312);

// Extract yaw (automatically chooses best sequence)
float yaw = getEulerYaw(R);

// Replace yaw with a new value while preserving orientation tilt
matrix::Dcmf R_new = updateYawInRotMat(1.5f, R);

// Convert quaternion to inverse DCM
matrix::Quatf q(1.0f, 0.0f, 0.0f, 0.0f); // identity
matrix::Dcmf R_inv = quatToInverseRotMat(q);
```

## Integration

- **Dependencies**: Requires the [`matrix`](https://github.com/PX4/matrix) math library (part of PX4).
- **License**: BSD 3‑Clause (see copyright header in the file).
- **Namespace**: All symbols are inside `math::Utilities`.

## Notes

- The functions are templated, but the provided implementations call `cosf`, `sinf`, `atan2f`, `asinf` – these are single‑precision. For `double`, the caller must ensure the correct `<cmath>` overloads are used (the code uses `f` variants explicitly).
- Input quaternions are assumed to be normalized. The code does not perform normalization internally.
- The yaw update functions only modify yaw; they do **not** preserve the exact original roll/pitch when switching sequences – the tilt is recomputed in the chosen Euler convention.

## References

- [Euler Sequence Documentation](http://www.atacolorado.com/eulersequences.doc)
- PX4 ECL Matlab scripts for quaternion‑to‑yaw conversion: [quat2yaw321.m](https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw321.m) and [quat2yaw312.m](https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw312.m)
```