# World Calibration — Math & Implementation Notes

> **Package**: `dexim-manus` (`dexim.manus.calibration`)
> **Date**: 2026-03-13

---

## Overview

World calibration maps tracker-frame coordinates (raw Manus glove positions) into a
user-defined world frame using a single 4×4 rigid-body transform. After calibration every
position streamed by the glove tracker can be expressed in a consistent, robot-agnostic
coordinate system.

---

## 1. Three-Point Registration (`calibrate_world`)

Three physical positions are captured in tracker space:

| Point     | Role                                     |
| --------- | ---------------------------------------- |
| `origin`  | World-frame origin                       |
| `x_point` | A point on the +X axis                   |
| `y_point` | A point on the +Y axis (in the XY plane) |

The distances between these points define the scale of the world axes — no fixed ruler
measurement is required:

$$d_x = \|p_x - p_0\|, \quad d_y = \|p_y - p_0\|$$

The corresponding **reference** (world-frame) point set is then:

$$\mathbf{R}_\text{ref} = \begin{bmatrix} 0 & 0 & 0 \\ d_x & 0 & 0 \\ 0 & d_y & 0 \end{bmatrix}$$

and the **measured** (tracker-frame) point set is:

$$\mathbf{M}_\text{meas} = \begin{bmatrix} p_0 \\ p_x \\ p_y \end{bmatrix}$$

The Kabsch / SVD algorithm (`_rigid_transform`) finds the rotation $\mathbf{R}$ and
translation $\mathbf{t}$ that minimise:

$$\min_{\mathbf{R},\mathbf{t}} \sum_{i=1}^{3} \left\| \mathbf{r}_i - \left(\mathbf{R}\,\mathbf{m}_i + \mathbf{t}\right) \right\|^2$$

The result is packed into a 4×4 homogeneous transform:

$$\mathbf{T}_{w \leftarrow b} = \begin{bmatrix} \mathbf{R} & \mathbf{t} \\ \mathbf{0}^\top & 1 \end{bmatrix}$$

which maps any tracker-frame point $\tilde{\mathbf{m}}$ to world frame:

$$\mathbf{p}_\text{world} = \mathbf{T}_{w \leftarrow b}\, \tilde{\mathbf{m}}$$

### Reflection correction

If the three capture points produce a left-handed basis (det $\mathbf{R} < 0$), the SVD
solution is a reflection, not a rotation. The algorithm detects this and applies a sign
flip on the last singular vector to recover a proper rotation.

---

## 2. Reprojection Error (`_reprojection_error_mm`)

After obtaining `wM_base`, calibration quality is quantified by re-projecting the
original tracker measurements into world space and comparing them to the known reference:

$$\varepsilon = \frac{1}{N} \sum_{i=1}^{N} \left\| \mathbf{T}_{w \leftarrow b}\, \tilde{\mathbf{m}}_i - \mathbf{r}_i \right\| \times 1000$$

where the factor $\times 1000$ converts metres to millimetres (Manus positions are in SI
units).

**Argument contract:**

| Argument     | Frame   | Description                   |
| ------------ | ------- | ----------------------------- |
| `wM_base`    | —       | 4×4 tracker → world transform |
| `ref_array`  | world   | $(N, 3)$ reference points     |
| `meas_array` | tracker | $(N, 3)$ measured points      |

A perfect calibration yields $\varepsilon = 0$. In practice, values below ~5 mm indicate
a good calibration for teleoperation use.

### Common mistakes (and why they fail)

| Mistake                                                                                         | Symptom                                                      |
| ----------------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| Applying `wM_base` to `ref_array` instead of `meas_array`                                       | Nonzero error even for a mathematically perfect calibration  |
| Passing unit-distance reference `[1,0,0]` / `[0,1,0]` when calibration points are not 1 m apart | Scaled offset in error; does not converge to 0               |
| Passing `ref == meas` in tests                                                                  | Masks argument-order bugs because symmetric distances cancel |

---

## 3. SVD-Based Rigid Transform (`_rigid_transform`)

Given $N$ point correspondences, compute centroids:

$$\bar{\mathbf{m}} = \frac{1}{N}\sum_i \mathbf{m}_i, \quad \bar{\mathbf{r}} = \frac{1}{N}\sum_i \mathbf{r}_i$$

Form the cross-covariance matrix:

$$\mathbf{H} = \sum_i (\mathbf{m}_i - \bar{\mathbf{m}})(\mathbf{r}_i - \bar{\mathbf{r}})^\top$$

Decompose $\mathbf{H} = \mathbf{U} \mathbf{\Sigma} \mathbf{V}^\top$, then:

$$\mathbf{R} = \mathbf{V} \mathbf{U}^\top$$

with a reflection correction if $\det(\mathbf{R}) < 0$:

$$\mathbf{S} = \operatorname{diag}(1, 1, \det(\mathbf{V}\mathbf{U}^\top)), \quad \mathbf{R} = \mathbf{V} \mathbf{S} \mathbf{U}^\top$$

Translation:

$$\mathbf{t} = \bar{\mathbf{r}} - \mathbf{R}\,\bar{\mathbf{m}}$$

The algorithm requires at least 2 non-collinear points (rank $\mathbf{H} > 1$). With 3
non-collinear points the solution is unique and exact.

---

## 4. File Format

Calibrations are saved as JSON:

```json
{
  "world_to_tracker": [[...], [...], [...], [...]],
  "timestamp": "20260312_185644",
  "error_mm": 1.23,
  "reference_points": {...},
  "measured_points": {...}
}
```

`load_world_calibration` picks the **lexicographically latest** file matching
`world_calibration_*.json` in the target directory (timestamps are zero-padded ISO-style
so lexicographic order equals chronological order).
