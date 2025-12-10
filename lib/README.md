# Library User Guide

## Usage
The library is organized into three modules:

| Module                                                 | Purpose                                    |
|--------------------------------------------------------|--------------------------------------------|
| [Preprocessing](microstrain_mag_cal/preprocessing.hpp) | Filters and prepares data for calibration. |
| [Calibration](microstrain_mag_cal/calibration.hpp)     | Estimates calibration coefficients.        |
| [Analysis](microstrain_mag_cal/analysis.hpp)           | Analyzes data and evaluates fit quality.   |

### Example workflow
```C++
// Set up point manager for data filtering.
const microstrain_mag_cal::VoxelGrid unique_point_grid(voxel_size);
microstrain_mag_cal::PointManager point_manager(unique_point_grid);

// Filter the points
for (const std::array<float, 3> &point : points)
{
    point_manager.addPoint(point);
}

const Eigen::MatrixX3d points = point_manager.getMatrix();
const Eigen::RowVector3d initial_offset = microstrain_mag_cal::estimateInitialHardIronOffset(points);

// Analyze data quality
microstrain_mag_cal::calculateSpatialCoverage(points, initial_offset)

// Run a calibration fit
constexpr double field_strength = 0.52;
const microstrain_mag_cal::FitResult fit_result = microstrain_mag_cal::fitEllipsoid(points, field_strength, initial_offset);

// Evaluate the fit quality
const double fit_RMSE = microstrain_mag_cal::calculateFitRMSE(points, fit_result, field_strength);
```

### Fit functions
The currently supported fit functions are:

| Function        | Correction                                                                  | Coefficients calibrated                                                                              |
|-----------------|-----------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------|
| Spherical fit   | $\mathbf{m}\_\text{corr} = s \cdot (\mathbf{m}\_\text{raw} - \mathbf{b})$   | - Hard-iron offset<br/> - Uniform scale factor                                                       |
| Ellipsoidal fit | $\mathbf{m}\_\text{corr} = \mathbf{A}(\mathbf{m}\_\text{raw} - \mathbf{b})$ | - Hard-iron offset<br/> - Non-uniform scale factor<br/>- Symmetric (non-uniform) cross-axis coupling |

where,

$$
\begin{aligned}
\mathbf{m}_\text{cal} &= \text{Corrected measurement vector} \\
\mathbf{m}_\text{raw} &= \text{Raw measurement vector} \\
\mathbf{b} &= \text{Hard-iron offset vector} \\
s &= \text{Uniform scale factor} \\
\mathbf{A} &= \text{Soft-iron matrix}
\end{aligned}
$$
