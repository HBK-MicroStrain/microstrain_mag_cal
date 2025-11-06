#pragma once

#include <Eigen/Dense>


namespace microstrain_mag_cal
{
    struct FitResult
    {
        FitResult(
            Eigen::Matrix<double, 3, 3> soft_iron_matrix,
            Eigen::Vector3d hard_iron_offset,
            const bool valid) :
                soft_iron_matrix(std::move(soft_iron_matrix)),
                hard_iron_offset(std::move(hard_iron_offset)),
                valid(valid) {}

        const Eigen::Matrix<double, 3, 3> soft_iron_matrix;
        const Eigen::Vector3d hard_iron_offset;
        const bool valid;
    };

    /// Calculates the mean measured field strength from raw magnetometer measurements.
    ///
    /// Computes the magnitude (Euclidean norm) of each raw measurement vector and returns the
    /// average magnitude across all samples.
    ///
    /// NOTE: This represents the average field strength as measured by the uncalibrated sensor,
    ///       which will differ from the true geomagnetic field strength due to hard-iron and
    ///       soft-iron distortions.
    ///
    /// @param points Nx3 matrix of raw magnetometer measurements (mx, my, mz).
    ///
    /// @returns Mean measured field strength in the same unit as the input measurements.
    ///
    // TODO: Rename to calculateMeanMeasuredFieldStrength()
    double calculate_measured_field_strength(const Eigen::MatrixX3d &points);

    /// Calculates the spatial coverage percentage from raw magnetometer measurements.
    ///
    /// Measures how uniformly the measurement directions are distributed in 3D space.
    /// Good spatial coverage (>70%) is necessary for accurate calibration.
    ///
    /// @param points Nx3 matrix of raw magnetometer measurements (mx, my, mz).
    ///
    /// @returns Coverage percentage (0-100%), where higher values indicate better distribution.
    ///
    double calculate_spatial_coverage(const Eigen::MatrixX3d &points);

    /// Performs spherical calibration fit on raw magnetometer measurements.
    ///
    /// Fits a sphere to the measurement data to estimate hard-iron offset (bias) and axis-aligned
    /// soft-iron scaling factors. Assumes distortions are symmetric along sensor axes. The soft
    /// iron matrix will be diagonal.
    ///
    /// Suitable for sensors with minimal soft-iron effects or axis-aligned distortions. Use
    /// ellipsoidal fit for more complex soft-iron effects involving rotation or shear.
    ///
    /// @param points Nx3 matrix of raw magnetometer measurements (mx, my, mz).
    /// @param field_strength The field strength to use for the target radius. Use the reference
    ///                       field strength if possible. Only use the measured field strength if
    ///                       the reference is unknown.
    ///
    /// @returns Fit result containing hard-iron offset, soft-iron scale factors, and whether the
    ///          fit succeeded. The units will be the same as the input data.
    ///
    FitResult calculate_spherical_fit(const Eigen::MatrixX3d &points, double field_strength);

    /// Performs ellipsoidal calibration fit on raw magnetometer measurements.
    ///
    /// Fits an ellipsoid to the measurement data to estimate both hard-iron offset (bias) and
    /// soft-iron effects (scale/rotation). Provides more accurate calibration than spherical fit
    /// when ferromagnetic materials cause field distortion.
    ///
    /// @param points Nx3 matrix of raw magnetometer measurements (mx, my, mz).
    /// @param field_strength The field strength to use for the target radius. Use the reference
    ///                       field strength if possible. Only use the measured field strength if
    ///                       the reference is unknown.
    ///
    /// @returns Fit result containing hard-iron offset, full soft-iron matrix, and whether the
    ///          fit succeeded. The units will be the same as the input data.
    ///
    FitResult calculate_ellipsoidal_fit(const Eigen::MatrixX3d &points, double field_strength);

    /// Calculates the root mean square error (RMSE) for a calibration fit.
    ///
    /// Computes the average deviation of calibrated measurements from the expected field strength.
    /// Lower RMSE indicates a better fit quality.
    ///
    /// @param points Nx3 matrix of raw magnetometer measurements (mx, my, mz).
    /// @param soft_iron_matrix 3x3 soft-iron correction matrix from the fit
    /// @param hard_iron_offset 3x1 hard-iron offset vector from the fit
    /// @param field_strength The field strength to use for the target radius. Use the reference
    ///                       field strength if possible. Only use the measured field strength if
    ///                       the reference is unknown.
    ///
    /// @returns RMSE in the same units as the input data.
    ///
    double calculateFitRMSE(
        const Eigen::MatrixX3d &points,
        const Eigen::Matrix<double, 3, 3> &soft_iron_matrix,
        const Eigen::Vector3d &hard_iron_offset,
        double field_strength);
}