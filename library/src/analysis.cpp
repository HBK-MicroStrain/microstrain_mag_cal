#include "analysis.hpp"

#include <set>

#define M_PI   3.14159265358979323846 // π
#define M_PI_2 1.57079632679489661923 // π/2


namespace microstrain_mag_cal
{
    /// Calculates the spatial coverage percentage from raw magnetometer measurements.
    ///
    /// Measures how uniformly the measurement directions are distributed in 3D space.
    /// Good spatial coverage (>70%) is necessary for accurate calibration.
    ///
    /// @param points Nx3 matrix of raw magnetometer measurements (mx, my, mz).
    /// @param initial_offset 1x3 row vector of the estimated initial hard iron offset (bx, by, bz).
    ///
    /// @returns Coverage percentage (0-100%), where higher values indicate better distribution.
    ///
    double calculateSpatialCoverage(const Eigen::MatrixX3d &points, const Eigen::RowVector3d &initial_offset)
    {
        // This algorithm uses the geography convention for coverage.

        if (points.size() == 0)
        {
            return 0.0;
        }

        constexpr int num_latitude_bins = 18;
        constexpr int num_longitude_bins = 32;

        std::set<std::pair<int, int>> occupied_bins;

        for (int i = 0; i < points.rows(); ++i) {
            Eigen::Vector3d point = points.row(i) - initial_offset;

            // Normalizing to the unit sphere here because we only care about directional
            // information, not magnitude information.
            point.normalize();

            const double x = point(0);
            const double y = point(1);
            const double z = point(2);

            // Convert to spherical coordinates
            const double latitude = std::asin(z);      // -π/2 to π/2 radians
            const double longitude = std::atan2(y, x); // -π to π radians

            // Assign to bins.
            // Map angle ranges to [0, 1], then scale to bin count to get the bin indices.
            int point_latitude_bin = static_cast<int>((latitude + M_PI_2) / M_PI * num_latitude_bins);
            int point_longitude_bin = static_cast<int>((longitude + M_PI) / (2.0 * M_PI) * num_longitude_bins);

            // Clamp bin indices to valid range [0, num_bins - 1] in case floating-point rounding
            // causes coordinates at bin boundaries to produce out-of-range indices.
            point_latitude_bin = std::max(0, std::min(point_latitude_bin, num_latitude_bins - 1));
            point_longitude_bin = std::max(0, std::min(point_longitude_bin, num_longitude_bins - 1));

            occupied_bins.insert({point_latitude_bin, point_longitude_bin});
        }

        // S = (occupied_bins / total_bins)
        // S% = S * 100
        return 100.0 * occupied_bins.size() / (num_latitude_bins * num_longitude_bins);
    }

    /// Calculates the root mean square error (RMSE) for a calibration fit.
    ///
    /// Computes the average deviation of calibrated measurements from the expected field strength.
    /// Lower RMSE indicates a better fit quality.
    ///
    /// @param points Nx3 matrix of raw magnetometer measurements (mx, my, mz).
    /// @param fit_result Calibration fit result containing soft-iron matrix and hard-iron offset.
    /// @param field_strength The field strength to use for the target radius. Use the reference
    ///                       field strength if possible. Only use the measured field strength if
    ///                       the reference is unknown.
    ///
    /// @returns RMSE in the same units as the input data.
    ///
    double calculateFitRMSE(const Eigen::MatrixX3d &points, const FitResult &fit_result, const double field_strength)
    {
        const double field_strength_squared = field_strength * field_strength;

        // Precompute A
        const Eigen::Matrix3d A = fit_result.soft_iron_matrix.transpose() * fit_result.soft_iron_matrix;

        // Compute compensated magnetometer readings
        const Eigen::MatrixX3d centered = points.rowwise() - fit_result.hard_iron_offset.transpose();

        // For each point: compute (p - bias)^T * A * (p - bias)
        const Eigen::VectorXd quadratic_forms = (centered * A).cwiseProduct(centered).rowwise().sum();

        // Compute residuals
        const Eigen::VectorXd residuals = Eigen::VectorXd::Constant(points.rows(), field_strength_squared) - quadratic_forms;

        // Sum of squared residuals
        const double sum_squared_residuals = residuals.array().square().sum();

        return std::sqrt(sum_squared_residuals / points.rows());
    }
}
