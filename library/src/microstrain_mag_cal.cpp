#include <set>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include <microstrain_mag_cal.hpp>

#define M_PI   3.14159265358979323846 // π
#define M_PI_2 1.57079632679489661923 // π/2


namespace microstrain_mag_cal
{
    // ---------------------------------------------------------------------------------------------
    // Initial Parameter Estimation
    // ---------------------------------------------------------------------------------------------

    /// @brief Estimates the initial hard-iron offset using the mean of all measurements.
    ///
    /// Computes the centroid of all magnetometer measurements as an initial estimate. This provides
    /// a reasonable starting point for iterative calibration algorithms.
    ///
    /// @param points Nx3 matrix of raw magnetometer measurements (mx, my, mz).
    ///
    /// @returns 1x3 row vector containing the estimated hard-iron offset [bx, by, bz] in the same
    ///          units as the input measurements.
    ///
    Eigen::RowVector3d estimateInitialHardIronOffset(const Eigen::MatrixX3d &points)
    {
        if (points.rows() == 0)
        {
            return Eigen::RowVector3d::Zero();
        }

        return points.colwise().mean();
    }

    // ---------------------------------------------------------------------------------------------
    // Data Statistics
    // ---------------------------------------------------------------------------------------------

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
    /// @param initial_offset 1x3 row vector of the estimated initial hard iron offset (bx, by, bz).
    ///
    /// @returns Mean measured field strength in the same unit as the input measurements.
    ///
    double calculateMeanMeasuredFieldStrength(const Eigen::MatrixX3d &points, const Eigen::RowVector3d &initial_offset)
    {
        if (points.size() == 0)
        {
            return 0.0;
        }

        return (points.rowwise() - initial_offset).rowwise().norm().mean();
    }

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

    // ---------------------------------------------------------------------------------------------
    // Calibration Fitting
    // ---------------------------------------------------------------------------------------------

    // Each fitting algorithm requires a functor that's used by the Eigen solver. This base functor
    // contains the shared (required) definitions for Eigen.
    template<typename Derived, int NumParameters>
    struct FitFunctorBase
    {
        using Scalar = double;
        using InputType = Eigen::Vector<double, NumParameters>;
        using ValueType = Eigen::VectorXd;
        using JacobianType = Eigen::MatrixXd;

        static constexpr int InputsAtCompileTime = NumParameters;
        static constexpr int ValuesAtCompileTime = Eigen::Dynamic;

        const Eigen::MatrixX3d &points;
        const double target_radius;

        FitFunctorBase(const Eigen::MatrixX3d& points, const double field_strength)
            : points(points), target_radius(field_strength) {}

        int values() const { return static_cast<int>(points.rows()); }
        int inputs() const { return NumParameters; }

        // Common residual calculation --> delegates correction to each derived functor
        int operator()(const InputType& parameters, Eigen::VectorXd& residuals) const
        {
            for (int i = 0; i < points.rows(); ++i)
            {
                const Eigen::Vector3d corrected_point =
                    static_cast<const Derived *>(this)->applyCorrection(parameters, points.row(i).transpose());
                residuals(i) = corrected_point.norm() - target_radius;
            }

            return 0;
        }
    };

    // Returns a fit result that leaves the calibration unchanged (doesn't apply).
    FitResult noCalibrationApplied()
    {
        // Identity matrix and zero vector applies no change.
        return FitResult(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), false);
    }

    template<typename FunctorType>
    bool calculateFit(const Eigen::MatrixX3d &points, const double field_strength, Eigen::VectorXd &parameters)
    {
        constexpr int MAX_ITERATIONS = 1000;
        constexpr double TOLERANCE = 1.0e-10;

        // Mathematical minimum
        if (points.rows() < parameters.cols())
        {
            return false;
        }

        // Setup optimization
        const FunctorType functor(points, field_strength);
        Eigen::NumericalDiff<FunctorType> numerical_differentiator(functor);
        Eigen::LevenbergMarquardt<Eigen::NumericalDiff<FunctorType>> solver(numerical_differentiator);

        solver.parameters.maxfev = MAX_ITERATIONS;
        solver.parameters.xtol = TOLERANCE;

        // Optimize
        const Eigen::LevenbergMarquardtSpace::Status status = solver.minimize(parameters);

        // TODO: Combine
        // Check convergence
        const bool converged =
            status == Eigen::LevenbergMarquardtSpace::Status::RelativeErrorTooSmall ||
            status == Eigen::LevenbergMarquardtSpace::Status::RelativeReductionTooSmall;

        if (!converged)
        {
            return false;
        }

        return true;
    }

    struct SphericalFitFunctor : FitFunctorBase<SphericalFitFunctor, 4>
    {
        using Base = FitFunctorBase;
        using Base::Base;

        static Eigen::Vector3d applyCorrection(const Eigen::Vector4d &parameters, const Eigen::Vector3d &point)
        {
            const double scale = std::sqrt(parameters(0));
            const Eigen::Vector3d hard_iron_offset = parameters.tail<3>();

            return (point - hard_iron_offset) / scale;
        }
    };

    /// Calculates spherical calibration fit from raw magnetometer measurements.
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
    /// @param initial_offset 1x3 row vector of the estimated initial hard iron offset (bx, by, bz).
    ///
    /// @returns Fit result containing hard-iron offset, soft-iron scale factors, and whether the
    ///          fit succeeded. The units will be the same as the input data.
    ///
    FitResult calculateSphericalFit(
        const Eigen::MatrixX3d &points,
        const double field_strength,
        const Eigen::RowVector3d &initial_offset)
    {
        // Initialize parameters for the solver
        Eigen::VectorXd parameters(4);
        parameters(0) = 1.0;               // scale^2
        parameters.tail<3>() = initial_offset;  // hard iron offset

        // Optimize
        if (!calculateFit<SphericalFitFunctor>(points, field_strength, parameters))
        {
            return noCalibrationApplied();
        }

        // Extract spherical results
        const double scale = std::sqrt(parameters(0));
        const Eigen::Matrix<double, 3, 3> soft_iron_matrix = Eigen::Matrix<double, 3, 3>::Identity() * scale;
        const Eigen::Vector3d hard_iron_offset = parameters.tail<3>();

        return FitResult(soft_iron_matrix, hard_iron_offset, true);
    }

    // Upper and lower triangles are the same in a symmetric matrix!!
    inline Eigen::Matrix3d createSymmetricMatrixFromUpperTriangle(const Eigen::VectorXd& params, const int start_index = 0)
    {
        Eigen::Matrix3d matrix;
        matrix << params(start_index + 0), params(start_index + 1), params(start_index + 2),
                  params(start_index + 1), params(start_index + 3), params(start_index + 4),
                  params(start_index + 2), params(start_index + 4), params(start_index + 5);

        return matrix;
    }

    struct EllipsoidalFitFunctor : FitFunctorBase<EllipsoidalFitFunctor, 9>
    {
        using Base = FitFunctorBase;
        using Base::Base;

        static Eigen::Vector3d applyCorrection(const Eigen::Vector<double, 9> &parameters, const Eigen::Vector3d &point)
        {
            const Eigen::Matrix3d soft_iron_matrix = createSymmetricMatrixFromUpperTriangle(parameters);
            const Eigen::Vector3d hard_iron_offset = parameters.tail<3>();

            return soft_iron_matrix * (point - hard_iron_offset);
        }
    };

    /// Calculates ellipsoidal calibration fit from raw magnetometer measurements.
    ///
    /// Fits an ellipsoid to the measurement data to estimate both hard-iron offset (bias) and
    /// soft-iron effects (scale/rotation). Provides more accurate calibration than spherical fit
    /// when ferromagnetic materials cause field distortion.
    ///
    /// @param points Nx3 matrix of raw magnetometer measurements (mx, my, mz).
    /// @param field_strength The field strength to use for the target radius. Use the reference
    ///                       field strength if possible. Only use the measured field strength if
    ///                       the reference is unknown.
    /// @param initial_offset 1x3 row vector of the estimated initial hard iron offset (bx, by, bz).
    ///
    /// @returns Fit result containing hard-iron offset, full soft-iron matrix, and whether the
    ///          fit succeeded. The units will be the same as the input data.
    ///
    FitResult calculateEllipsoidalFit(
        const Eigen::MatrixX3d &points,
        const double field_strength,
        const Eigen::RowVector3d &initial_offset)
    {
        // Mathematical minimum since we are optimizing nine parameters:
        // * Six for symmetric matrix
        // * Three offset parameters (bx, by, bz)

        // Initialize parameters for the solver
        Eigen::VectorXd parameters(9);
        parameters.head<6>() << 1.0, 0.0, 0.0,  // Initialize soft-iron as identity matrix. Using
                                     1.0, 0.0,  // the upper triangle for optimization.
                                          1.0;
        parameters.tail<3>() = initial_offset;

        // Optimize
        if (!calculateFit<EllipsoidalFitFunctor>(points, field_strength, parameters))
        {
            return noCalibrationApplied();
        }

        // Extract ellipsoidal results
        const Eigen::Matrix3d soft_iron_matrix = createSymmetricMatrixFromUpperTriangle(parameters);
        const Eigen::Vector3d hard_iron_offset = parameters.tail<3>();

        return FitResult(soft_iron_matrix, hard_iron_offset, true);
    }

    // ---------------------------------------------------------------------------------------------
    // Fit Quality metrics
    // ---------------------------------------------------------------------------------------------

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
