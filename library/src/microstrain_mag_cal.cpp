#include <set>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include <microstrain_mag_cal.hpp>

#define M_PI   3.14159265358979323846 // π
#define M_PI_2 1.57079632679489661923 // π/2


namespace microstrain_mag_cal
{
    double calculate_measured_field_strength(const Eigen::MatrixX3d &points)
    {
        if (points.size() == 0)
        {
            return 0.0;
        }

        // TODO: Move to separate internal function
        // -------------------------------
        const Eigen::RowVector3d field_offset = points.colwise().mean();
        // -------------------------------

        return (points.rowwise() - field_offset).rowwise().norm().mean();
    }

    double calculate_spatial_coverage(const Eigen::MatrixX3d &points)
    {
        if (points.size() == 0)
        {
            return 0.0;
        }

        constexpr int num_latitude_bins = 18;
        constexpr int num_longitude_bins = 32;
        // TODO: Move to separate internal function
        const Eigen::RowVector3d field_offset = points.colwise().mean();

        std::set<std::pair<int, int>> occupied_bins;

        for (int i = 0; i < points.rows(); ++i) {
            Eigen::Vector3d point = points.row(i) - field_offset;

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
            // Map angle ranges to [0, 1], then scale to bin count.
            int point_latitude_bin = static_cast<int>((latitude + M_PI_2) / M_PI * num_latitude_bins);
            int point_longitude_bin = static_cast<int>((longitude + M_PI) / (2.0 * M_PI) * num_longitude_bins);

            // TODO: Make comment clearer
            // Clamp to valid bin range in case edge coordinates fall out of the index range.
            point_latitude_bin = std::max(0, std::min(point_latitude_bin, num_latitude_bins - 1));
            point_longitude_bin = std::max(0, std::min(point_longitude_bin, num_longitude_bins - 1));

            occupied_bins.insert({point_latitude_bin, point_longitude_bin});
        }

        // S = (occupied_bins / total_bins)
        // S% = S * 100
        return 100.0 * occupied_bins.size() / (num_latitude_bins * num_longitude_bins);
    }

    // Returns a fit result that leaves the calibration unchanged (doesn't apply).
    FitResult no_calibration_applied()
    {
        // Identity matrix and zero vector applies no change
        return FitResult(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), false);
    }

    struct SphericalFitFunctor
    {
        // Required type definitions for Eigen
        using Scalar = double;
        using InputType = Eigen::Vector4d;
        using ValueType = Eigen::VectorXd;
        using JacobianType = Eigen::MatrixXd;

        enum {
            InputsAtCompileTime = 4,              // Number of parameters
            ValuesAtCompileTime = Eigen::Dynamic  // Number of residuals (dynamic)
        };

        const Eigen::MatrixX3d &points;
        const double target_radius;

        SphericalFitFunctor(const Eigen::MatrixX3d& points, const double field_strength)
            : points(points), target_radius(field_strength) {}

        // Required by Eigen ---> Number of residuals (one per point)
        int values() const { return static_cast<int>(points.rows()); }

        // Required by Eigen ---> Number of parameters to optimize (scale^2, offset_x, offset_y, offset_z)
        int inputs() const { return 4; }

        // Required by Eigen ---> Computes residual for each point
        int operator()(const Eigen::Vector4d& parameters, Eigen::VectorXd& residuals) const
        {
            const double scale = std::sqrt(parameters(0));
            const Eigen::Vector3d offset = parameters.tail<3>();

            for (int i = 0; i < points.rows(); ++i)
            {
                const Eigen::Vector3d corrected_point = (points.row(i).transpose() - offset) / scale;
                residuals(i) = corrected_point.norm() - target_radius;
            }

            return 0;
        }
    };

    FitResult calculate_spherical_fit(const Eigen::MatrixX3d &points, const double field_strength)
    {
        constexpr int MAX_ITERATIONS = 1000;
        constexpr double TOLERANCE = 1.0e-10;

        // Mathematical minimum since we are optimizing four parameters:
        // * One scale parameter
        // * Three offset parameters (x, y, z)
        if (points.rows() < 4)
        {
            return no_calibration_applied();
        }

        // Initialize parameters for solver
        Eigen::VectorXd parameters(4);
        parameters(0) = 1.0;                       // Initial scale^2;
        // TODO: Move field offset calculation to separate internal function
        parameters.tail<3>() = points.colwise().mean(); // Initial offset (offset_x, offset_y, offset_z)

        // Setup optimization
        const SphericalFitFunctor functor(points, field_strength);
        Eigen::NumericalDiff<SphericalFitFunctor> numerical_differentiator(functor);
        Eigen::LevenbergMarquardt<Eigen::NumericalDiff<SphericalFitFunctor>> solver(numerical_differentiator);

        solver.parameters.maxfev = MAX_ITERATIONS;
        solver.parameters.xtol = TOLERANCE;

        // Optimize
        const Eigen::LevenbergMarquardtSpace::Status status = solver.minimize(parameters);

        // Check convergence
        const bool converged =
            status == Eigen::LevenbergMarquardtSpace::Status::RelativeErrorTooSmall ||
            status == Eigen::LevenbergMarquardtSpace::Status::RelativeReductionTooSmall;

        if (!converged)
        {
            return no_calibration_applied();
        }

        // Extract results
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

    struct EllipsoidalFitFunctor
    {
        // Required type definitions for Eigen
        using Scalar = double;
        using InputType = Eigen::Vector<double, 9>;
        using ValueType = Eigen::VectorXd;
        using JacobianType = Eigen::MatrixXd;

        enum {
            InputsAtCompileTime = 9,              // Number of parameters
            ValuesAtCompileTime = Eigen::Dynamic  // Number of residuals (dynamic)
        };

        const Eigen::MatrixX3d &points;
        const double target_radius;

        EllipsoidalFitFunctor(const Eigen::MatrixX3d& points, const double field_strength)
            : points(points), target_radius(field_strength) {}

        // Required by Eigen ---> Number of residuals (one per point)
        int values() const { return static_cast<int>(points.rows()); }

        // Required by Eigen ---> Number of parameters to optimize
        // (upper triangle of soft iron matrix, offset_x, offset_y, offset_z)
        int inputs() const { return 9; }

        // Required by Eigen ---> Direct parameterization: soft iron matrix (symmetric) + offset
        int operator()(const Eigen::VectorXd& parameters, Eigen::VectorXd& residuals) const
        {
            const Eigen::Matrix3d soft_iron_matrix = createSymmetricMatrixFromUpperTriangle(parameters);
            const Eigen::Vector3d offset = parameters.tail<3>();

            for (int i = 0; i < points.rows(); ++i)
            {
                const Eigen::Vector3d corrected_point = soft_iron_matrix * (points.row(i).transpose() - offset); // / scale;
                residuals(i) = corrected_point.norm() - target_radius;
            }

            return 0;
        }
    };

    FitResult calculate_ellipsoidal_fit(const Eigen::MatrixX3d &points, double field_strength)
    {
        constexpr int MAX_ITERATIONS = 1000;
        constexpr double TOLERANCE = 1.0e-10;

        // Mathematical minimum since we are optimizing nine parameters:
        // * Six for symmetric matrix
        // * Three offset parameters (x, y, z)
        if (points.rows() < 9)
        {
            return no_calibration_applied();
        }

        // Initialize parameters for solver
        Eigen::VectorXd parameters(9);
        parameters.head<6>() << 1.0, 0.0, 0.0,  // Initialize soft-iron as identity matrix. Using
                                     1.0, 0.0,  // the upper triangle for optimization.
                                          1.0;
        // TODO: Move field offset calculation to separate internal function
        parameters.tail<3>() = points.colwise().mean(); // Initial offset (offset_x, offset_y, offset_z)

        // Setup optimization
        const EllipsoidalFitFunctor functor(points, field_strength);
        Eigen::NumericalDiff<EllipsoidalFitFunctor> numerical_differentiator(functor);
        Eigen::LevenbergMarquardt<Eigen::NumericalDiff<EllipsoidalFitFunctor>> solver(numerical_differentiator);

        solver.parameters.maxfev = MAX_ITERATIONS;
        solver.parameters.xtol = TOLERANCE;

        // Optimize
        const Eigen::LevenbergMarquardtSpace::Status status = solver.minimize(parameters);

        // Check convergence
        const bool converged =
            status == Eigen::LevenbergMarquardtSpace::Status::RelativeErrorTooSmall ||
            status == Eigen::LevenbergMarquardtSpace::Status::RelativeReductionTooSmall;

        if (!converged)
        {
            return no_calibration_applied();
        }

        // Extract results
        const Eigen::Matrix3d soft_iron_matrix = createSymmetricMatrixFromUpperTriangle(parameters);
        const Eigen::Vector3d hard_iron_offset = parameters.tail<3>();

        return FitResult(soft_iron_matrix, hard_iron_offset, true);
    }

    double calculateFitRMSE(
        const Eigen::MatrixX3d &points,
        const Eigen::Matrix<double, 3, 3> &soft_iron_matrix,
        const Eigen::Vector3d &hard_iron_offset,
        const double field_strength)
    {
        const double field_strength_squared = field_strength * field_strength;

        // Precompute A
        const Eigen::Matrix3d A = soft_iron_matrix.transpose() * soft_iron_matrix;

        // Compute compensated magnetometer readings
        const Eigen::MatrixX3d centered = points.rowwise() - hard_iron_offset.transpose();

        // For each point: compute (p - bias)^T * A * (p - bias)
        const Eigen::VectorXd quadratic_forms = (centered * A).cwiseProduct(centered).rowwise().sum();

        // Compute residuals
        const Eigen::VectorXd residuals = Eigen::VectorXd::Constant(points.rows(), field_strength_squared) - quadratic_forms;

        // Sum of squared residuals
        const double sum_squared_residuals = residuals.array().square().sum();

        return std::sqrt(sum_squared_residuals / points.rows());
    }
}
