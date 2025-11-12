#include "calibration.hpp"

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>


namespace microstrain_mag_cal
{
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
    bool optimizeFit(const Eigen::MatrixX3d &points, const double field_strength, Eigen::VectorXd &parameters)
    {
        constexpr int MAX_ITERATIONS = 1000;
        constexpr double TOLERANCE = 1.0e-10;

        // Mathematical minimum for optimizing N parameters
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

        // Fail if the optimization didn't converge
        if (status != Eigen::LevenbergMarquardtSpace::Status::RelativeErrorTooSmall &&
            status != Eigen::LevenbergMarquardtSpace::Status::RelativeReductionTooSmall)
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

    /// @brief Performs a spherical fit on the raw magnetometer data.
    ///
    /// Fits a sphere to magnetometer data for hard-iron calibration and uniform field scaling.
    /// Estimates hard-iron offset (bias) and a single scale factor applied uniformly to all axes.
    /// The soft-iron matrix will be a scaled identity matrix.
    ///
    /// Suitable for sensors with minimal soft-iron effects or when only uniform scaling correction
    /// is needed. Use ellipsoidal fit for more complex soft-iron effects involving axis-dependent
    /// scaling, rotation or shear.
    ///
    /// @param points Nx3 matrix of raw magnetometer measurements (mx, my, mz).
    /// @param field_strength The field strength to use for the target radius. Use the reference
    ///                       field strength if possible. Only use the measured field strength if
    ///                       the reference is unknown.
    /// @param initial_offset 1x3 row vector of the estimated initial hard iron offset (bx, by, bz).
    ///
    /// @returns Fit result containing hard-iron offset, uniform soft-iron scale factor, and whether
    ///          the fit succeeded. The units will be the same as the input data.
    ///
    FitResult fitSphere(
        const Eigen::MatrixX3d &points,
        const double field_strength,
        const Eigen::RowVector3d &initial_offset)
    {
        Eigen::VectorXd fit_parameters(4);
        fit_parameters(0) = 1.0;               // Scale^2
        fit_parameters.tail<3>() = initial_offset;  // Hard iron offset

        if (!optimizeFit<SphericalFitFunctor>(points, field_strength, fit_parameters))
        {
            return noCalibrationApplied();
        }

        const double scale = std::sqrt(fit_parameters(0));
        const Eigen::Matrix<double, 3, 3> soft_iron_matrix = Eigen::Matrix<double, 3, 3>::Identity() * scale;
        const Eigen::Vector3d hard_iron_offset = fit_parameters.tail<3>();

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

    /// @brief Performs an ellipsoidal fit on the raw magnetometer data.
    ///
    /// Fits an ellipsoid to magnetometer data to estimate both hard-iron offset (bias) and soft-iron
    /// effects (scaling, rotation, and shear). Provides more accurate calibration than spherical fit
    /// when ferromagnetic materials cause axis-dependent field distortion. The soft-iron matrix is
    /// symmetric (6 unique parameters).
    ///
    /// @param points Nx3 matrix of raw magnetometer measurements (mx, my, mz).
    /// @param field_strength The field strength to use for the target radius. Use the reference
    ///                       field strength if possible. Only use the measured field strength if
    ///                       the reference is unknown.
    /// @param initial_offset 1x3 row vector of the estimated initial hard iron offset (bx, by, bz).
    ///
    /// @returns Fit result containing hard-iron offset, full symmetric soft-iron matrix, and whether
    ///          the fit succeeded. The units will be the same as the input data.
    ///
    FitResult fitEllipsoid(
        const Eigen::MatrixX3d &points,
        const double field_strength,
        const Eigen::RowVector3d &initial_offset)
    {
        Eigen::VectorXd fit_parameters(9);
        fit_parameters.head<6>() << 1.0, 0.0, 0.0,  // Initialize soft-iron as identity matrix. Using
                                         1.0, 0.0,  // the upper triangle for optimization.
                                              1.0;
        fit_parameters.tail<3>() = initial_offset;  // Hard iron offset

        if (!optimizeFit<EllipsoidalFitFunctor>(points, field_strength, fit_parameters))
        {
            return noCalibrationApplied();
        }

        const Eigen::Matrix3d soft_iron_matrix = createSymmetricMatrixFromUpperTriangle(fit_parameters);
        const Eigen::Vector3d hard_iron_offset = fit_parameters.tail<3>();

        return FitResult(soft_iron_matrix, hard_iron_offset, true);
    }
}
