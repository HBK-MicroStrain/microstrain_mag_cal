#include "calibration.hpp"

#include <cassert>
#include <fstream>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>


namespace microstrain_mag_cal
{
    bool operator==(const FitResult& lhs, const FitResult& rhs)
    {
        return lhs.error == rhs.error &&
               lhs.hard_iron_offset.isApprox(rhs.hard_iron_offset, 0.001) &&
               lhs.soft_iron_matrix.isApprox(rhs.soft_iron_matrix, 0.001);
    }

    std::ostream& operator<<(std::ostream& os, const FitResult& result)
    {
        os << "FitResult:\n"
           << "  Error: " << magic_enum::enum_name(result.error) << "\n"
           << "  Hard Iron Offset: " << "[" << result.hard_iron_offset << "]\n"
           << "  Soft Iron Matrix:\n" << "[" << result.soft_iron_matrix << "]\n";

        return os;
    }

    /// @brief Returns a fit result that leaves the calibration unchanged (doesn't apply).
    FitResult FitResult::noCorrection(const Error &error)
    {
        return {Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), error};
    }


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

        [[nodiscard]] int values() const { return static_cast<int>(points.rows()); }
        static int inputs() { return NumParameters; }

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

    bool verifyMatrixIsPositiveDefinite(const Eigen::Matrix3d &matrix)
    {
        // This indicates insufficient data coverage or a bug in the fitting algorithm
        if ((Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(matrix).eigenvalues().array() <= 0).any())
        {
            assert(false && "Matrix is not positive definite");
            return false;
        }

        return true;
    }

    template<typename FunctorType>
    FitResult::Error optimizeFit(const Eigen::MatrixX3d &points, const double field_strength, Eigen::VectorXd &parameters)
    {
        constexpr int MAX_ITERATIONS = 1000;
        constexpr double TOLERANCE = 1.0e-10;

        // Mathematical minimum for optimizing N parameters
        if (points.rows() < parameters.cols())
        {
            return FitResult::Error::FIT_OPTIMIZATION_INSUFFICIENT_INPUT_DATA;
        }

        // Setup optimization
        const FunctorType functor(points, field_strength);
        Eigen::NumericalDiff<FunctorType> numerical_differentiator(functor);
        Eigen::LevenbergMarquardt<Eigen::NumericalDiff<FunctorType>> solver(numerical_differentiator);

        solver.parameters.maxfev = MAX_ITERATIONS;
        solver.parameters.xtol = TOLERANCE;

        // Optimize
        if (const Eigen::LevenbergMarquardtSpace::Status status = solver.minimize(parameters);
            status != Eigen::LevenbergMarquardtSpace::Status::RelativeErrorTooSmall &&
            status != Eigen::LevenbergMarquardtSpace::Status::RelativeReductionTooSmall)
        {
            return FitResult::Error::FIT_OPTIMIZATION_DID_NOT_CONVERGE;
        }

        return FitResult::Error::NONE;
    }

    struct SphericalFitFunctor : FitFunctorBase<SphericalFitFunctor, 4>
    {
        using Base = FitFunctorBase;
        using Base::Base;

        static Eigen::Vector3d applyCorrection(const Eigen::Vector4d &parameters, const Eigen::Vector3d &point)
        {
            const double scale = std::sqrt(parameters(0));
            const Eigen::Vector3d hard_iron_offset = parameters.tail<3>();

            return (point - hard_iron_offset) * scale;
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
    /// TODO: Document that this returns the correction, not distortion.
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

        if (const FitResult::Error error = optimizeFit<SphericalFitFunctor>(points, field_strength, fit_parameters);
            error != FitResult::Error::NONE)
        {
            return FitResult::noCorrection(error);
        }

        const double scale = std::sqrt(fit_parameters(0));
        const Eigen::Matrix3d soft_iron_matrix = Eigen::Matrix3d::Identity() * scale;
        const Eigen::Vector3d hard_iron_offset = fit_parameters.tail<3>();

        if (!verifyMatrixIsPositiveDefinite(soft_iron_matrix))
        {
            return FitResult::noCorrection(FitResult::Error::FIT_CORRECTION_MATRIX_NOT_POSITIVE_DEFINITE);
        }

        return {soft_iron_matrix, hard_iron_offset};
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

        if (const FitResult::Error error = optimizeFit<EllipsoidalFitFunctor>(points, field_strength, fit_parameters);
            error != FitResult::Error::NONE)
        {
            return FitResult::noCorrection(error);
        }

        const Eigen::Matrix3d soft_iron_matrix = createSymmetricMatrixFromUpperTriangle(fit_parameters);
        const Eigen::Vector3d hard_iron_offset = fit_parameters.tail<3>();

        if (!verifyMatrixIsPositiveDefinite(soft_iron_matrix))
        {
            return FitResult::noCorrection(FitResult::Error::FIT_CORRECTION_MATRIX_NOT_POSITIVE_DEFINITE);
        }

        return {soft_iron_matrix, hard_iron_offset};
    }

    /// @brief Serializes the fit result to a json object.
    nlohmann::json serializeFitResult(const FitResult &fit_result)
    {
        nlohmann::json output;

        output["error"] = fit_result.error;

        output["softIronMatrix"] = {
            {"xx", fit_result.soft_iron_matrix(0, 0)},
            {"xy", fit_result.soft_iron_matrix(0, 1)},
            {"xz", fit_result.soft_iron_matrix(0, 2)},
            {"yx", fit_result.soft_iron_matrix(1, 0)},
            {"yy", fit_result.soft_iron_matrix(1, 1)},
            {"yz", fit_result.soft_iron_matrix(1, 2)},
            {"zx", fit_result.soft_iron_matrix(2, 0)},
            {"zy", fit_result.soft_iron_matrix(2, 1)},
            {"zz", fit_result.soft_iron_matrix(2, 2)},
        };

        output["hardIronOffset"] = {
            {"x", fit_result.hard_iron_offset(0)},
            {"y", fit_result.hard_iron_offset(1)},
            {"z", fit_result.hard_iron_offset(2)},
        };

        return output;
    }

    /// @brief Serializes the fit result to json and writes it to the given file.
    void serializeFitResultToFile(const std::filesystem::path &filepath, const FitResult& fit_result)
    {
        writeJsonToFile(filepath, serializeFitResult(fit_result));
    }

    /// @brief Deserializes the json object into a fit result.
    FitResult deserializeFitResult(const nlohmann::json &fit_result_json)
    {
        FitResult fit_result;

        fit_result.error = fit_result_json["error"].get<FitResult::Error>();

        const nlohmann::json& soft_iron = fit_result_json["softIronMatrix"];
        fit_result.soft_iron_matrix <<
            soft_iron["xx"].get<double>(), soft_iron["xy"].get<double>(), soft_iron["xz"].get<double>(),
            soft_iron["yx"].get<double>(), soft_iron["yy"].get<double>(), soft_iron["yz"].get<double>(),
            soft_iron["zx"].get<double>(), soft_iron["zy"].get<double>(), soft_iron["zz"].get<double>();

        // Parse hard iron offset
        const nlohmann::json& hard_iron = fit_result_json["hardIronOffset"];
        fit_result.hard_iron_offset <<
            hard_iron["x"].get<double>(),
            hard_iron["y"].get<double>(),
            hard_iron["z"].get<double>();

        return fit_result;
    }

    FitResult deserializeFitResultFromFile(const std::filesystem::path& filepath)
    {
        std::ifstream file(filepath);

        if (!file.is_open())
        {
            return FitResult::noCorrection(FitResult::Error::DESERIALIZATION_COULD_NOT_OPEN_FILE);
        }

        return deserializeFitResult(nlohmann::json::parse(file));
    }

    /// @brief Writes the json content to the given file.
    void writeJsonToFile(const std::filesystem::path &filepath, const nlohmann::json& json_output)
    {
        std::ofstream json_file(filepath);
        json_file << std::setw(2) << json_output;
    }
}
