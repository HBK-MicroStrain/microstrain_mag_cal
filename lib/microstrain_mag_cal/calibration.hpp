#pragma once

#include <filesystem>

#include <Eigen/Dense>
#include <magic_enum/magic_enum.hpp>
#include <nlohmann/json.hpp>


namespace microstrain_mag_cal
{
    struct FitResult
    {
        enum class Error : uint8_t
        {
            NONE,
            FIT_OPTIMIZATION_INSUFFICIENT_INPUT_DATA,
            FIT_OPTIMIZATION_DID_NOT_CONVERGE,
            FIT_CORRECTION_MATRIX_NOT_POSITIVE_DEFINITE,
            DESERIALIZATION_COULD_NOT_OPEN_FILE
        };

        Eigen::Matrix3d soft_iron_matrix;
        Eigen::RowVector3d hard_iron_offset;
        Error error = Error::NONE;

        static FitResult noCorrection(const Error &error);

        friend bool operator==(const FitResult& lhs, const FitResult& rhs);
        friend std::ostream& operator<<(std::ostream& os, const FitResult& result);
    };


    Eigen::RowVector3d estimateInitialHardIronOffset(const Eigen::MatrixX3d &points);
    double calculateMeanMeasuredFieldStrength(const Eigen::MatrixX3d &points, const Eigen::RowVector3d &initial_offset);

    FitResult fitSphere(const Eigen::MatrixX3d &points, double field_strength, const Eigen::RowVector3d &initial_offset);
    FitResult fitEllipsoid(const Eigen::MatrixX3d &points, double field_strength, const Eigen::RowVector3d &initial_offset);

    nlohmann::json serializeFitResult(const FitResult &fit_result);
    void serializeFitResultToFile(const std::filesystem::path &filepath, const FitResult& fit_result);

    FitResult deserializeFitResult(const nlohmann::json &fit_result_json);
    FitResult deserializeFitResultFromFile(const std::filesystem::path& filepath);

    void writeJsonToFile(const std::filesystem::path &filepath, const nlohmann::json& json_output);

    /// @brief Converts an Eigen container to vector.
    ///
    /// @tparam T Type of the data contained within the Eigen container.
    ///
    /// @param container The Eigen container to convert. Can be any Eigen container.
    ///
    /// @returns Vector of type T elements in row-major order from the Eigen container.
    template<typename T, typename Derived>
    std::vector<T> toVector(const Eigen::MatrixBase<Derived>& container)
    {
        constexpr int Options = (Derived::RowsAtCompileTime == 1 || Derived::ColsAtCompileTime == 1)
            ? static_cast<int>(Derived::Options)  // Keep original options for vectors
            : static_cast<int>(Eigen::RowMajor);

        using MatrixTemplate = Eigen::Matrix<
            T,
            Derived::RowsAtCompileTime,
            Derived::ColsAtCompileTime,
            Options,
            Derived::MaxRowsAtCompileTime,
            Derived::MaxColsAtCompileTime>;

        MatrixTemplate converted = container.template cast<T>();

        return std::vector<T>(converted.data(), converted.data() + converted.size());
    }
}


// JSON serializer for fit result
template <>
struct nlohmann::adl_serializer<microstrain_mag_cal::FitResult::Error>
{
    static void to_json(json& error_json, const microstrain_mag_cal::FitResult::Error& error)
    {
        error_json = magic_enum::enum_name(error);
    }

    static void from_json(const json& error_json, microstrain_mag_cal::FitResult::Error& error)
    {
        const std::string error_string = error_json.get<std::string>();

        if (const std::optional<microstrain_mag_cal::FitResult::Error> error_opt =
                magic_enum::enum_cast<microstrain_mag_cal::FitResult::Error>(error_string);
            error_opt.has_value())
        {
            error = error_opt.value();
        }
        else
        {
            const std::string type_name = std::string(magic_enum::enum_type_name<microstrain_mag_cal::FitResult::Error>());

            throw std::invalid_argument("Invalid " + type_name + " enum value: " + error_string);
        }
    }
};
