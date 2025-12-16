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
            FIT_CORRECTION_MATRIX_NOT_POSITIVE_DEFINITE
        };

        Eigen::Matrix3d soft_iron_matrix;
        Eigen::RowVector3d hard_iron_offset;
        Error error = Error::NONE;

        static std::string getErrorMessage(const Error& error);

        friend bool operator==(const FitResult& lhs, const FitResult& rhs);
        friend std::ostream& operator<<(std::ostream& os, const FitResult& result);
    };


    Eigen::RowVector3d estimateInitialHardIronOffset(const Eigen::MatrixX3d &points);
    double calculateMeanMeasuredFieldStrength(const Eigen::MatrixX3d &points, const Eigen::RowVector3d &initial_offset);

    FitResult fitSphere(const Eigen::MatrixX3d &points, double field_strength, const Eigen::RowVector3d &initial_offset);
    FitResult fitEllipsoid(const Eigen::MatrixX3d &points, double field_strength, const Eigen::RowVector3d &initial_offset);

    nlohmann::json convertFitResultToJson(const FitResult &fit_result);
    FitResult parseCalibrationFromJson(const nlohmann::json &fit_result_json);

    void writeJsonToFile(const std::filesystem::path &filepath, const nlohmann::json& json_output);
    void writeJsonToFile(const std::filesystem::path &filepath, const FitResult& fit_result);
}


// JSON serializer for fit result
template <>
struct nlohmann::adl_serializer<microstrain_mag_cal::FitResult::Error>
{
    static void to_json(json& j, const microstrain_mag_cal::FitResult::Error& e)
    {
        j = magic_enum::enum_name(e);
    }

    static void from_json(const json& j, microstrain_mag_cal::FitResult::Error& e)
    {
        const std::string str = j.get<std::string>();

        if (const std::optional<microstrain_mag_cal::FitResult::Error> opt =
                magic_enum::enum_cast<microstrain_mag_cal::FitResult::Error>(str);
            opt.has_value())
        {
            e = opt.value();
        }
        else
        {
            const std::string type_name = std::string(magic_enum::enum_type_name<microstrain_mag_cal::FitResult::Error>());

            throw std::invalid_argument("Invalid " + type_name + " enum value: " + str);
        }
    }
};
