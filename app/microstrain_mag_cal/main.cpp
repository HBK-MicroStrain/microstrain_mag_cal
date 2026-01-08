#include <filesystem>
#include <iostream>

#include <magic_enum/magic_enum.hpp>

#include <microstrain_mag_cal/analysis.hpp>
#include <microstrain_mag_cal/calibration.hpp>
#include "backend.hpp"
#include "cli.hpp"


using microstrain_mag_cal::FitResult;


int main(const int argc, char **argv)
{
    cli::ProgramArgs args(argv);
    CLI11_PARSE(args.app, argc, argv);

    const std::optional<backend::MappedBinaryData> mapped_data = backend::mapBinaryFile(args.input_data_filepath);
    assert(mapped_data.has_value());

    const microstrain_mag_cal::PointManager point_manager = backend::extractPointsFromRawData(mapped_data->view, args.field_strength);
    const Eigen::MatrixX3d points = point_manager.getMatrix();
    const Eigen::RowVector3d initial_offset = microstrain_mag_cal::estimateInitialHardIronOffset(points);

    if (!args.field_strength.has_value())
    {
        printf("\nNOTE: Using estimate for field strength.\n");

        args.field_strength = microstrain_mag_cal::calculateMeanMeasuredFieldStrength(points, initial_offset);
    }

    if (args.display_analysis)
    {
        printf("\n");
        printf("--------------------------------------------------\n");
        printf("Analysis:\n");
        printf("--------------------------------------------------\n");
        printf("  Used Points       : %s\n", cli::getPointUsageDisplay(point_manager).c_str());
        printf("  Spatial Coverage  : %.5f%%\n", microstrain_mag_cal::calculateSpatialCoverage(points, initial_offset));
        printf("  Field Strength    : %.5f\n", args.field_strength.value());
        printf("  Initial Offset    : [%.5f, %.5f, %.5f]\n", initial_offset.x(), initial_offset.y(), initial_offset.z());
    }

    if (args.spherical_fit)
    {
        const FitResult fit_result = microstrain_mag_cal::fitSphere(points, args.field_strength.value(), initial_offset);
        const double fit_RMSE = microstrain_mag_cal::calculateFitRMSE(points, fit_result, args.field_strength.value());

        cli::displayFitResult("Spherical Fit", fit_result, fit_RMSE);

        if (!args.output_json_directory.empty())
        {
            microstrain_mag_cal::serializeFitResultToFile(args.output_json_directory / "spherical_fit.json", fit_result);
        }
    }

    if (args.ellipsoidal_fit)
    {
        const FitResult fit_result = microstrain_mag_cal::fitEllipsoid(points, args.field_strength.value(), initial_offset);
        const double fit_RMSE = microstrain_mag_cal::calculateFitRMSE(points, fit_result, args.field_strength.value());

        cli::displayFitResult("Ellipsoidal Fit", fit_result, fit_RMSE);

        if (!args.output_json_directory.empty())
        {
            microstrain_mag_cal::serializeFitResultToFile(args.output_json_directory / "ellipsoidal_fit.json", fit_result);
        }
    }

    return 0;
}
