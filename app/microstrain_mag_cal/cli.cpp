#include <cli.hpp>
#include <iostream>

namespace cli
{
    // Console output after the fitting algorithms are run
    void displayFitResult(const std::string &fit_name, const microstrain_mag_cal::FitResult &result, const double fit_RMSE)
    {
        printf("\n");
        printf("--------------------------------------------------\n");
        printf("%s\n", fit_name.data());
        printf("--------------------------------------------------\n");

        std::string result_output = "SUCCESS";
        if (result.error != microstrain_mag_cal::FitResult::Error::NONE)
        {
            result_output = "FAIL - ";
            const std::string_view error_name = magic_enum::enum_name(result.error);
            result_output += error_name.empty() ? "UNKNOWN" : error_name;
        }
        printf("  Result: %s\n\n", result_output.c_str());

        printf("  Soft-Iron Matrix:\n");
        const Eigen::IOFormat fmt(6, 0, " ", "\n", "  ");
        std::cout << result.soft_iron_matrix.format(fmt) << "\n\n";

        printf("  Hard-Iron Offset:\n");
        std::cout << result.hard_iron_offset.format(fmt) << "\n\n";

        printf("  Fit RMSE: %.5f\n", fit_RMSE);
    }
}
