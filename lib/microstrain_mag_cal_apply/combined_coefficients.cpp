#include "combined_coefficients.hpp"


namespace microstrain_mag_cal
{
    FitResult combineCorrections(const FitResult &old_fit, const FitResult &new_fit)
    {
        FitResult combined_fit;

        combined_fit.soft_iron_matrix = new_fit.soft_iron_matrix * old_fit.soft_iron_matrix;
        combined_fit.hard_iron_offset = (old_fit.hard_iron_offset.transpose() + (old_fit.soft_iron_matrix.inverse() * new_fit.hard_iron_offset.transpose())).transpose();

        return combined_fit;
    }
}
