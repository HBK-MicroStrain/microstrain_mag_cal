#include "composed_coefficients.hpp"


namespace microstrain_mag_cal
{
    /// @brief Composes two calibration corrections into a single combined correction.
    ///
    /// When calibrating data that is already-calibrated, the new correction must be composed with
    /// the old one to produce a single correction that applies both transformations.
    ///
    /// This function computes:
    ///     composed(m) = new_calibration(old_calibration(m)).
    ///
    /// Example: If a sensor has calibration A applied, and you collect data to compute correction B,
    /// then B alone only corrects the residual error. To fully correct raw data, you need to apply
    /// both: first A, then B. This function composes them so you can apply both in one step.
    ///
    /// @param old_fit The old calibration that was previously on the device.
    /// @param new_fit The new calibration that is ready to be applied to the device.
    ///
    /// @return A single composed calibration that can be applied to the device in one step.
    ///
    FitResult composeCorrections(const FitResult &old_fit, const FitResult &new_fit)
    {
        FitResult combined_fit;

        combined_fit.soft_iron_matrix = new_fit.soft_iron_matrix * old_fit.soft_iron_matrix;
        combined_fit.hard_iron_offset = old_fit.hard_iron_offset +
                                        old_fit.soft_iron_matrix.colPivHouseholderQr().solve(new_fit.hard_iron_offset);

        return combined_fit;
    }
}
