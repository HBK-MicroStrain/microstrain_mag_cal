#include <microstrain_mag_cal.hpp>

namespace MicrostrainMagCal
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
}
