#include <set>

#include <microstrain_mag_cal.hpp>

#define M_PI   3.14159265358979323846
#define M_PI_2 1.57079632679489661923 // π/2


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
}
