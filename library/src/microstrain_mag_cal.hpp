#pragma once

#include <unordered_set>
#include <vector>

#include <Eigen/Dense>


namespace microstrain_mag_cal
{
    // ---------------------------------------------------------------------------------------------
    // Data Structures
    // ---------------------------------------------------------------------------------------------

    struct VoxelKey
    {
        int x, y, z;

        bool operator==(const VoxelKey &other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };
}

namespace std
{
    template<>
    struct hash<microstrain_mag_cal::VoxelKey>
    {
        size_t operator()(const microstrain_mag_cal::VoxelKey &k) const noexcept
        {
            // TODO: This should be faster than alternatives for smaller datasets. Let's look into
            //       adding a check for data that exceeds hundreds of thousands of points and switch
            //       to a more collision-friendly implementation in that case.
            return hash<int>()(k.x) ^ (hash<int>()(k.y) << 1) ^ (hash<int>()(k.z) << 2);
        }
    };
}

namespace microstrain_mag_cal
{
    class VoxelGrid
    {
    public:
        explicit VoxelGrid(const double voxel_size) : m_voxel_size(voxel_size) {}

    private:
        std::unordered_set<VoxelKey> m_occupied_voxels;
        double m_voxel_size;
    };

    class PointManager
    {
    public:
        PointManager() = default;
        explicit PointManager(size_t data_size_estimate);

        void addPoint(const std::array<float, 3> &point);

        Eigen::MatrixX3d getMatrix();

    private:
        // Extract point vectors as flattened list of points (x1, y1, z1, ..., xN, yN, zN).
        // We can then map the Eigen matrix directly to this list for zero-copy.
        std::vector<double> m_flattened_points;
    };

    struct FitResult
    {
        FitResult(Eigen::Matrix<double, 3, 3> soft_iron_matrix, Eigen::Vector3d hard_iron_offset, const bool succeeded) :
            soft_iron_matrix(std::move(soft_iron_matrix)),
            hard_iron_offset(std::move(hard_iron_offset)),
            succeeded(succeeded) {}

        const Eigen::Matrix<double, 3, 3> soft_iron_matrix;
        const Eigen::Vector3d hard_iron_offset;
        const bool succeeded;
    };

    // ---------------------------------------------------------------------------------------------
    // Initial Parameter Estimation
    // ---------------------------------------------------------------------------------------------

    Eigen::RowVector3d estimateInitialHardIronOffset(const Eigen::MatrixX3d &points);

    // ---------------------------------------------------------------------------------------------
    // Data Statistics
    // ---------------------------------------------------------------------------------------------

    double calculateMeanMeasuredFieldStrength(const Eigen::MatrixX3d &points, const Eigen::RowVector3d &initial_offset);
    double calculateSpatialCoverage(const Eigen::MatrixX3d &points, const Eigen::RowVector3d &initial_offset);

    // ---------------------------------------------------------------------------------------------
    // Calibration Fitting
    // ---------------------------------------------------------------------------------------------

    FitResult fitSphere(
        const Eigen::MatrixX3d &points,
        double field_strength,
        const Eigen::RowVector3d &initial_offset);

    FitResult fitEllipsoid(
        const Eigen::MatrixX3d &points,
        double field_strength,
        const Eigen::RowVector3d &initial_offset);

    // ---------------------------------------------------------------------------------------------
    // Fit Quality metrics
    // ---------------------------------------------------------------------------------------------

    double calculateFitRMSE(const Eigen::MatrixX3d &points, const FitResult &fit_result, double field_strength);
}
