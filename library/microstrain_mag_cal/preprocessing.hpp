#pragma once

#include <unordered_set>
#include <vector>

#include <Eigen/Dense>


namespace microstrain_mag_cal
{
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
        VoxelGrid() = delete;
        explicit VoxelGrid(const double voxel_size) : m_voxel_size(voxel_size) {}

        bool isPointInUniqueVoxel(const std::array<float, 3> &point);

    private:
        std::unordered_set<VoxelKey> m_occupied_voxels{};
        double m_voxel_size;
    };

    class PointManager
    {
    public:
        explicit PointManager(const VoxelGrid &unique_point_grid, size_t data_size_estimate);
        // TODO: Document - Moderate default filtering, might get better results taking into account the field strength
        explicit PointManager(const size_t data_size_estimate) : PointManager(VoxelGrid(0.5), data_size_estimate) {}
        explicit PointManager(const VoxelGrid &unique_point_grid) : PointManager(unique_point_grid, 0) {}

        void addPoint(const std::array<float, 3> &point);

        Eigen::MatrixX3d getMatrix();

    private:
        // Extract point vectors as flattened list of points (x1, y1, z1, ..., xN, yN, zN).
        // We can then map the Eigen matrix directly to this list for zero-copy.
        std::vector<double> m_flattened_points;
        VoxelGrid m_unique_point_grid;
    };
}
