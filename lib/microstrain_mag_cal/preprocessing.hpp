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

        struct Hash
        {
            size_t operator()(const VoxelKey &k) const noexcept
            {
                // Prime number mixing (similar to boost hash_combine)
                size_t seed = 0;
                seed ^= std::hash<int>()(k.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                seed ^= std::hash<int>()(k.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                seed ^= std::hash<int>()(k.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

                return seed;
            }
        };
    };


    class VoxelGrid
    {
    public:
        static constexpr double DEFAULT_VOXEL_SIZE = 0.015;

        VoxelGrid() = delete;
        explicit VoxelGrid(const double voxel_size) : m_voxel_size(voxel_size) {}

        bool isPointInUniqueVoxel(const std::array<float, 3> &point);

    private:
        std::unordered_set<VoxelKey, VoxelKey::Hash> m_occupied_voxels{};
        const double m_voxel_size;
    };


    class PointManager
    {
    public:
        PointManager() = delete;

        explicit PointManager(VoxelGrid unique_point_grid, size_t data_size_estimate);
        explicit PointManager(const size_t data_size_estimate)
            : PointManager(VoxelGrid(VoxelGrid::DEFAULT_VOXEL_SIZE), data_size_estimate) {}
        explicit PointManager(const VoxelGrid &unique_point_grid)
            : PointManager(unique_point_grid, 0) {}

        void addPoint(const std::array<float, 3> &point);

        Eigen::MatrixX3d getMatrix() const;

        size_t getNumPointsSeen() const;
        size_t getNumFilteredPoints() const;

    private:
        // Extract point vectors as flattened list of points (x1, y1, z1, ..., xN, yN, zN).
        // We can then map the Eigen matrix directly to this list for zero-copy.
        std::vector<double> m_flattened_points;
        VoxelGrid m_unique_point_grid;
        size_t m_num_points_seen = 0;
    };
}
