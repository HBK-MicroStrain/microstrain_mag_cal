#include "preprocessing.hpp"


namespace mag_cal_core
{
    bool VoxelGrid::isPointInUniqueVoxel(const std::array<float, 3> &point)
    {
        const VoxelKey key{
            static_cast<int>(std::floor(point[0] / m_voxel_size)),
            static_cast<int>(std::floor(point[1] / m_voxel_size)),
            static_cast<int>(std::floor(point[2] / m_voxel_size))
        };

        return m_occupied_voxels.insert(key).second;
    }

    PointManager::PointManager(const VoxelGrid &unique_point_grid, const size_t data_size_estimate)
        : m_unique_point_grid(unique_point_grid)
    {
        if (data_size_estimate > 0)
        {
            m_flattened_points.reserve(data_size_estimate);
        }
    }

    void PointManager::addPoint(const std::array<float, 3> &point)
    {
        if (m_unique_point_grid.isPointInUniqueVoxel(point))
        {
            m_flattened_points.push_back(point[0]);
            m_flattened_points.push_back(point[1]);
            m_flattened_points.push_back(point[2]);
        }
    }

    Eigen::MatrixX3d PointManager::getMatrix()
    {
        return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>(
            m_flattened_points.data(),
            static_cast<Eigen::Index>(m_flattened_points.size()) / 3,
            3
        );
    }
}
