#pragma once

#include <microstrain_mag_cal/calibration.hpp>


namespace microstrain_mag_cal
{
    FitResult composeCorrections(const FitResult &old_fit, const FitResult &new_fit);

    template<typename T>
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> toSoftIronMatrix(const T* row_major_data)
    {
        return Eigen::Map<const Eigen::Matrix<T, 3, 3, Eigen::RowMajor>>(row_major_data)
            .template cast<double>();
    }

    template<typename T>
    Eigen::Vector3d toHardIronOffset(const T* data)
    {
        return Eigen::Map<const Eigen::Vector<T, 3>>(data)
            .template cast<double>();
    }
};
