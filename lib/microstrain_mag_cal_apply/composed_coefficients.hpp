#pragma once

#include <microstrain_mag_cal/calibration.hpp>


namespace microstrain_mag_cal
{
    FitResult composeCorrections(const FitResult &old_fit, const FitResult &new_fit);

    /// @brief Converts raw row-major soft-iron matrix data to a row-major matrix in the library format.
    ///
    /// This can be used to convert the raw data read from a device so it can be used with the
    /// library. This is especially useful when composing a new calibration onto an old one.
    ///
    /// @tparam T Input data type (float, double, etc.)
    ///
    /// @param row_major_data Pointer to 9 elements in row-major order.
    ///
    /// @return 3x3 row-major soft-iron matrix in the library format.
    ///
    template<typename T>
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> toSoftIronMatrix(const T* row_major_data)
    {
        return Eigen::Map<const Eigen::Matrix<T, 3, 3, Eigen::RowMajor>>(row_major_data)
            .template cast<double>();
    }

    /// @brief Converts raw hard-iron offset data to a vector in the library format.
    ///
    /// This can be used to convert the raw data read from a device so it can be used with the
    /// library. This is especially useful when composing a new calibration onto an old one.
    ///
    /// @tparam T Input data type (float, double, etc.)
    ///
    /// @param data Pointer to 3 elements.
    ///
    /// @return Hard-iron offset vector in the library format.
    ///
    template<typename T>
    Eigen::Vector3d toHardIronOffset(const T* data)
    {
        return Eigen::Map<const Eigen::Vector<T, 3>>(data)
            .template cast<double>();
    }
};
