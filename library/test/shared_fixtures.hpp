#pragma once

#include <microstrain_mag_cal/calibration.hpp>

namespace fixture
{
    /// @brief Creates a matrix of synthetic magnetometer data with the given error coefficients.
    ///
    /// The matrix is created using the clean data matrix generated from the script.
    ///
    /// Reference: https://pmc.ncbi.nlm.nih.gov/articles/PMC8401862/#sec2-sensors-21-05288
    ///
    /// Usage:
    /// 1) Add any desired error coefficients
    /// 2) Apply the error (returns the matrix)
    ///
    class MagCalDataBuilder
    {
    public:
        explicit MagCalDataBuilder(const Eigen::MatrixX3d &data) : m_base_data(data) {}

        void addUniformBias(const double bias)
        {
            m_bias = Eigen::RowVector3d(bias, bias, bias);
        }

        void addBias(const Eigen::RowVector3d &bias)
        {
            m_bias = bias;
        }

        void addUniformScaleFactor(const double scale_factor)
        {
            m_error_matrix.diagonal() = Eigen::Vector3d(scale_factor, scale_factor, scale_factor);
        }

        void addScaleFactor(const Eigen::Vector3d &scale_factor)
        {
            m_error_matrix.diagonal() = scale_factor;
        }

        void addUniformCrossCoupling(const double cross_coupling)
        {
            m_error_matrix.triangularView<Eigen::StrictlyUpper>().setConstant(cross_coupling);
            m_error_matrix.triangularView<Eigen::StrictlyLower>().setConstant(cross_coupling);
        }

        [[nodiscard]] Eigen::MatrixX3d applyError() const
        {
            // -----------------------------------------------------------
            // Single mag cal point real-world physics model:
            // -----------------------------------------------------------
            //    m_r = E * m_t + b, where:
            //
            //        m_r := 3x1 column vector of the raw mag cal reading
            //          E := 3x3 error distortion matrix
            //        m_t := 3x1 column vector of the true mag cal value
            //          b := 3x1 column vector of bias
            //
            // -----------------------------------------------------------
            // To convert the model to our format:
            // -----------------------------------------------------------
            //    m_r^T = (E * m_t + b)^T
            //          = m_t^T * E^T + b^T, where:
            //
            //        m_r^T := 1x3 row vector of raw mag cal data
            //          E^T := Flipped 3x3 error distortion matrix
            //        m_t^T := 1x3 row vector of true mag cal data
            //          b^T := 1x3 row vector of bias
            //
            // -----------------------------------------------------------
            //
            // Now, the data matrix is Nx3, where each row is the 1x3 row vector.
            //
            return (m_base_data * m_error_matrix.transpose()).rowwise() + m_bias;
        }

        [[nodiscard]] Eigen::MatrixX3d getDistortionMatrix() const
        {
           return m_error_matrix;
        }

    private:
        const Eigen::MatrixX3d m_base_data;  // Copying is safer here.

        Eigen::RowVector3d m_bias{0.0, 0.0, 0.0};
        Eigen::Matrix<double, 3, 3> m_error_matrix = Eigen::Matrix<double, 3, 3>::Identity();
    };


    /// @brief Applies corrections to the given data.
    ///
    /// @param data Nx3 matrix of data with error to correct
    /// @param error_correction 3x3 error correction matrix
    /// @param bias 1x3 row vector of bias
    ///
    /// @returns Nx3 matrix containing the corrected data
    ///
    inline Eigen::MatrixX3d applyCorrections(
        const Eigen::MatrixX3d &data,
        const Eigen::Matrix3d &error_correction,
        const Eigen::RowVector3d &bias)
    {
        // ---------------------------------------------------------------
        // Correction equation is the inverse of the distortion equation:
        // ---------------------------------------------------------------
        //    m_t = E^-1 * (m_r - b), where:
        //
        //         m_t := 3x1 column vector of true mag cal data
        //        E^-1 := The 3x3 error correction matrix
        //         m_r := 3x1 column vector of raw mag cal data
        //           b := 3x1 column vector of bias
        //
        // ---------------------------------------------------------------
        // To convert the model to our format:
        // ---------------------------------------------------------------
        //     m_t^T = (E^-1 * (m_r - b))^T
        //           = (m_r - b)^T * (E^-1)^T
        //           = (m_r^T - b^T) * (E^-1)^T, where:
        //
        //            m_t^T := 1x3 row vector of true mag cal data
        //         (E^-1)^T := Flipped 3x3 error correction matrix
        //            m_r^T := 1x3 row vector of raw mag cal data
        //              b^T := 1x3 row vector of bias
        //
        // ---------------------------------------------------------------
        //
        // Now, the data matrix is Nx3, where each row is the 1x3 row vector.
        //
        return (data.rowwise() - bias) * error_correction.transpose();
    }

    /// @brief Convenience wrapper for fit result.
    inline Eigen::MatrixX3d applyCorrections(const Eigen::MatrixX3d &data, const microstrain_mag_cal::FitResult &result)
    {
        return applyCorrections(data, result.soft_iron_matrix, result.hard_iron_offset);
    }
}
