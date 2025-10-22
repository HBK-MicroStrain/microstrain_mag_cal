#pragma once

#include "Device/DataBuffer.hpp"
#include "Utility/Curl.hpp"
#include "Widget/Core/NumberInput.hpp"
#include "Widget/Core/SplitViewContent.hpp"
#include "Widget/Core/Threepp.hpp"

namespace Device
{
    class MipDevice;
} // namespace Device

namespace Widget
{
    class MagCalSection final : public SplitViewContent
    {
    public:
        enum class Mode : uint8_t
        {
            IDLE = 0,
            COLLECT,
            VERIFY
        };

        enum class FitStatus : uint8_t
        {
            NONE = 0,
            VALID,
            INVALID
        };

        // Device-side data store for this feature
        struct MagCalDataStore
        {
            std::mutex lock;

            static constexpr double MIN_MAG_ANGLE_DEG                  = 2.5;
            static constexpr double MAX_MAG_MAGNITUDE                  = 5.0;
            static constexpr double SPATIAL_CALC_VERTICAL_SUBDIVISIONS = 8.0;
            static constexpr double MAX_POINT_DT_MS                    = 100.0;

            std::vector<std::shared_ptr<threepp::Object3D>>    points;
            std::vector<std::shared_ptr<threepp::Object3D>>    verify_points;
            std::chrono::time_point<std::chrono::steady_clock> last_point_time;

            threepp::Vector3 mag_points_mean;
            double measured_field_strength = 0.0;
            double spacial_coverage        = 0.0;
            double fit_rmse                = 0.0;

            bool   use_custom_field_strength   = false;
            double custom_field_strength_gauss = 0.0;
            double latitude                    = 0.0;
            double longitude                   = 0.0;

            NumberInputState field_strength_config;
            NumberInputState latitude_config;
            NumberInputState longitude_config;

            NumberInputState curr_matrix_config[9];
            NumberInputState curr_offset_config[3];

            mip::commands_3dm::MagSoftIronMatrix *cached_matrix         = nullptr;
            bool                                 *cached_matrix_changed = nullptr;

            mip::commands_3dm::MagHardIronOffset *cached_offset         = nullptr;
            bool                                 *cached_offset_changed = nullptr;

            FitStatus fit_status = FitStatus::NONE;

            threepp::Object3D *selected_point = nullptr;
            std::vector<Device::DataBuffer::CallBackId> callback_ids;

            Mode data_mode = Mode::IDLE;

            bool collect_hide = false;
            bool verify_hide  = false;

            int selected_calibration = 0;

            bool has_computed_fit = false;

            bool isUniquePoint(threepp::Vector3 point, const std::vector<std::shared_ptr<threepp::Object3D>> &curr_points);
            void calculateStatistics();
            void calculateFitRMSE(mip::commands_3dm::MagSoftIronMatrix &matrix, mip::commands_3dm::MagHardIronOffset &offset);

            threepp::Vector3 addCalibrationToPoint(const threepp::Vector3 &point_pos, mip::commands_3dm::MagSoftIronMatrix &matrix,
                mip::commands_3dm::MagHardIronOffset &offset);

            threepp::Vector3 removeCalibrationFromPoint(threepp::Vector3 &point_pos, mip::commands_3dm::MagSoftIronMatrix &matrix,
                mip::commands_3dm::MagHardIronOffset &offset);

            bool calculateSphericalFit();
            bool calculateEllipsoidalFit();

            MagCalDataStore()
            {
                field_strength_config.label = "Field Strength##Gauss";

                latitude_config.label = "Latitude##Degrees";
                latitude_config.setInclusiveLimits(-90.0, 90.0);

                longitude_config.label = "Longitude##Degrees";
                longitude_config.setInclusiveLimits(-180.0, 180.0);

                for (int i = 0; i < 9; i++)
                {
                    curr_matrix_config[i].setSignificantDigits(5);
                }

                curr_offset_config[0].label = "X";
                curr_offset_config[0].setSignificantDigits(5);

                curr_offset_config[1].label = "Y";
                curr_offset_config[1].setSignificantDigits(5);

                curr_offset_config[2].label = "Z";
                curr_offset_config[2].setSignificantDigits(5);
            }

            double fieldStrength() const
            {
                if (use_custom_field_strength)
                {
                    return custom_field_strength_gauss;
                }

                return measured_field_strength;
            }

            // Eigen-required functor base class
            // Important: Do not rename typedefs or accessors!  Eigen expects specific names

            template<typename Scalar_, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
            struct MagCalFunctor
            {
                std::vector<std::shared_ptr<threepp::Object3D>> *m_points_ptr;
                double radius = 0.5;

                typedef Scalar_ Scalar;

                enum {
                  InputsAtCompileTime = NX,
                  ValuesAtCompileTime = NY
                };

                typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
                typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
                typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

                int m_inputs;
                int m_values;

                MagCalFunctor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
                MagCalFunctor(const int inputs, const int values) : m_inputs(inputs), m_values(values) {}

                int inputs() const { return m_inputs; }
                int values() const { return m_values; }

                void inputs(const int num_inputs) { m_inputs = num_inputs; }
                void values(const int num_values) { m_values = num_values; }

                // Must be defined in the subclass:
                // void operator() (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const;
            };

            // Spherical Fit Functor
            struct SphericalFitFunctor : MagCalFunctor<double>
            {
                int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
                {
                    //Calculates the following model:
                    // y^2 - (p_meas - bias)T * A * (p_meas - bias)
                    // where A = [scale^2 0 0; 0 scale^2 0; 0 0 scale^2]
                    // with x = [scale^2, bias_x, bias_y, bias_z]

                    Eigen::Matrix3d A;
                    Eigen::Vector3d bias;

                    A    << x[0], 0.0, 0.0, 0.0, x[0], 0.0, 0.0, 0.0, x[0];
                    bias << x[1], x[2], x[3];

                    //Calculate the residual values
                    for (int i = 0; i < m_points_ptr->size(); i++)
                    {
                        threepp::Vector3 point_pos = m_points_ptr->at(i)->position;
                        Eigen::Vector3d  point;

                        point << point_pos[0], point_pos[1], point_pos[2];

                        fvec[i] = radius * radius - (point - bias).transpose() * A * (point - bias);
                    }

                    return 0;
                }
            };

            // Ellipsoidal Fit Functor

            struct EllipsoidalFitFunctor : MagCalFunctor<double>
            {
                int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
                {
                    //Calculates the following model:
                    // y^2 - (p_meas - bias)T * A * (p_meas - bias)
                    // where A = [A_xx A_xy A_xz; A_xy A_yy A_yz; A_xz A_yz A_zz]
                    // with x = [A_xx, A_xy, A_xz, A_yy, A_yz, A_zz, off_x, off_y, off_z]

                    Eigen::Matrix3d A;
                    Eigen::Vector3d bias;

                    A    << x[0], x[1], x[2], x[1], x[3], x[4], x[2], x[4], x[5];
                    bias << x[6], x[7], x[8];

                    //Calculate the residual values
                    for (int i = 0; i < m_points_ptr->size(); i++)
                    {
                        threepp::Vector3 point_pos = m_points_ptr->at(i)->position;
                        Eigen::Vector3d   point;

                        point << point_pos[0], point_pos[1], point_pos[2];

                        fvec[i] = radius * radius - (point - bias).transpose() * A * (point - bias);
                    }

                    return 0;
                }
            };
        };

        using MagCalDataStorePtr = std::shared_ptr<MagCalDataStore>;

        MagCalSection();

        bool drawMenuPanel(Device::MipDevice &device)   override;
        bool drawDetailPanel(Device::MipDevice &device) override;

    private:
        static inline const std::string MAG_POINT    = "mag_point";
        static inline const std::string VERIFY_POINT = "verify_point";

        static inline const std::string CAL_SPHERE_OBJECT_NAME    = "cal_sphere";
        static inline const std::string VERIFY_SPHERE_OBJECT_NAME = "verify_sphere";

        static constexpr int CURRENT_CALIBRATION     = 0;
        static constexpr int SPHERICAL_CALIBRATION   = 1;
        static constexpr int ELLIPSOIDAL_CALIBRATION = 2;

        static constexpr std::chrono::duration<double> MAX_POINT_AGE_S = std::chrono::milliseconds(100);

        std::shared_ptr<threepp::Object3D> add3DPoint(threepp::Vector3 pos, const std::string &type) const;
        void                               removeAll3DPoints(std::string type) const;
        void                               updateCalibration(Device::MipDevice &device, bool user_changed = false) const;

        Utility::Curl m_curl;

        ThreeppState m_visualization_window;

        std::shared_ptr<threepp::Mesh> m_origin, m_calibration_sphere, m_verify_sphere;
        std::shared_ptr<threepp::Text2D> m_origin_label;

        std::shared_ptr<threepp::SphereGeometry>       m_point_geometry               = nullptr;
        std::shared_ptr<threepp::MeshStandardMaterial> m_mag_point_material           = nullptr;
        std::shared_ptr<threepp::MeshStandardMaterial> m_mag_point_highlight_material = nullptr;
        std::shared_ptr<threepp::MeshStandardMaterial> m_verify_point_material        = nullptr;

        // Device change callback
        void deviceChangeCallback(Device::MipDevice *device, const Application::DeviceManager::CallbackReason reason) const;
    };
} // namespace Widget
