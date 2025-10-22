#include "Widget/Config/MagCalVisualization.hpp"

#include "Application/CustomStyle.hpp"
#include "Application/icomoon.hpp"
#include "Application/Resources.hpp"
#include "Application/Theme.hpp"
#include "Device/DeviceTask.hpp"
#include "Device/MipDevice.hpp"
#include "Widget/Core/CheckboxExtra.hpp"
#include "Widget/Core/DeviceCommand.hpp"
#include "Widget/Core/NumberOutput.hpp"
#include "Widget/Core/Utility.hpp"
#include "Widget/Core/Windowing.hpp"
#include "Widget/Feature/Notifications.hpp"

namespace Widget
{
    bool MagCalSection::MagCalDataStore::isUniquePoint(const threepp::Vector3 point,
        const std::vector<std::shared_ptr<threepp::Object3D>> &curr_points)
    {
        if (!curr_points.size())
        {
            return true;
        }

        for (const std::shared_ptr compare_point : curr_points)
        {
            threepp::Vector3 compare_point_pos = compare_point->position;

            const double angle_deg = static_cast<double>(std::acos(point.dot(compare_point_pos) / (compare_point_pos.length() * point.length())) *
                180.0f / M_PI);

            if (angle_deg < MIN_MAG_ANGLE_DEG)
            {
                return false;
            }
        }

        return true;
    }

    void MagCalSection::MagCalDataStore::calculateStatistics()
    {
        if (!points.size())
        {
            mag_points_mean         = threepp::Vector3::ZEROS();
            measured_field_strength = 0.0;
            spacial_coverage        = 0.0;
            return;
        }

        // Determine the field offset
        threepp::Vector3 field_offset;

        for (const std::shared_ptr point : points)
        {
            field_offset += point->position;
        }

        field_offset /= static_cast<float>(points.size());

        mag_points_mean = field_offset;

        measured_field_strength = 0.0;

        //Calculate field strength
        for (const std::shared_ptr point : points)
        {
            measured_field_strength += (point->position - mag_points_mean).length();
        }

        measured_field_strength /= points.size();

        //Calculate spatial coverage
        //Note: since we are using the measured field offset, the calculation isn't perfect and may increase if a point is removed or decrease when a point is added
        //      if that point shifts the measured field offset just right.

        std::vector<threepp::Spherical> mag_points_spherical;

        for (const std::shared_ptr point : points)
        {
            threepp::Spherical point_spherical;

            point_spherical.setFromVector3(point->position - mag_points_mean);

            mag_points_spherical.push_back(point_spherical);
        }

        // Loop through the spatial subdivisions and check if we have points in each section
        int num_occupied_divisions = 0;

        constexpr double angle_delta = M_PI / SPATIAL_CALC_VERTICAL_SUBDIVISIONS;

        for (double phi = 0.0; phi < M_PI; phi += angle_delta)
        {
            for (double theta = -M_PI; theta < M_PI; theta += angle_delta)
            {
                for (int i = 0; i < mag_points_spherical.size(); i++)
                {
                    if (mag_points_spherical[i].phi >= phi && mag_points_spherical[i].phi < phi + angle_delta &&
                        mag_points_spherical[i].theta >= theta && mag_points_spherical[i].theta < theta + angle_delta)
                    {
                        num_occupied_divisions += 1;
                        break;
                    }
                }
            }
        }

        //Note: factor of 2 comes from the fact that phi-> 0 to pi, theta -> -pi to pi
        spacial_coverage = static_cast<double>(num_occupied_divisions) /
            (SPATIAL_CALC_VERTICAL_SUBDIVISIONS * 2.0 * SPATIAL_CALC_VERTICAL_SUBDIVISIONS) * 100.0;
    }

    threepp::Vector3 MagCalSection::MagCalDataStore::addCalibrationToPoint(const threepp::Vector3 &point_pos,
        mip::commands_3dm::MagSoftIronMatrix &matrix, mip::commands_3dm::MagHardIronOffset &offset)
    {
        threepp::Vector3 bias;
        threepp::Matrix3 mat;

        bias.set(offset.offset[0], offset.offset[1], offset.offset[2]);
        mat.set(
            matrix.offset[0], matrix.offset[1], matrix.offset[2],
            matrix.offset[3], matrix.offset[4], matrix.offset[5],
            matrix.offset[6], matrix.offset[7], matrix.offset[8]
        );

        return (point_pos - bias).applyMatrix3(mat);
    }

    threepp::Vector3 MagCalSection::MagCalDataStore::removeCalibrationFromPoint(threepp::Vector3 &point_pos,
        mip::commands_3dm::MagSoftIronMatrix &matrix, mip::commands_3dm::MagHardIronOffset &offset)
    {
        threepp::Vector3 bias;
        threepp::Matrix3 mat;

        bias.set(offset.offset[0], offset.offset[1], offset.offset[2]);
        mat.set(
            matrix.offset[0], matrix.offset[1], matrix.offset[2],
            matrix.offset[3], matrix.offset[4], matrix.offset[5],
            matrix.offset[6], matrix.offset[7], matrix.offset[8]
        );

        const threepp::Matrix3 mat_inv = mat.invert();

        return point_pos.applyMatrix3(mat_inv) + bias;
    }

    bool MagCalSection::MagCalDataStore::calculateSphericalFit()
    {
        SphericalFitFunctor spherical_fit;
        Eigen::VectorXd x(4);

        has_computed_fit = false;

        if (!cached_matrix || !cached_offset)
        {
            return false;
        }

        //Set initial conditions for fit
        x << 1.0, mag_points_mean[0], mag_points_mean[1], mag_points_mean[2];

        spherical_fit.radius       = fieldStrength();
        spherical_fit.m_points_ptr = &points;
        spherical_fit.values(static_cast<int>(points.size()));

        Eigen::NumericalDiff spherical_fit_with_numerical_diff(spherical_fit);
        Eigen::LevenbergMarquardt lm(spherical_fit_with_numerical_diff);

        lm.parameters.maxfev = 1000;
        lm.parameters.xtol   = 1.0e-10;

        const Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(x);

        if (status == Eigen::LevenbergMarquardtSpace::Status::RelativeErrorTooSmall ||
            status == Eigen::LevenbergMarquardtSpace::Status::RelativeReductionTooSmall)
        {
            fit_status = FitStatus::VALID;
            has_computed_fit = true;
        }
        else
        {
            fit_status = FitStatus::INVALID;
        }

        cached_matrix->offset.fill(0.0f);
        cached_matrix->offset[0] = static_cast<float>(sqrt(x[0]));
        cached_matrix->offset[4] = cached_matrix->offset[0];
        cached_matrix->offset[8] = cached_matrix->offset[0];

        cached_offset->offset[0] = static_cast<float>(x[1]);
        cached_offset->offset[1] = static_cast<float>(x[2]);
        cached_offset->offset[2] = static_cast<float>(x[3]);

        *cached_matrix_changed = true;
        *cached_offset_changed = true;

        return true;
    }

    bool MagCalSection::MagCalDataStore::calculateEllipsoidalFit()
    {
        EllipsoidalFitFunctor ellipsoidal_fit;
        Eigen::VectorXd x(9);

        has_computed_fit = false;

        if (!cached_matrix || !cached_offset)
        {
            return false;
        }

        // Set initial conditions for fit
        x << 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, mag_points_mean[0], mag_points_mean[1], mag_points_mean[2];

        ellipsoidal_fit.radius       = fieldStrength();
        ellipsoidal_fit.m_points_ptr = &points;
        ellipsoidal_fit.values(static_cast<int>(points.size()));

        Eigen::NumericalDiff ellipsoidal_fit_with_numerical_diff(ellipsoidal_fit);
        Eigen::LevenbergMarquardt lm(ellipsoidal_fit_with_numerical_diff);

        lm.parameters.maxfev = 1000;
        lm.parameters.xtol   = 1.0e-10;

        const Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(x);

        if (status == Eigen::LevenbergMarquardtSpace::Status::RelativeErrorTooSmall ||
            status == Eigen::LevenbergMarquardtSpace::Status::RelativeReductionTooSmall)
        {
            fit_status = FitStatus::VALID;
            has_computed_fit = true;
        }
        else
        {
            fit_status = FitStatus::INVALID;
        }

        const Eigen::Matrix3d A_matrix {
            { x[0], x[1], x[2] },
            { x[1], x[3], x[4] },
            { x[2], x[4], x[5] }
        };

        Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver;

        eigen_solver.compute(A_matrix, true);

        // Calculate R * srt(D) * R^T
        Eigen::Matrix3d scale;

        scale.setZero();

        scale(0, 0) = 1.0 / sqrt(1.0 / eigen_solver.eigenvalues().x().real());
        scale(1, 1) = 1.0 / sqrt(1.0 / eigen_solver.eigenvalues().y().real());
        scale(2, 2) = 1.0 / sqrt(1.0 / eigen_solver.eigenvalues().z().real());

        Eigen::Matrix3cd soft_iron = eigen_solver.eigenvectors() * scale * eigen_solver.eigenvectors().transpose();

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                cached_matrix->offset[i * 3 + j] = static_cast<float>(soft_iron(i, j).real());
            }
        }

        cached_offset->offset[0] = static_cast<float>(x[6]);
        cached_offset->offset[1] = static_cast<float>(x[7]);
        cached_offset->offset[2] = static_cast<float>(x[8]);

        *cached_matrix_changed = true;
        *cached_offset_changed = true;

        return true;
    }

    void MagCalSection::MagCalDataStore::calculateFitRMSE(mip::commands_3dm::MagSoftIronMatrix &matrix, mip::commands_3dm::MagHardIronOffset &offset)
    {
        float total = 0.0f;

        const double field_strength_sq = fieldStrength() * fieldStrength();

        threepp::Vector3 bias;
        threepp::Matrix3 mat;

        bias.set(offset.offset[0], offset.offset[1], offset.offset[2]);
        mat.set(
            matrix.offset[0], matrix.offset[1], matrix.offset[2],
            matrix.offset[3], matrix.offset[4], matrix.offset[5],
            matrix.offset[6], matrix.offset[7], matrix.offset[8]
        );

        threepp::Matrix3 mat_transpose = mat;
        mat_transpose.transpose();

        const threepp::Matrix3 A = mat_transpose.multiply(mat);

        for (const std::shared_ptr point : points)
        {
            const double factor = field_strength_sq - static_cast<double>((point->position - bias).applyMatrix3(A).dot(point->position - bias));
            total += static_cast<float>(factor * factor);
        }

        fit_rmse = static_cast<double>(std::sqrt(total / static_cast<float>(points.size())));
    }

    MagCalSection::MagCalSection() : SplitViewContent("Magnetic Calibration", true, true)
    {
        m_required_descriptors.push_back({mip::data_sensor::DESCRIPTOR_SET, mip::data_sensor::DATA_MAG_SCALED});
        m_required_descriptors.push_back({mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_SOFT_IRON_MATRIX});
        m_required_descriptors.push_back({mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_HARD_IRON_OFFSET});

        // Register device update callback
        using namespace std::placeholders;
        Application::getDeviceList().addStatusChangeCallback(std::bind(&MagCalSection::deviceChangeCallback, this, _1, _2), this);

        create3DScene(m_visualization_window);

        // Create Axis
        createAxis(m_origin, m_origin_label, "Origin", 0.2f, 0.03f);
        m_visualization_window.scene->add(m_origin);
        m_origin_label->visible = false;

        m_visualization_window.zoom_extents({m_origin});

        // Create point shared objects
        threepp::SphereGeometry::Params point_params;
        point_params.radius         = 0.01f;
        point_params.heightSegments = 25;
        point_params.widthSegments  = 25;

        m_point_geometry = threepp::SphereGeometry::create(point_params);

        // Mag point material
        m_mag_point_material              = threepp::MeshStandardMaterial::create();
        m_mag_point_material->color       = threepp::Color (1.0f, 0.0f, 0.0f);
        m_mag_point_material->opacity     = 1.0f;
        m_mag_point_material->roughness   = 0.6f;
        m_mag_point_material->metalness   = 0.2f;
        m_mag_point_material->transparent = true;
        m_mag_point_material->opacity     = 0.55f;

        // Mag point highlight material
        m_mag_point_highlight_material              = threepp::MeshStandardMaterial::create();
        m_mag_point_highlight_material->color       = threepp::Color(1.0f, 1.0f, 0.0f);
        m_mag_point_highlight_material->opacity     = 1.0f;
        m_mag_point_highlight_material->roughness   = 0.6f;
        m_mag_point_highlight_material->metalness   = 0.2f;
        m_mag_point_highlight_material->transparent = true;
        m_mag_point_highlight_material->opacity     = 0.6f;

        // Verify point material
        m_verify_point_material              = threepp::MeshStandardMaterial::create();
        m_verify_point_material->color       = threepp::Color(0.0f, 1.0f, 0.0f);
        m_verify_point_material->opacity     = 1.0f;
        m_verify_point_material->roughness   = 0.6f;
        m_verify_point_material->metalness   = 0.2f;
        m_verify_point_material->transparent = true;
        m_verify_point_material->opacity     = 0.6f;


        // Add the calibration and verify spheres
        threepp::SphereGeometry::Params sphere_params;
        sphere_params.radius         = 1.0f;
        sphere_params.heightSegments = 25;
        sphere_params.widthSegments  = 25;

        const std::shared_ptr<threepp::SphereGeometry> sphere_geometry = threepp::SphereGeometry::create(sphere_params);

        const std::shared_ptr<threepp::MeshStandardMaterial> cal_sphere_material = threepp::MeshStandardMaterial::create();
        cal_sphere_material->color                                         = threepp::Color(0.5f, 0.5f, 0.5f);
        cal_sphere_material->opacity                                       = 1.0f;
        cal_sphere_material->roughness                                     = 0.6f;
        cal_sphere_material->metalness                                     = 0.5f;
        cal_sphere_material->transparent                                   = true;
        cal_sphere_material->wireframe                                     = true;

        m_calibration_sphere = threepp::Mesh::create();
        m_calibration_sphere->setGeometry(sphere_geometry);
        m_calibration_sphere->setMaterial(cal_sphere_material);
        m_calibration_sphere->name    = CAL_SPHERE_OBJECT_NAME;
        m_calibration_sphere->visible = false;

        m_visualization_window.scene->add(m_calibration_sphere);

        const std::shared_ptr<threepp::MeshStandardMaterial> verify_sphere_material = threepp::MeshStandardMaterial::create();
        verify_sphere_material->color                                         = threepp::Color(0.0f, 0.6f, 0.0f);
        verify_sphere_material->opacity                                       = 1.0f;
        verify_sphere_material->roughness                                     = 0.6f;
        verify_sphere_material->metalness                                     = 0.5f;
        verify_sphere_material->transparent                                   = true;
        verify_sphere_material->wireframe                                     = true;

        m_verify_sphere = threepp::Mesh::create();
        m_verify_sphere->setGeometry(sphere_geometry);
        m_verify_sphere->setMaterial(verify_sphere_material);
        m_verify_sphere->name    = VERIFY_SPHERE_OBJECT_NAME;
        m_verify_sphere->visible = false;

        m_visualization_window.scene->add(m_verify_sphere);
    }

    void MagCalSection::deviceChangeCallback(Device::MipDevice *device, const Application::DeviceManager::CallbackReason reason) const
    {
        if (!device)
        {
            return;
        }

        // Process device changed/connected callbacks
        if (reason == Application::DeviceManager::CallbackReason::CONNECTED || reason == Application::DeviceManager::CallbackReason::CHANGED)
        {
            if (!device->isSupported<mip::data_sensor::ScaledMag>() || device->hasCustomDataStore(this))
            {
                return;
            }

            MagCalDataStorePtr store = std::make_shared<MagCalDataStore>();

            store->callback_ids.push_back(device->dataBuffer().addDataCallback(mip::data_sensor::DESCRIPTOR_SET, mip::data_sensor::DATA_MAG_SCALED,
                [this, device, store](const std::any &data, const double)->void
                {
                    mip::data_sensor::ScaledMag mag_data = std::any_cast<mip::data_sensor::ScaledMag>(data);

                    threepp::Vector3 point;

                    point.set(mag_data.scaled_mag[0], mag_data.scaled_mag[1], mag_data.scaled_mag[2]);

                    mip::commands_3dm::MagSoftIronMatrix matrix;
                    mip::commands_3dm::MagHardIronOffset offset;

                    if (!device->config().getConfigOnDevice<mip::commands_3dm::MagSoftIronMatrix>(matrix))
                    {
                        return;
                    }

                    if (!device->config().getConfigOnDevice<mip::commands_3dm::MagHardIronOffset>(offset))
                    {
                        return;
                    }

                    if (!store->cached_matrix || !store->cached_offset)
                    {
                        return;
                    }

                    const threepp::Vector3 uncalibrated_point = store->removeCalibrationFromPoint(point, matrix, offset);

                    std::lock_guard lock_guard(store->lock);

                    //If this point passes the uniqueness test, store it and add it to the 3D view
                    if (store->data_mode == Mode::COLLECT && store->isUniquePoint(uncalibrated_point, store->points))
                    {
                        store->points.push_back(add3DPoint(uncalibrated_point, MAG_POINT));
                        store->calculateStatistics();
                    }
                    else if (store->data_mode == Mode::VERIFY)
                    {
                        threepp::Vector3 calibrated_point;

                        if (store->selected_calibration == CURRENT_CALIBRATION)
                        {
                            calibrated_point = store->addCalibrationToPoint(uncalibrated_point, matrix, offset);
                        }
                        else
                        {
                            calibrated_point = store->addCalibrationToPoint(uncalibrated_point, *store->cached_matrix, *store->cached_offset);
                        }

                        if (store->isUniquePoint(calibrated_point, store->verify_points))
                        {
                            store->verify_points.push_back(add3DPoint(calibrated_point, VERIFY_POINT));
                        }
                    }

                    // Store the time the point was received
                    store->last_point_time = std::chrono::steady_clock::now();
                }));

            device->addCustomDataStore(this, store);
        }
        // Remove callbacks if disconnected/removed
        else if (reason == Application::DeviceManager::CallbackReason::DISCONNECTED || reason == Application::DeviceManager::CallbackReason::REMOVED)
        {
            std::any store;

            if (!device->getCustomDataStore(this, store))
            {
                return;
            }

            const MagCalDataStorePtr data_store = std::any_cast<MagCalDataStorePtr>(store);

            // Remove the callbacks
            for (Device::DataBuffer::CallBackId callback_id : data_store->callback_ids)
            {
                device->dataBuffer().deleteDataCallback(callback_id);
            }

            device->deleteCustomDataStore(this);
        }
    }

    bool MagCalSection::drawMenuPanel(Device::MipDevice &device)
    {
        std::any store;

        // Get the data store for this device
        if (!device.getCustomDataStore(this, store))
        {
            return false;
        }

        const MagCalDataStorePtr data_store = std::any_cast<MagCalDataStorePtr>(store);

        // Must have cached values
        if (!device.config().getConfigChangedPointer<mip::commands_3dm::MagSoftIronMatrix>(data_store->cached_matrix, data_store->cached_matrix_changed))
        {
            return false;
        }

        if (!device.config().getConfigChangedPointer<mip::commands_3dm::MagHardIronOffset>(data_store->cached_offset, data_store->cached_offset_changed))
        {
            return false;
        }

        bool has_changes = false;

        // Check if we need to request a point from the device (if the device isn't streaming or it isn't in the message format)
        std::chrono::duration<double> elapsed_time = std::chrono::steady_clock::now() - data_store->last_point_time;

        Device::TaskHandle read_point_task{device, {"GetMagCalPoint"}};

        if (!read_point_task.exists() && data_store->data_mode != Mode::IDLE && elapsed_time > MAX_POINT_AGE_S)
        {
            read_point_task.queue([](Device::MipDevice &task_device)->mip::CompositeResult
            {
                const mip::DescriptorRate mag_descriptor = {mip::data_sensor::DATA_MAG_SCALED, 0};

                //Ideally, we wouldn't wait for the response here!
                if (task_device.isSupported(mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_POLL_DATA))
                {
                    return mip::commands_3dm::pollData(task_device, mip::data_sensor::DESCRIPTOR_SET, false, 1, &mag_descriptor.descriptor);
                }

                if (task_device.isSupported(mip::commands_3dm::DESCRIPTOR_SET, mip::commands_3dm::CMD_POLL_IMU_MESSAGE))
                {
                    return mip::commands_3dm::pollImuMessage(task_device, false, 1, &mag_descriptor);
                }

                return mip::CmdResult::STATUS_NONE;
            });
        }

        // Remove the task when complete
        if (read_point_task.resolveIfComplete())
        {}

        //
        // Data Controls
        //
        {
            SectionHeading("Data Controls", "user_manual_content/installation/Magnetometer%20Calibration.htm");

            // Format selector
            ImGui::AlignTextToFramePadding();
            NumberOutput("Num Points:", static_cast<double>(data_store->points.size()));

            NumberOutput("Measured Field Strength:", data_store->measured_field_strength);
            ImGui::SameLine();
            ImGui::TextUnformatted(" gauss");

            // Insufficient
            if (data_store->spacial_coverage < 33.0)
            {
                ImGui::PushStyleColor(ImGuiCol_Text, RED.Value);
            }
            // Warning
            else if (data_store->spacial_coverage < 66.0)
            {
                ImGui::PushStyleColor(ImGuiCol_Text, DARK_YELLOW.Value);
            }
            // Good
            else
            {
                ImGui::PushStyleColor(ImGuiCol_Text, GREEN.Value);
            }

            NumberOutput("Spatial Coverage:##%", data_store->spacial_coverage);
            ImGui::PopStyleColor();

            ImGui::TextUnformatted("Collect Data:");

            Application::StyleGuard<void> btn_style = theme().action_button();

            if (ImGui::Button(data_store->data_mode == Mode::COLLECT ? "Stop###collect_stop_start" : "Start###collect_stop_start"))
            {
                if (data_store->data_mode == Mode::COLLECT)
                {
                    data_store->data_mode = Mode::IDLE;
                }
                else
                {
                    data_store->data_mode = Mode::COLLECT;
                }
            }

            ImGui::SameLine();

            {
                ConditionalDisable disabled(data_store->data_mode == Mode::COLLECT || data_store->points.size() == 0);

                if (ImGui::Button(data_store->collect_hide ? "Show###collect_show_hide" : "Hide###collect_show_hide"))
                {
                    data_store->collect_hide = !data_store->collect_hide;

                    for (std::shared_ptr object : m_visualization_window.scene->children)
                    {
                        if (object->name == MAG_POINT)
                        {
                            object->visible = !data_store->collect_hide;
                        }
                    }

                    m_calibration_sphere->visible = !data_store->collect_hide;
                }

                ImGui::SameLine();

                // Clear data points popup
                ModalPopupState confirm_clear_data_points({
                    { "Clear Collected Data##MagCalPanel" },                 // Title
                    { "Are you sure you want to clear the collected data?" } // Content
                });

                confirm_clear_data_points.continue_button.on_button_click = [data_store, this]()->void
                {
                    std::lock_guard lock_guard(data_store->lock);
                    data_store->points.clear();
                    removeAll3DPoints(MAG_POINT);
                    m_calibration_sphere->visible = false;
                };

                ModalPopup(confirm_clear_data_points);

                if (ImGui::Button("Clear##DataControls"))
                {
                    OpenModalPopup(confirm_clear_data_points);
                }
            }

            ImGui::SameLine();

            {
                ConditionalDisable disabled(data_store->selected_point == nullptr);

                if (ImGui::Button("Delete Point"))
                {
                    // Remove the point from the 3D scene
                    m_visualization_window.scene->remove(data_store->selected_point);

                    // Remove the point from our cached array
                    std::erase_if(data_store->points, [data_store](const std::shared_ptr<threepp::Object3D> &point)->bool
                    {
                        return point.get() == data_store->selected_point;
                    });

                    // Clear the selected point
                    data_store->selected_point = nullptr;

                    // Recalculate the statistics
                    data_store->calculateStatistics();
                }
            }

            ImGui::TextUnformatted("Verify Data: ");

            if (ImGui::Button(data_store->data_mode == Mode::VERIFY ? "Stop###verify_stop_start" : "Start###verify_stop_start"))
            {
                if (data_store->data_mode == Mode::VERIFY)
                {
                    data_store->data_mode = Mode::IDLE;
                }
                else
                {
                    data_store->data_mode = Mode::VERIFY;
                    m_verify_sphere->visible = true;

                    float field_strength = static_cast<float>(data_store->fieldStrength());

                    if (field_strength > 0.0f)
                    {
                        m_verify_sphere->scale.set(field_strength, field_strength, field_strength);
                    }
                }
            }

            ImGui::SameLine();

            {
                ConditionalDisable disabled(data_store->data_mode == Mode::VERIFY || data_store->verify_points.size() == 0);

                if (ImGui::Button(data_store->verify_hide ? "Show###verify_show_hide" : "Hide###verify_show_hide"))
                {
                    data_store->verify_hide = !data_store->verify_hide;

                    for (std::shared_ptr object : m_visualization_window.scene->children)
                    {
                        if (object->name == VERIFY_POINT)
                        {
                            object->visible = !data_store->verify_hide;
                        }
                    }

                    m_verify_sphere->visible = !data_store->verify_hide;
                }

                ImGui::SameLine();

                //Clear verify points popup
                ModalPopupState confirm_clear_verify_points({
                    { "Clear Verify Data##MagCalPanel" },                 // Title
                    { "Are you sure you want to clear the verify data?" } // Content
                });

                confirm_clear_verify_points.continue_button.on_button_click = [data_store, this]()->void
                {
                    std::lock_guard lock_guard(data_store->lock);
                    data_store->verify_points.clear();
                    removeAll3DPoints(VERIFY_POINT);

                    if (data_store->data_mode != Mode::VERIFY)
                    {
                        m_verify_sphere->visible = false;
                    }
                };

                ModalPopup(confirm_clear_verify_points);

                if (ImGui::Button("Clear##VerifyControls"))
                {
                    OpenModalPopup(confirm_clear_verify_points);
                }
            }
        }

        //
        // Device Calibration
        //
        {
            SectionHeading("Device Calibration", "user_manual_content/installation/Magnetometer%20Calibration.htm");

            const char *calibration_options[] = { "Current Calibration", "Spherical Fit", "Ellipsoidal Fit" };

            if (ImGui::Combo("##CalibrationSelector", &data_store->selected_calibration, calibration_options, IM_ARRAYSIZE(calibration_options)))
            {
                updateCalibration(device);
            }

            ImGui::SameLine();
            {
                Icon iconStyle;

                if (theme().action_button() >> ImGui::Button(icon_reload.c_str()))
                {
                    updateCalibration(device);
                }
            }

            if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal))
            {
                SetTopMostTooltip("Click to recalculate fit");
            }

            checkbox("Specify Field Strength", &data_store->use_custom_field_strength);

            if (data_store->use_custom_field_strength)
            {
                ImGui::Separator();

                if (data_store->custom_field_strength_gauss < 0)
                {
                    ImGui::PushStyleColor(ImGuiCol_Text, RED.Value);
                }
                else
                {
                    ImGui::PushStyleColor(ImGuiCol_Text, BLACK.Value);
                }

                NumberInput(data_store->field_strength_config, data_store->custom_field_strength_gauss);

                ImGui::PopStyleColor();

                ImGui::AlignTextToFramePadding();
                {
                    ImGui::TextUnformatted("Get from Web");
                    Application::FontManager::underline();
                }

                // Web query requires latitude/longitude
                const int TABLE_FLAGS = ImGuiTableFlags_NoBordersInBody | ImGuiTableFlags_NoHostExtendX | ImGuiTableFlags_SizingStretchProp;

                if (ImGui::BeginTable("WebQueryFieldStrength", 2, TABLE_FLAGS))
                {
                    ImGui::TableNextRow();

                    // User-selected Latitude
                    NumberInput(data_store->latitude_config, data_store->latitude, nullptr, true);
                    ImGui::TableNextRow();

                    // User-selected Longitude
                    NumberInput(data_store->longitude_config, data_store->longitude, nullptr, true);
                    ImGui::EndTable();
                }

                // Get Field Strength Button
                if (ImGui::Button("Get From Web"))
                {
                    std::string url = "https://www.ngdc.noaa.gov/geomag-web/calculators/calculateIgrfwmm?lat1=" +
                        std::to_string(data_store->latitude) + "&lon1=" + std::to_string(data_store->longitude) +
                        "&key=EAU2y&resultFormat=json";

                    std::string response_data;

                    long http_response = m_curl.get(url, response_data);

                    // Successful read
                    if (http_response == 200)
                    {
                        bool has_error = false;

                        // Try to parse the response as JSON
                        try
                        {
                            Json json = Json::parse(response_data);

                            // Try to extract the field strength
                            if (!json["result"][0]["totalintensity"].empty())
                            {
                                json["result"][0]["totalintensity"].get_to(data_store->custom_field_strength_gauss);

                                // nT->Gauss
                                data_store->custom_field_strength_gauss *= 1.0e-5;
                            }
                            else
                            {
                                has_error = true;
                            }
                        }
                        catch (const Json::exception &ex)
                        {
                            Application::logError("Exception = %s", ex.what());
                            has_error = true;
                        }

                        // Provide feedback
                        if (has_error)
                        {
                            Application::addNotification(Message::error(
                                "Field Strength from Web",
                                "The server response could not be parsed."
                            ));
                        }
                        else
                        {
                            Application::addNotification(Message::success(
                                "Field Strength from Web",
                                "Read the field strength"
                            ));
                        }
                    }
                    else
                    {
                        Application::addNotification(Message::error(
                            "Field Strength from Web",
                            "The HTTP request failed with code " + std::to_string(http_response)
                        ));
                    }
                }

                ImGui::Separator();
            }

            mip::commands_3dm::MagSoftIronMatrix *displayed_matrix;
            mip::commands_3dm::MagHardIronOffset *displayed_offset;
            mip::commands_3dm::MagSoftIronMatrix matrix;
            mip::commands_3dm::MagHardIronOffset offset;

            std::string status_text;
            ImColor     status_color = BLACK;

            //Get the calibration from the device config
            if (data_store->selected_calibration == CURRENT_CALIBRATION)
            {
                if (device.config().getConfigOnDevice<mip::commands_3dm::MagSoftIronMatrix>(matrix) &&
                    device.config().getConfigOnDevice<mip::commands_3dm::MagHardIronOffset>(offset))
                {
                    displayed_matrix = &matrix;
                    displayed_offset = &offset;
                }

                status_text = "N/A";
            }
            // A computed matrix values
            else
            {
                displayed_matrix = data_store->cached_matrix;
                displayed_offset = data_store->cached_offset;

                if (data_store->fit_status == FitStatus::NONE)
                {
                    status_text = "None";
                }
                else if (data_store->fit_status == FitStatus::INVALID)
                {
                    status_text  = "Invalid";
                    status_color = RED;
                }

                if (data_store->fit_status == FitStatus::VALID)
                {
                    status_text = "Valid";
                    status_color = GREEN;
                }
            }

            // Fit status
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted("Status: ");
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Text, status_color.Value);
            ImGui::TextUnformatted(status_text.c_str());
            ImGui::PopStyleColor();

            ImGui::AlignTextToFramePadding();

            NumberOutput("RMSE:", data_store->fit_rmse);
            ImGui::SameLine();
            ImGui::TextUnformatted(" gauss");


            // Disabled user changes if we are displaying the current calibration
            {
                ConditionalDisable disabled(data_store->selected_calibration == CURRENT_CALIBRATION);

                float available_width = ImGui::GetContentRegionAvail().x;

                // Calibration values
                ImGui::BeginGroup();

                ImGui::TextUnformatted("Matrix");

                data_store->curr_matrix_config->outer_size = ImVec2(available_width * 2.0f / 3.0f, 0.0f);

                if (NumberInput(data_store->curr_matrix_config, displayed_matrix->offset, 3, 3, nullptr, false))
                {
                    *data_store->cached_matrix_changed = true;
                    has_changes = true;
                }

                ImGui::EndGroup();

                ImGui::SameLine(0.0f, ImGui::GetStyle().ItemSpacing.x * 2.0f);

                ImGui::BeginGroup();

                ImGui::TextUnformatted("Offset");
                data_store->curr_offset_config->outer_size = ImVec2(available_width / 3.0f, 0.0f);

                if (NumberInput(data_store->curr_offset_config, displayed_offset->offset, 3))
                {
                    *data_store->cached_offset_changed = true;
                    has_changes = true;
                }

                ImGui::EndGroup();

                // Update the displayed calibration if user changes have occurred
                if (has_changes)
                {
                    updateCalibration(device, true);
                }

                if (theme().action_button() >> ImGui::Button("Reset"))
                {
                    data_store->cached_matrix->offset = {
                        1.0f, 0.0f, 0.0f,
                        0.0f, 1.0f, 0.0f,
                        0.0f, 0.0f, 1.0f
                    };
                    data_store->cached_offset->offset = {0.0f, 0.0f, 0.0f};
                    updateCalibration(device, true);
                }
            }
        }
        return has_changes;
    }

    void MagCalSection::updateCalibration(Device::MipDevice &device, const bool user_changed) const
    {
        std::any store;

        if (!device.getCustomDataStore(this, store))
        {
            return;
        }

        const MagCalDataStorePtr data_store = std::any_cast<MagCalDataStorePtr>(store);

        mip::commands_3dm::MagSoftIronMatrix selected_matrix;
        mip::commands_3dm::MagHardIronOffset selected_offset;

        if (data_store->selected_calibration == SPHERICAL_CALIBRATION)
        {
            if (!user_changed)
            {
                data_store->calculateSphericalFit();
            }

            selected_matrix = *data_store->cached_matrix;
            selected_offset = *data_store->cached_offset;
        }
        else if (data_store->selected_calibration == ELLIPSOIDAL_CALIBRATION)
        {
            if (!user_changed)
            {
                data_store->calculateEllipsoidalFit();
            }

            selected_matrix = *data_store->cached_matrix;
            selected_offset = *data_store->cached_offset;
        }

        data_store->calculateFitRMSE(selected_matrix, selected_offset);

        // Set the calibration sphere
        // Transformation for display is inverse of soft iron calibration matrix
        threepp::Vector3 offset;
        offset.fromArray(selected_offset.offset);

        threepp::Matrix3 trans;
        trans.fromArray(selected_matrix.offset);

        threepp::Matrix4 trans_inv;
        trans_inv.setFromMatrix3(trans.invert());

        // Add scale of current field
        threepp::Matrix4 scale;
        const float field_strength = static_cast<float>(data_store->fieldStrength());
        scale.makeScale(field_strength, field_strength, field_strength);

        const threepp::Matrix4 final_matrix = trans_inv.multiply(scale);

        // Update the 3D display

        // Reset sphere state
        m_calibration_sphere->position = threepp::Vector3::ZEROS();
        m_calibration_sphere->scale    = threepp::Vector3::ONES();
        m_calibration_sphere->quaternion.identity();

        // Apply rotation and offset and set visibility
        m_calibration_sphere->applyMatrix4(final_matrix);
        m_calibration_sphere->position += offset;
        m_calibration_sphere->visible  = !data_store->collect_hide;
   }

    bool MagCalSection::drawDetailPanel(Device::MipDevice &device)
    {
        // Get the data store attached to this device
        std::any store;

        if (!device.getCustomDataStore(this, store))
        {
            return false;
        }

        const MagCalDataStorePtr data_store = std::any_cast<MagCalDataStorePtr>(store);

        // Allow for raycasting to select objects
        threepp::Raycaster raycaster;
        raycaster.params.lineThreshold = 0.1f;

        const ImVec2 size = ImGui::GetContentRegionAvail();

        // 3D View update function (do raycasting logic)
        const std::function update_function = [&, this]()->void
        {
            const ImVec2 mouse_pos_vis = m_visualization_window.getMousePos();

            // Mouse Pos must be in normalized coordinates of [-1, 1] (of course)
            threepp::Vector2 mouse_pos;

            mouse_pos.x = mouse_pos_vis.x / size.x * 2.0f - 1.0f;
            mouse_pos.y = -mouse_pos_vis.y / size.y * 2.0f + 1.0f;

            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            {
                data_store->selected_point = nullptr;

                //Clear out any selected points
                for (int i = 0; i < data_store->points.size(); i++)
                {
                    threepp::Mesh *mesh = dynamic_cast<threepp::Mesh *>(data_store->points[i].get());

                    if (mesh->material() == m_mag_point_highlight_material.get())
                    {
                        mesh->setMaterial(m_mag_point_material);
                    }
                }

                raycaster.setFromCamera(mouse_pos, m_visualization_window.camera);
                const std::vector<threepp::Intersection> intersects = raycaster.intersectObjects(m_visualization_window.scene->children);

                // Set the material of the first intersected mag point
                for (const threepp::Intersection intersect : intersects)
                {
                    if (intersect.object->name != MAG_POINT)
                    {
                        continue;
                    }

                    threepp::Mesh *object = dynamic_cast<threepp::Mesh *>(intersect.object);

                    object->setMaterial(m_mag_point_highlight_material);

                    // Allow deletion of the point
                    data_store->selected_point = object;

                    // Only alter the first point along the ray
                    break;
                }
            }
        };

        // Draw the ThreePP view
        {
            std::lock_guard lock_guard(data_store->lock);
            ThreePP(m_visualization_window, update_function, static_cast<int>(size.x), static_cast<int>(size.y));
        }

        return true;
    }

    std::shared_ptr<threepp::Object3D> MagCalSection::add3DPoint(const threepp::Vector3 pos, const std::string &type) const
    {
        std::shared_ptr<threepp::MeshStandardMaterial> material = nullptr;

        if (type == MAG_POINT)
        {
            material = m_mag_point_material;
        }
        else if (type == VERIFY_POINT)
        {
            material = m_verify_point_material;
        }
        else
        {
            return nullptr;
        }

        std::shared_ptr<threepp::Mesh> mesh = threepp::Mesh::create();
        mesh->setGeometry(m_point_geometry);
        mesh->setMaterial(material);
        mesh->position   = pos;
        mesh->name       = type;

        m_visualization_window.scene->add(mesh);

        return mesh;
    }

    void MagCalSection::removeAll3DPoints(std::string type) const
    {
        std::vector<std::shared_ptr<threepp::Object3D>> &objects = m_visualization_window.scene->children;

        std::erase_if(objects, [type](const std::shared_ptr<threepp::Object3D> &object)->bool
        {
            return object->name == type;
        });
    }
} // namespace Widget
