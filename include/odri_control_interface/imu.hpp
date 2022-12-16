/**
 * @file imu.hpp
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-12-05
 *
 * @brief IMU abstraction.
 */

#pragma once

#include <math.h>
#include <unistd.h>
#include <stdexcept>
#include <memory>

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

#include <odri_control_interface/common.hpp>

namespace odri_control_interface
{
/**
 * @brief Class for dealing with the IMU.
 */
class IMU
{
protected:
    std::shared_ptr<MasterBoardInterface> robot_if_;
    std::array<int, 3> rotate_vector_;
    std::array<int, 4> orientation_vector_;

    // Cache for the results.
    Eigen::Vector3d gyroscope_;
    Eigen::Vector3d accelerometer_;
    Eigen::Vector3d linear_acceleration_;
    Eigen::Vector3d attitude_euler_;
    Eigen::Vector4d attitude_quaternion_;

public:
    IMU(const std::shared_ptr<MasterBoardInterface>& robot_if,
        RefVectorXl rotate_vector,
        RefVectorXl orientation_vector);

    IMU(const std::shared_ptr<MasterBoardInterface>& robot_if);

    // If needed, add some error handling for the IMU as well.
    // For instance, check for bounds on the maximum linear acceleration
    // or the maximum angular velocity that should be detected as an error.
    bool HasError()
    {
        return false;
    }

    void ParseSensorData();

    const std::shared_ptr<MasterBoardInterface>& GetMasterBoardInterface();
    const Eigen::Vector3d& GetGyroscope();
    const Eigen::Vector3d& GetAccelerometer();
    const Eigen::Vector3d& GetLinearAcceleration();
    const Eigen::Vector3d& GetAttitudeEuler();
    const Eigen::Vector4d& GetAttitudeQuaternion();
};

}  // namespace odri_control_interface
