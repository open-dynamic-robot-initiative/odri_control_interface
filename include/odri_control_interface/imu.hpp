/**
 * @file imu.hpp
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-11-27
 *
 * @brief
 */

#pragma once

#include <unistd.h>

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

namespace odri_control_interface
{
/**
 * @brief Class for dealing with the IMU.
 */
class IMU: Device
{
public:
    IMU(
        std::shared_ptr<MasterBoardInterface> robot_if,
        std::array<double, 3>& rotate_vector,
        std::array<double, 4>& orientation_vector
    )
    {

    };

    // If needed, add some error handling for the IMU as well.
    // For instance, check for bounds on the maximum linear acceleration
    // or the maximum angular velocity that should be detected as an error.
    bool HasError() { return false; }

    std::array<double, 3> GetAngularVelocity();
    std::array<double, 4> GetOrientation();
    std::array<double, 3> GetLinearAcceleration();
};

}  // namespace odri_control_interface
