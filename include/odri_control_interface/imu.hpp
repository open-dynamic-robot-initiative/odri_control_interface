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

#include <math.h>
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
protected:
    std::shared_ptr<MasterBoardInterface> robot_if_;
    std::array<int, 3> rotate_vector_;
    std::array<int, 4> orientation_vector_;

public:
    IMU(
        std::shared_ptr<MasterBoardInterface> robot_if,
        std::array<int, 3>& rotate_vector,
        std::array<int, 4>& orientation_vector
    ): robot_if_(robot_if),
       rotate_vector_(rotate_vector),
       orientation_vector_(orientation_vector)
    {
    };

    IMU(
        std::shared_ptr<MasterBoardInterface> robot_if
    ): robot_if_(robot_if),
       rotate_vector_({1, 2, 3}),
       orientation_vector_({1, 2, 3, 4})
    {
    };

    // If needed, add some error handling for the IMU as well.
    // For instance, check for bounds on the maximum linear acceleration
    // or the maximum angular velocity that should be detected as an error.
    bool HasError() { return false; }

    std::array<double, 3> GetGyroscope()
    {
        std::array<double, 3> angular_velocity;

        for (int i = 0; i < 3; i++) {
            int index = rotate_vector_[i];
            if (index < 0) {
                angular_velocity[i] = -robot_if_->imu_data_gyroscope(-index + 1);
            } else {
                angular_velocity[i] = robot_if_->imu_data_gyroscope(index - 1);
            }
        }
        return angular_velocity;
    }

    std::array<double, 3> GetAccelerometer()
    {
        std::array<double, 3> accelerometer;

        for (int i = 0; i < 3; i++) {
            int index = rotate_vector_[i];
            if (index < 0) {
                accelerometer[i] = -robot_if_->imu_data_accelerometer(-index + 1);
            } else {
                accelerometer[i] = robot_if_->imu_data_accelerometer(index - 1);
            }
        }
        return accelerometer;
    }

    std::array<double, 3> GetLinearAcceleration()
    {
        std::array<double, 3> linear_acceleration;

        for (int i = 0; i < 3; i++) {
            int index = rotate_vector_[i];
            if (index < 0) {
                linear_acceleration[i] = -robot_if_->imu_data_linear_acceleration(-index + 1);
            } else {
                linear_acceleration[i] = robot_if_->imu_data_linear_acceleration(index - 1);
            }
        }
        return linear_acceleration;
    }

    std::array<double, 4> GetAttitude()
    {
        std::array<double, 4> attitude;
        std::array<double, 4> attitude_rotated;

        double attitude0 = robot_if_->imu_data_attitude(0);
        double attitude1 = robot_if_->imu_data_attitude(1);
        double attitude2 = robot_if_->imu_data_attitude(2);

        double sr = sin(attitude0 / 2.);
        double cr = cos(attitude0 / 2.);
        double sp = sin(attitude1 / 2.);
        double cp = cos(attitude1 / 2.);
        double sy = sin(attitude2 / 2.);
        double cy = cos(attitude2 / 2.);
        attitude[0] = sr * cp * cy - cr * sp * sy;
        attitude[1] = cr * sp * cy + sr * cp * sy;
        attitude[2] = cr * cp * sy - sr * sp * cy;
        attitude[3] = cr * cp * cy + sr * sp * sy;

        // Rotate the attitude.
        for (int i = 0; i < 4; i++) {
            int index = orientation_vector_[i];
            if (index < 0) {
                attitude_rotated[i] = -attitude[-index + 1];
            } else {
                attitude_rotated[i] = attitude[index - 1];
            }
        }

        return attitude_rotated;
    }
};

}  // namespace odri_control_interface
