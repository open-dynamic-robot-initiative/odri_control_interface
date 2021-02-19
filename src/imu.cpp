/**
 * @file imu.cpp
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-12-05
 *
 * @brief IMU abstraction.
 */

#include "odri_control_interface/imu.hpp"

#include <iostream>

namespace odri_control_interface
{
IMU::IMU(const std::shared_ptr<MasterBoardInterface>& robot_if,
         RefVectorXl rotate_vector,
         RefVectorXl orientation_vector)
    : robot_if_(robot_if)
{
    if (rotate_vector.size() != 3)
    {
        throw std::runtime_error("Expecting rotate_vector of size 3");
    }
    if (orientation_vector.size() != 4)
    {
        throw std::runtime_error("Expecting orientation_vector of size 4");
    }

    for (int i = 0; i < 3; i++)
    {
        rotate_vector_[i] = (int)rotate_vector(i);
    }
    for (int i = 0; i < 4; i++)
    {
        orientation_vector_[i] = (int)orientation_vector(i);
    }
}

IMU::IMU(const std::shared_ptr<MasterBoardInterface>& robot_if)
    : robot_if_(robot_if),
      rotate_vector_({1, 2, 3}),
      orientation_vector_({1, 2, 3, 4})
{
}

const std::shared_ptr<MasterBoardInterface>& IMU::GetMasterBoardInterface()
{
    return robot_if_;
}

RefVector3d IMU::GetGyroscope()
{
    return gyroscope_;
}

RefVector3d IMU::GetAccelerometer()
{
    return accelerometer_;
}

RefVector3d IMU::GetLinearAcceleration()
{
    return linear_acceleration_;
}

RefVector3d IMU::GetAttitudeEuler()
{
    return attitude_euler_;
}

RefVector4d IMU::GetAttitudeQuaternion()
{
    return attitude_quaternion_;
}

void IMU::ParseSensorData()
{
    int index;
    for (int i = 0; i < 3; i++)
    {
        index = rotate_vector_[i];
        if (index < 0)
        {
            gyroscope_(i) = -robot_if_->imu_data_gyroscope(-index - 1);
            accelerometer_(i) = -robot_if_->imu_data_accelerometer(-index - 1);
            linear_acceleration_(i) =
                -robot_if_->imu_data_linear_acceleration(-index - 1);
        }
        else
        {
            gyroscope_(i) = robot_if_->imu_data_gyroscope(index - 1);
            accelerometer_(i) = robot_if_->imu_data_accelerometer(index - 1);
            linear_acceleration_(i) =
                robot_if_->imu_data_linear_acceleration(index - 1);
        }
    }

    std::array<double, 4> attitude;
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
    for (int i = 0; i < 4; i++)
    {
        int index = orientation_vector_[i];
        if (index < 0)
        {
            attitude_quaternion_(i) = -attitude[-index - 1];
        }
        else
        {
            attitude_quaternion_(i) = attitude[index - 1];
        }
    }

    // Convert the rotated quatenion back into euler angles.
    // Source:
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    double qx = attitude_quaternion_(0);
    double qy = attitude_quaternion_(1);
    double qz = attitude_quaternion_(2);
    double qw = attitude_quaternion_(3);

    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    attitude_euler_(0) = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        attitude_euler_(1) =
            std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
    else
        attitude_euler_(1) = std::asin(sinp);

    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    attitude_euler_(2) = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace odri_control_interface
