/**
 * @file joint_modules.cpp
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-12-05
 *
 * @brief Robot class orchistrating the devices.
 */

#include "odri_control_interface/robot.hpp"

namespace odri_control_interface
{

Robot::Robot(
    MasterBoardInterface* robot_if,
    JointModules* joint_modules,
    IMU* imu
): robot_if(robot_if), joints(joint_modules), imu(imu),
    saw_error_(false)
{
}

void Robot::Init()
{
    // Init the robot.
    robot_if->Init();

    // Enable the joints.
    joints->Enable();
}

void Robot::SendInit()
{
    robot_if->SendInit();
}

/**
 * @brief Initializes the session and blocks until either the package
 *   got acknowledged or the communication timed out.
 */
void Robot::Start()
{
    Init();

    // Initiate the communication session.
    std::chrono::time_point<std::chrono::system_clock> last = std::chrono::system_clock::now();
    while (!robot_if->IsTimeout() && !robot_if->IsAckMsgReceived()) {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > 0.001)
        {
            last = std::chrono::system_clock::now();
            robot_if->SendInit();
        }
    }
}

bool Robot::IsAckMsgReceived()
{
    return robot_if->IsAckMsgReceived();
}

/**
 * @brief If no error happend, send the previously specified commands
 *   to the robot. If an error was detected, go into safety mode
 *   and apply the safety control from the joint_module.
 */
bool Robot::SendCommand()
{
    HasError();
    if (saw_error_)
    {
        joints->RunSafetyController();
    }
    robot_if->SendCommand();
    return !saw_error_;
}

/**
 *
 */
void Robot::ParseSensorData()
{
    robot_if->ParseSensorData();
    joints->ParseSensorData();

    if (imu) {
        imu->ParseSensorData();
    }
};

/**
 * @brief Way to report an external error. Causes the robot to go into
 *   safety mode.
 */
void Robot::ReportError(std::string error)
{
    msg_out_ << "ERROR: " << error << std::endl;
    saw_error_ = true;
}

/**
 * @brief Returns true if all connected devices report ready.
 */
bool Robot::IsReady()
{
    return joints->IsReady();
}

void Robot::WaitUntilReady()
{
    std::chrono::time_point<std::chrono::system_clock> last = std::chrono::system_clock::now();
    while (!IsReady() && !HasError())
    {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > 0.001)
        {
            last += std::chrono::milliseconds(1);
            ParseSensorData();
            SendCommand();
        } else {
            std::this_thread::yield();
        }
    }
}

bool Robot::IsTimeout()
{
    return robot_if->IsTimeout();
}

/**
 * @brief Checks all connected devices for errors. Also checks
 *  if there is a timeout.
 */
bool Robot::HasError()
{
    saw_error_ |= joints->HasError();
    if (imu) {
        saw_error_ |= imu->HasError();
    }

    if (robot_if->IsTimeout())
    {
        if (timeout_counter_++ % 2000 == 0)
        {
            msg_out_ << "ERROR: Robot communicaton timedout." << std::endl;
        }
        saw_error_ = true;
    }

    return saw_error_;
}

} // namespace ordi_control_interface


