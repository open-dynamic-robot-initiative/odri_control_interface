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

#include <stdexcept>   // for exception, runtime_error, out_of_range

#include "odri_control_interface/robot.hpp"

namespace odri_control_interface
{
Robot::Robot(const std::shared_ptr<MasterBoardInterface>& robot_if,
             const std::shared_ptr<JointModules>& joint_modules,
             const std::shared_ptr<IMU>& imu,
             const std::shared_ptr<Powerboard>& powerboard,
             const std::shared_ptr<JointCalibrator>& calibrator)
    : robot_if(robot_if), joints(joint_modules), imu(imu),
      powerboard(powerboard), calibrator(calibrator),
      saw_error_(false)
{
    last_time_ = std::chrono::system_clock::now();
}

const std::shared_ptr<MasterBoardInterface>& Robot::GetRobotInterface()
{
    return robot_if;
}

const std::shared_ptr<JointModules>& Robot::GetJoints()
{
    return joints;
}

const std::shared_ptr<IMU>& Robot::GetIMU()
{
    return imu;
}

const std::shared_ptr<Powerboard>& Robot::GetPowerboard()
{
    return powerboard;
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
    std::chrono::time_point<std::chrono::system_clock> last =
        std::chrono::system_clock::now();
    while (!robot_if->IsTimeout() && !robot_if->IsAckMsgReceived())
    {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now() -
                                             last))
                .count() > 0.001)
        {
            last = std::chrono::system_clock::now();
            robot_if->SendInit();
        }
    }

    if (robot_if->IsTimeout())
    {
        throw std::runtime_error("Timeout during Robot::Start().");
    }

    // Parse the sensor data to make sure all fields are filled properly when
    // the user starts using the robot object.
    ParseSensorData();
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
 * @brief
 */
bool Robot::SendCommandAndWaitEndOfCycle(double dt)
{
    bool result = SendCommand();

    while (((std::chrono::duration<double>)(std::chrono::system_clock::now() -
                                            last_time_))
               .count() < dt)
    {
        std::this_thread::yield();
    }
    last_time_ = std::chrono::system_clock::now();

    return result;
}

/**
 *
 */
void Robot::ParseSensorData()
{
    robot_if->ParseSensorData();
    joints->ParseSensorData();

    if (imu)
    {
        imu->ParseSensorData();
    }
    if (powerboard)
    {
        powerboard->ParseSensorData();
    }
}

bool Robot::RunCalibration(const std::shared_ptr<JointCalibrator>& calibrator,
                           VectorXd const& target_positions)
{
    bool is_done = false;
    if (target_positions.size() != joints->GetNumberMotors())
    {
        throw std::runtime_error(
            "Target position vector has a different size than the "
            "number of motors.");
    }
    while (!IsTimeout())
    {
        ParseSensorData();

        is_done = calibrator->RunAndGoTo(target_positions);

        if (is_done)
        {
            return true;
        }

        if (!SendCommandAndWaitEndOfCycle(calibrator->dt()))
        {
            throw std::runtime_error("Error during Robot::RunCalibration().");
        }
    }

    throw std::runtime_error("Timeout during Robot::RunCalibration().");
    return false;
}

bool Robot::RunCalibration(VectorXd const& target_positions)
{
    return RunCalibration(calibrator, target_positions);
}

/**
 * @brief Way to report an external error. Causes the robot to go into
 *   safety mode.
 */
void Robot::ReportError(const std::string& error)
{
    msg_out_ << "ERROR: " << error << std::endl;
    ReportError();
}

/**
 * @brief Way to report an external error. Causes the robot to go into
 *   safety mode.
 */
void Robot::ReportError()
{
    saw_error_ = true;
}

/**
 * @brief Returns true if all connected devices report ready.
 */
bool Robot::IsReady()
{
    return joints->IsReady();
}

bool Robot::WaitUntilReady()
{
    ParseSensorData();
    joints->SetZeroCommands();

    std::chrono::time_point<std::chrono::system_clock> last =
        std::chrono::system_clock::now();
    while (!IsReady() && !HasError())
    {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now() -
                                             last))
                .count() > 0.001)
        {
            last += std::chrono::milliseconds(1);
            if (!IsAckMsgReceived()) {
                SendInit();
            } else {
                ParseSensorData();
                SendCommand();
            }
        }
        else
        {
            std::this_thread::yield();
        }
    }

    if (HasError()) {
        if (robot_if->IsTimeout())
        {
            throw std::runtime_error("Timeout during Robot::WaitUntilReady().");
        } else {
            throw std::runtime_error("Error during Robot::WaitUntilReady().");
        }
    }

    return !saw_error_;
}

void Robot::Initialize(VectorXd const& target_positions)
{
    Start();
    WaitUntilReady();
    RunCalibration(target_positions);
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
    if (imu)
    {
        saw_error_ |= imu->HasError();
    }
    if (powerboard)
    {
        saw_error_ |= powerboard->HasError();
    }

    if (robot_if->IsTimeout())
    {
        if (timeout_counter_++ % 2000 == 0)
        {
            msg_out_ << "ERROR: Robot communication timedout." << std::endl;
        }
        saw_error_ = true;
    }

    return saw_error_;
}

}  // namespace odri_control_interface
