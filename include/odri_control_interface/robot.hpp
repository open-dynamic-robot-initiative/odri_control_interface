/**
 * @file robot.hpp
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-11-27
 *
 * @brief Main robot interface from the package.
 */

#pragma once

#include <unistd.h>

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

#include <odri_control_interface/device.hpp>
#include <odri_control_interface/imu.hpp>
#include <odri_control_interface/joint_modules.hpp>

namespace odri_control_interface
{
/**
 * @brief Class abstracting the blmc motors to modules.
 */
template <int COUNT>
class Robot: Device
{
public:
    std::shared_ptr<MasterBoardInterface> robot_if;
    std::shared_ptr<JointModules<COUNT> > joints;
    std::shared_ptr<IMU> imu;


    Robot(
        std::shared_ptr<MasterBoardInterface> robot_if,
        std::shared_ptr<JointModules<COUNT> > joint_modules,
        std::shared_ptr<IMU> imu
    ): robot_if(robot_if), joints(joint_modules), imu(imu),
        saw_error_(false)
    {
    }

    /**
     * @brief Initializes the session and blocks until either the package
     *   got acknowledged or the communication timed out.
     */
    void Start()
    {
        // Init the robot.
        robot_if->Init();

        // Enable the joints.
        joints->Enable();

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

    /**
     * @brief If no error happend, send the previously specified commands
     *   to the robot. If an error was detected, go into safety mode
     *   and apply the safety control from the joint_module.
     */
    bool SendCommand()
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
    void ParseSensorData()
    {
        robot_if->ParseSensorData();
    };

    /**
     * @brief Way to report an external error. Causes the robot to go into
     *   safety mode.
     */
    void ReportError(std::string error);

    /**
     * @brief Returns true if all connected devices report ready.
     */
    bool IsReady()
    {
        return joints->IsReady();
    }

    bool IsTimeout()
    {
        return robot_if->IsTimeout();
    }

    /**
     * @brief Checks all connected devices for errors. Also checks
     *  if there is a timeout.
     */
    bool HasError()
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
protected:
    int timeout_counter_;
    bool saw_error_;
    std::ostream& msg_out_ = std::cout;
};

}  // namespace odri_control_interface
