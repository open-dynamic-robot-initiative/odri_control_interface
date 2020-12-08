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

#include <odri_control_interface/imu.hpp>
#include <odri_control_interface/joint_modules.hpp>

namespace odri_control_interface
{
/**
 * @brief Class abstracting the blmc motors to modules.
 */
class Robot
{
protected:
    int timeout_counter_;
    bool saw_error_;
    std::ostream& msg_out_ = std::cout;

public:
    MasterBoardInterface* robot_if;
    JointModules* joints;
    IMU* imu;

    Robot(
        MasterBoardInterface* robot_if,
        JointModules* joint_modules,
        IMU* imu
    );

    MasterBoardInterface* GetRobotInterface();

    JointModules* GetJoints();

    IMU* GetIMU();

    void Init();

    void SendInit();

    /**
     * @brief Initializes the session and blocks until either the package
     *   got acknowledged or the communication timed out.
     */
    void Start();

    /**
     * @brief If no error happend, send the previously specified commands
     *   to the robot. If an error was detected, go into safety mode
     *   and apply the safety control from the joint_module.
     */
    bool SendCommand();

    /**
     *
     */
    void ParseSensorData();

    /**
     * @brief Returns true if all connected devices report ready.
     */
    bool IsReady();

    bool IsTimeout();

    bool IsAckMsgReceived();

    void WaitUntilReady();

    /**
     * @brief Checks all connected devices for errors. Also checks
     *  if there is a timeout.
     */
    bool HasError();

    /**
     * @brief Way to report an external error. Causes the robot to go into
     *   safety mode.
     */
    void ReportError(std::string error);
};

}  // namespace odri_control_interface
