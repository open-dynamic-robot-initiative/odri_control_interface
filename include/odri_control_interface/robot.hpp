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

#include <odri_control_interface/calibration.hpp>
#include <odri_control_interface/imu.hpp>
#include <odri_control_interface/joint_modules.hpp>

namespace odri_control_interface
{
/**
 * @brief Class abstracting the blmc motors to modules.
 */
class Robot
{
public:
    std::shared_ptr<MasterBoardInterface> robot_if;
    std::shared_ptr<JointModules> joints;
    std::shared_ptr<IMU> imu;
    std::shared_ptr<JointCalibrator> calibrator;

protected:
    int timeout_counter_;
    bool saw_error_;
    std::ostream& msg_out_ = std::cout;
    std::chrono::time_point<std::chrono::system_clock> last_time_;

public:
    Robot(const std::shared_ptr<MasterBoardInterface>& robot_if,
          const std::shared_ptr<JointModules>& joint_modules,
          const std::shared_ptr<IMU>& imu,
          const std::shared_ptr<JointCalibrator>& calibrator);

    /**
     * @brief Returns the underlying robot interface
     */
    const std::shared_ptr<MasterBoardInterface>& GetRobotInterface();

    /**
     * @brief Returns the joint module.
     */
    const std::shared_ptr<JointModules>& GetJoints();

    /**
     * @brief Return the IMU.
     */
    const std::shared_ptr<IMU>& GetIMU();

    /**
     * @brief Initializes the connection. Use `SendInit` to initialize the
     * session.
     */
    void Init();

    /**
     * @brief Establishes the session with the robot interface.
     */
    void SendInit();

    /**
     * @brief Initializes the session and blocks until either the package
     *   got acknowledged or the communication timed out.
     */
    void Start();

    /**
     * @brief Initializes the robot. This inclues establishing the communication
     * to the robot, wait until the joints are all ready, running the
     * calibration procedure and waiting in the desired initial configuration.
     *
     * This command corresponds to running `Start()`, `WaitUntilReady()` and
     * `RunCalibration(target_positions)` in sequence.
     */
    void Initialize(VectorXd const& target_positions);

    /**
     * @brief If no error happend, send the previously specified commands
     *   to the robot. If an error was detected, go into safety mode
     *   and apply the safety control from the joint_module.
     */
    bool SendCommand();

    /**
     * @brief Same as SendCommand but waits till the end of the control cycle.
     */
    bool SendCommandAndWaitEndOfCycle(double dt);

    /**
     * @brief Parses the sensor data and calls ParseSensorData on all devices.
     */
    void ParseSensorData();

    /**
     * @brief Run the calibration procedure and blocks. Returns true if the
     * calibration procedure finished successfully. Otherwise (e.g. when an
     * error occurred or the communication timed-out) return false.
     */
    bool RunCalibration(const std::shared_ptr<JointCalibrator>& calibrator,
                        VectorXd const& target_positions);

    /**
     * @brief Runs the calibration procedure for the calibrator passed in
     * during initialization and blocks. * calibration procedure finished
     * successfully. Otherwise (e.g. when an error occurred or the communication
     * timed-out) return false.
     */
    bool RunCalibration(VectorXd const& target_positions);

    /**
     * @brief Returns true if all connected devices report ready.
     */
    bool IsReady();

    /**
     * @brief Returns true if the connection timed out.
     */
    bool IsTimeout();

    /**
     * @brief Returns true if during the session initialization the message got
     * acknowledged.
     */
    bool IsAckMsgReceived();

    /**
     * @brief Blocks until all devices report ready.
     */
    bool WaitUntilReady();

    /**
     * @brief Checks all connected devices for errors. Also checks
     *  if there is a timeout.
     */
    bool HasError();

    /**
     * @brief Way to report an external error. Causes the robot to go into
     *   safety mode.
     */
    void ReportError(const std::string& error);

    /**
     * @brief Way to report an external error quietly. Causes the robot to go into
     *   safety mode.
     */
    void ReportError();
};

}  // namespace odri_control_interface
