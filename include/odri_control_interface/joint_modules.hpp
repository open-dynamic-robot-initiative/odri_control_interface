/**
 * @file joint_modules.hpp
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-11-27
 *
 * @brief Joint module abstraction for
 */

#pragma once

#include <unistd.h>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

#include <fmt/format.h>

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

#include <odri_control_interface/common.hpp>
#include <odri_control_interface/error.hpp>

namespace odri_control_interface
{
class JointVelocityLimitError : public Error
{
public:
    int joint_index = -1;
    double velocity = 0.0;
    double limit = 0.0;

    JointVelocityLimitError()
    {
    }

    JointVelocityLimitError(int joint_index, double velocity, double limit)
        : joint_index(joint_index), velocity(velocity), limit(limit)
    {
    }

    std::string get_message() const override
    {
        return fmt::format("Joint #{} has velocity {} which exceeds limit ({})",
                           joint_index,
                           velocity,
                           limit);
    }
};

class MotorDriverError : public Error
{
public:
    int motor_driver_index = -1;
    int error_code = -1;

    MotorDriverError()
    {
    }

    MotorDriverError(int motor_driver_index, int error_code)
        : motor_driver_index(motor_driver_index), error_code(error_code)
    {
    }

    std::string get_message() const override
    {
        std::string msg;
        switch (error_code)
        {
            case UD_SENSOR_STATUS_ERROR_ENCODER1:
                msg = "Encoder A error";
                break;
            case UD_SENSOR_STATUS_ERROR_SPI_RECV_TIMEOUT:
                msg = "SPI Receiver timeout";
                break;
            case UD_SENSOR_STATUS_ERROR_CRIT_TEMP:
                msg = "Critical temperature";
                break;
            case UD_SENSOR_STATUS_ERROR_POSCONV:
                msg = "SpinTAC Positon module";
                break;
            case UD_SENSOR_STATUS_ERROR_POS_ROLLOVER:
                msg = "Position rollover occured";
                break;
            case UD_SENSOR_STATUS_ERROR_ENCODER2:
                msg = "Encoder B error";
                break;
            /*
            case UD_SENSOR_STATUS_CRC_ERROR:
                msg = "CRC error in SPI transaction";
                break;
            */
            default:
                msg = "Other error";
                break;
        }

        return fmt::format(
            "Motor Driver #{}: [{}] {}", motor_driver_index, error_code, msg);
    }
};

/**
 * @brief Class abstracting the blmc motors to modules.
 */
class JointModules
{
protected:
    std::shared_ptr<MasterBoardInterface> robot_if_;
    std::vector<Motor*> motors_;

    Eigen::VectorXd gear_ratios_;
    Eigen::VectorXd motor_constants_;
    VectorXi polarities_;
    Eigen::VectorXd lower_joint_limits_;
    Eigen::VectorXd upper_joint_limits_;
    Eigen::VectorXd safety_damping_;

    // Cache for results.
    Eigen::VectorXd positions_;
    Eigen::VectorXd velocities_;
    Eigen::VectorXd sent_torques_;
    Eigen::VectorXd measured_torques_;

    // Cache for status bits.
    VectorXb index_been_detected_;
    VectorXb ready_;
    VectorXb enabled_;
    VectorXb motor_driver_enabled_;
    VectorXi motor_driver_errors_;

    Eigen::VectorXd zero_vector_;

    double max_joint_velocities_;

    int n_;
    int nd_;

    bool check_joint_limits_;

    std::ostream& msg_out_ = std::cout;

public:
    JointModules(const std::shared_ptr<MasterBoardInterface>& robot_if,
                 ConstRefVectorXi motor_numbers,
                 double motor_constants,
                 double gear_ratios,
                 double max_currents,
                 ConstRefVectorXb reverse_polarities,
                 ConstRefVectorXd lower_joint_limits,
                 ConstRefVectorXd upper_joint_limits,
                 double max_joint_velocities,
                 double safety_damping);
    virtual ~JointModules();

    void Enable();

    void ParseSensorData();

    void SetTorques(ConstRefVectorXd desired_torques);
    void SetDesiredPositions(ConstRefVectorXd desired_positions);
    void SetDesiredVelocities(ConstRefVectorXd desired_velocities);
    void SetPositionGains(ConstRefVectorXd desired_gains);
    void SetVelocityGains(ConstRefVectorXd desired_gains);
    void SetMaximumCurrents(double max_currents);

    /**
     * @brief Disables the position and velocity gains by setting them to zero.
     */
    void SetZeroGains();
    void SetZeroCommands();

    /**
     * @brief Overwrites the control commands for a default safety controller.
     * The safety controller applies a D control to all the joints based
     * on the provided `safety_damping`.
     */
    virtual void RunSafetyController();

    // Used for calibration.
    void SetPositionOffsets(ConstRefVectorXd position_offsets);

    void EnableIndexOffsetCompensation();
    void EnableIndexOffsetCompensation(int);

    const VectorXb& HasIndexBeenDetected();
    const VectorXb& GetReady();
    const VectorXb& GetEnabled();
    const VectorXb& GetMotorDriverEnabled();
    const VectorXi& GetMotorDriverErrors();

    bool SawAllIndices();

    /**
     * @brief Returns true once all motors are enabled and report ready.
     */
    bool IsReady();

    const VectorXd& GetPositions();
    const VectorXd& GetVelocities();
    const VectorXd& GetSentTorques();
    const VectorXd& GetMeasuredTorques();

    const VectorXd& GetGearRatios();

    int GetNumberMotors();

    void DisableJointLimitCheck();
    void EnableJointLimitCheck();

    /**
     * @brief Checks for errors and prints them
     */
    bool HasError();

    /**
     * @brief Get error if there is one.
     *
     * If there are multiple errors, only the first one that is detected is
     * returned.
     *
     * @return Error message if there is an error.
     */
    std::optional<ErrorMessage> GetError();

    void PrintVector(ConstRefVectorXd vector);

protected:
    int upper_joint_limits_counter_;
    int lower_joint_limits_counter_;
    int velocity_joint_limits_counter_;
    int motor_drivers_error_counter;
};

}  // namespace odri_control_interface
