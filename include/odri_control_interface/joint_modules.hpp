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

#include <iostream>
#include <vector>
#include <unistd.h>

#include <Eigen/Dense>

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

namespace odri_control_interface
{
typedef Eigen::Matrix<long, Eigen::Dynamic, 1> VectorXl;
typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;

typedef Eigen::Ref<Eigen::Matrix<long, Eigen::Dynamic, 1> > RefVectorXl;
typedef Eigen::Ref<Eigen::Matrix<bool, Eigen::Dynamic, 1> > RefVectorXb;
typedef Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1> > RefVectorXd;

/**
 * @brief Class abstracting the blmc motors to modules.
 */
class JointModules
{
protected:
    MasterBoardInterface* robot_if_;
    std::vector<Motor*> motors_;

    Eigen::VectorXd gear_ratios_;
    Eigen::VectorXd motor_constants_;
    Eigen::Matrix<long, Eigen::Dynamic, 1> polarities_;
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
    VectorXb motor_driver_errors_;

    Eigen::VectorXd zero_vector_;

    double max_joint_velocities_;

    int n_;
    int nd_;

    bool check_joint_limits_;

    std::ostream& msg_out_ = std::cout;

public:
    JointModules(
        MasterBoardInterface* robot_if,
        Eigen::Matrix<long, Eigen::Dynamic, 1> motor_numbers,
        double motor_constants,
        double gear_ratios,
        double max_currents,
        Eigen::Matrix<long, Eigen::Dynamic, 1> motor_polarities,
        Eigen::VectorXd lower_joint_limits,
        Eigen::VectorXd upper_joint_limits,
        double max_joint_velocities,
        double safety_damping
    );

    void Enable();

    void ParseSensorData();

    void SetTorques(const RefVectorXd desired_torques);
    void SetDesiredPositions(const RefVectorXd desired_positions);
    void SetDesiredVelocities(const RefVectorXd desired_velocities);
    void SetPositionGains(const RefVectorXd desired_gains);
    void SetVelocityGains(const RefVectorXd desired_gains);
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
    void SetPositionOffsets(const RefVectorXd position_offsets);

    void EnableIndexOffsetCompensation();

    RefVectorXb HasIndexBeenDetected();
    RefVectorXb GetReady();
    RefVectorXb GetEnabled();
    RefVectorXb GetMotorDriverEnabled();
    RefVectorXb GetMotorDriverErrors();

    bool SawAllIndices();

    /**
     * @brief Returns true once all motors are enabled and report ready.
     */
    bool IsReady();

    RefVectorXd GetPositions();
    RefVectorXd GetVelocities();
    RefVectorXd GetSentTorques();
    RefVectorXd GetMeasuredTorques();

    RefVectorXd GetGearRatios();

    void DisableJointLimitCheck();
    void EnableJointLimitCheck();

    /**
     * @brief Checks for errors and prints them
     */
    bool HasError();

    void PrintVector(RefVectorXd vector);

protected:
    int upper_joint_limits_counter_;
    int lower_joint_limits_counter_;
    int velocity_joint_limits_counter_;
    int motor_drivers_error_counter;
};

}  // namespace odri_control_interface
