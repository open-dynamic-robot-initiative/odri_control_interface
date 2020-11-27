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

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

namespace odri_control_interface
{
/**
 * @brief Class abstracting the blmc motors to modules.
 */
template <int COUNT>
class JointModules: Device
{
public:
    JointModules(
        std::shared_ptr<MasterBoardInterface> robot_if,
        std::array<int, COUNT>& motor_number,
        double motor_constants,
        double gear_ratios,
        double max_currents,
        std::array<bool, COUNT> reverse_polarities,
        std::array<double, COUNT> lower_joint_limit,
        std::array<double, COUNT> upper_joint_limit,
        std::array<double, COUNT> max_joint_velocity,
        std::array<double, COUNT> safety_damping
    );

    void enable();

    void set_torques(const std::array<double, COUNT>& desired_torques);
    void set_desired_positions(const std::array<double, COUNT>& desired_positions);
    void set_desired_velocities(const std::array<double, COUNT>& desired_velocities);
    void set_position_gains(const std::array<double, COUNT>& desired_gains);
    void set_velocity_gains(const std::array<double, COUNT>& desired_gains);

    /**
     * @brief Overwrites the control commands for a default safety controller.
     * The safety controller applies a D control to all the joints based
     * on the provided `safety_damping`.
     */
    virtual void set_safety_controller();

    // Used for calibration.
    void set_position_offset(const std::array<double, COUNT>& desired_gains);

    // During calibration, the joint limit check should be disabled.
    void disable_joint_limit_check();
    void enable_joint_limit_check();

    void enable_index_offset_compensation();

    /**
     * @brief Returns true once all motors are enabled and report ready.
     */
    bool is_ready();

    std::array<double, COUNT> get_sent_torques();
    std::array<double, COUNT> get_measured_torques();
    std::array<double, COUNT> get_positions();
    std::array<double, COUNT> get_velocities();

    /**
     * @brief Checks for errors and prints them
     */
    bool has_error();
};

}  // namespace odri_control_interface
