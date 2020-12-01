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
protected:
    std::shared_ptr<MasterBoardInterface> robot_if_;
    std::array<Motor*, COUNT> motors_;

    std::array<double, COUNT> gear_ratios_;
    std::array<double, COUNT> motor_constants_;
    std::array<double, COUNT> polarities_;
    std::array<double, COUNT> lower_joint_limits_;
    std::array<double, COUNT> upper_joint_limits_;

    double max_joint_velocities_;
    double safety_damping_;

    bool check_joint_limits_;

public:
    JointModules(
        std::shared_ptr<MasterBoardInterface> robot_if,
        std::array<int, COUNT>& motor_numbers,
        double motor_constants,
        double gear_ratios,
        double max_currents,
        std::array<bool, COUNT> reverse_polarities,
        std::array<double, COUNT> lower_joint_limits,
        std::array<double, COUNT> upper_joint_limits,
        double max_joint_velocities,
        double safety_damping
    )
    : Device(),
      robot_if_(robot_if),
      lower_joint_limits_(lower_joint_limits),
      upper_joint_limits_(upper_joint_limits),
      max_joint_velocities_(max_joint_velocities),
      safety_damping_(safety_damping),
      check_joint_limits_(true)
    {
        for (int i = 0; i < COUNT; i++)
        {
            polarities_[i] = reverse_polarities[i] ? -1. : 1.;
            motors_[i] = robot_if_->GetMotor(motor_numbers[i]);
            gear_ratios_[i] = gear_ratios;
            motor_constants_[i] = motor_constants;
        }
        set_maximum_currents(max_currents);
    };

 
    void enable()
    {
        std::array<double, COUNT> zeros;
        for (int i = 0; i < COUNT; i++) {
            zeros[i] = 0.; 
        }

        set_torques(zeros);
        set_desired_positions(zeros);
        set_desired_velocities(zeros);
        set_zero_gains();

        // Enable all motors and cards.
        for (int i = 0; i < (COUNT + 1) / 2; i++)
        {
            robot_if_->motor_drivers[i].motor1->Enable();
            robot_if_->motor_drivers[i].motor2->Enable();
            robot_if_->motor_drivers[i].EnablePositionRolloverError();
            robot_if_->motor_drivers[i].SetTimeout(5);
            robot_if_->motor_drivers[i].Enable();
        }
    };

    void set_torques(const std::array<double, COUNT>& desired_torques)
    {
        for (int i = 0; i < COUNT; i++)
        {
            motors_[i]->SetCurrentReference(
                polarities_[i] * desired_torques[i] / 
                    (gear_ratios_[i] * motor_constants_[i]));
        }
    };

    void set_desired_positions(const std::array<double, COUNT>& desired_positions)
    {
        for (int i = 0; i < COUNT; i++)
        {
            motors_[i]->SetPositionReference(
                polarities_[i] * desired_positions[i] * gear_ratios_[i]);
        }
    };

    void set_desired_velocities(const std::array<double, COUNT>& desired_velocities)
    {
        for (int i = 0; i < COUNT; i++)
        {
            motors_[i]->SetVelocityReference(
                polarities_[i] * desired_velocities[i] * gear_ratios_[i]);
        }
    };

    void set_position_gains(const std::array<double, COUNT>& desired_gains)
    {
       for (int i = 0; i < COUNT; i++)
        {
            motors_[i]->set_kp(
                desired_gains[i] / (gear_ratios_[i] * gear_ratios_[i] * motor_constants_[i]));
        }
    };

    void set_velocity_gains(const std::array<double, COUNT>& desired_gains)
    {
        for (int i = 0; i < COUNT; i++)
        {
            motors_[i]->set_kd(
                desired_gains[i] / (gear_ratios_[i] * gear_ratios_[i] * motor_constants_[i]));
        }
    };

   void set_maximum_currents(double max_currents)
    {
        for (int i = 0; i < COUNT; i++)
        {
            motors_[i]->SetSaturationCurrent(max_currents);
        }
    };

    /**
     * @brief Disables the position and velocity gains by setting them to zero.
     */
    void set_zero_gains()
    {
        std::array<double, COUNT> zeros;
        for (int i = 0; i < COUNT; i++) {
            zeros[i] = 0.; 
        }
        set_position_gains(zeros);
        set_velocity_gains(zeros);
    };

    /**
     * @brief Overwrites the control commands for a default safety controller.
     * The safety controller applies a D control to all the joints based
     * on the provided `safety_damping`.
     */
    virtual void run_safety_controller()
    {
        std::array<double, COUNT> zeros;
        for (int i = 0; i < COUNT; i++) {
            zeros[i] = 0.; 
        }

        set_torques(zeros);
        set_desired_positions(zeros);
        set_desired_velocities(zeros);
        set_zero_gains();

        std::array<double, COUNT> kd_safety;
        for (int i = 0; i < COUNT; i++) {
            kd_safety[i] = safety_damping_;
        }
        set_velocity_gains(kd_safety);
    };

    // Used for calibration.
    void set_position_offsets(const std::array<double, COUNT>& position_offsets)
    {
        for (int i = 0; i < COUNT; i++)
        {
            // REVIEW: Is the following correct?
            motors_[i]->SetPositionOffset(position_offsets[i] * 
                polarities_[i] * gear_ratios_[i]);
        }
    };

    // During calibration, the joint limit check should be disabled.
    void disable_joint_limit_check()
    {
        check_joint_limits_ = false;
    };

    void enable_joint_limit_check()
    {
        check_joint_limits_ = true;
    }

    void enable_index_offset_compensation()
    {
        for (int i = 0; i < COUNT; i++)
        {
            motors_[i]->set_enable_index_offset_compensation(true);
        }
    };

    std::array<bool, COUNT> has_index_been_detected()
    {
        std::array<bool, COUNT> index_detected;
        for (int i = 0; i < COUNT; i++)
        {
            index_detected = motors_[i]->get_has_index_been_detected();
        }
        return index_detected;
    }

    /**
     * @brief Returns true once all motors are enabled and report ready.
     */
    bool is_ready()
    {
        bool is_ready_ = true;

        for (int i = 0; i < COUNT; i++)
        {
            if (!motors_[i]->get_is_enabled() || !motors_[i]->get_is_ready())
            {
                is_ready_ = false;
                break;
            }
        }
        return is_ready_;
    }

    std::array<double, COUNT> get_positions()
    {
        std::array<double, COUNT> positions;
        for (int i = 0; i < COUNT; i++)
        {   
            positions[i] = (motors_[i]->get_position() * 
                polarities_[i] * gear_ratios_[i]);
        }
        return positions;
    };

    std::array<double, COUNT> get_velocities()
    {
        std::array<double, COUNT> velocities;
        for (int i = 0; i < COUNT; i++)
        {   
            velocities[i] = (motors_[i]->get_velocity() * 
                polarities_[i] * gear_ratios_[i]);
        }
        return velocities;
    };

    std::array<double, COUNT> get_sent_torques()
    {
        std::array<double, COUNT> torques;
        for (int i = 0; i < COUNT; i++)
        {   
            torques[i] = (motors_[i]->get_current_ref() * 
                polarities_[i] * gear_ratios_[i] * motor_constants_[i]);
        }
        return torques;
    };

    std::array<double, COUNT> get_measured_torques()
    {
        std::array<double, COUNT> torques;
        for (int i = 0; i < COUNT; i++)
        {   
            torques[i] = (motors_[i]->get_current() * 
                polarities_[i] * gear_ratios_[i] * motor_constants_[i]);
        }
        return torques;
    };

    /**
     * @brief Checks for errors and prints them
     */
    bool has_error()
    {
        return false;
    }
};

}  // namespace odri_control_interface
