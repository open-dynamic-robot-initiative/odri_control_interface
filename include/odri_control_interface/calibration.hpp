/**
 * @file joint_modules.hpp
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-11-27
 *
 * @brief Class for calibrating the joints.
 */

#pragma once

#include <math.h>
#include <unistd.h>

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

namespace odri_control_interface
{
enum CalibrationMethod { 
    AUTO,
    POSITIVE,
    NEGATIVE,
    ALTERNATIVE
};

/**
 * @brief 
 */
template <int COUNT>
class JointCalibrator
{
public:
    JointCalibrator(
        std::shared_ptr<JointModules<COUNT> > joints,
        std::array<CalibrationMethod, COUNT>& search_methods,
        std::array<double, COUNT>& position_offsets,
        double Kd, double Kp, double dt
    ): joints_(joints), seach_methods_(search_methods),
        position_offsets_(position_offsets), Kd_(Kd), Kp_(Kp),
        dt_(dt), t_(0.)
    {
        for (int i = 0; i < COUNT; i++)
        {
            if (seach_methods_[i] == AUTO) {
                if (encoder_offsets[i] > M_PI/2.) {
                    seach_methods_[i] = POSITIVE;
                } else if (encoder_offsets[i] < -M_PI/2.) {
                    seach_methods_[i] = NEGATIVE;
                } else {
                    seach_methods_[i] = ALTERNATIVE;
                }
            }
        }
    }

    /**
     * @brief Runs the calibration procedure. Returns true if the calibration is done.
     */
    bool run()
    {
        bool all_motors_done = true;
        if (t_ == 0.) 
        {
            joints->disable_joint_limit_check();
            joints->enable_index_offset_compensation();
            joints->set_zero_gains();
            joints->set_position_offsets(position_offsets_)
            initial_positions_ = joints_->get_positions();
        }

        std::array<double, COUNT> command;
        auto has_index_been_detected = joints->has_index_been_detected();
        auto positions = joints->get_positions();
        auto velocities = joints->get_velocities();
        double T = 2.;
        double des_pos = 0.;
        for (int i = 0; i < COUNT; i++)
        {
            if (has_index_been_detected[i]) 
            {
                command[i] = 0.;
            } else {
                if (seach_methods_ == ALTERNATIVE)
                {
                    if (t_ < T / 2.)
                    {
                        des_pos = initial_positions_[i] + 1.2 * M_PI * 0.5 * (1. - cos(2. * M_PI * (1. / T) * t_));
                    } else {
                        des_pos = initial_positions_[i] + 1.2 * M_PI * cos(2. * M_PI * (0.5 / T)*(t_-T/2.0));
                    }
                } else if (seach_methods_ == POSITIVE) {
                    des_pos = initial_positions_[i] + 1.2 * M_PI * (1. - cos(2. * M_PI * (0.5 / T) * t_));
                } else {
                    des_pos = initial_positions_[i] - 1.2 * M_PI * (1. - cos(2. * M_PI * (0.5 / T) * t_));
                }
                // Compute desired gains using PD controller.
                command[i] = Kp_ * (des_pos - positions[i]) - Kd_ * velocities[i];
            }
        }
        joint->set_torques(command);

        t_ += dt_;
        return is_calibration_done();
    }

    bool is_calibration_done()
    {
        bool all_detected = true;
        std::array<bool, COUNT> has_index_been_detected = joints->has_index_been_detected();
        for (int i = 0; i < COUNT; i++)
        {
            if (has_index_been_detected[i] == false)
            {
                all_detected = false;
                break;
            }
        }
        return all_detected;
    }
};


} // namespace odri_control_interface
