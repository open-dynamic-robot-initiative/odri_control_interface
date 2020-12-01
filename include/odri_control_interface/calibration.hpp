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
protected:
    std::shared_ptr<JointModules<COUNT> > joints_;
    std::array<CalibrationMethod, COUNT> search_methods_;
    std::array<double, COUNT> position_offsets_;
    std::array<double, COUNT> initial_positions_;
    double Kd_;
    double Kp_;
    double dt_;
    double t_;

public:
    JointCalibrator(
        std::shared_ptr<JointModules<COUNT> > joints,
        std::array<CalibrationMethod, COUNT>& search_methods,
        std::array<double, COUNT>& position_offsets,
        double Kp, double Kd, double dt
    ): joints_(joints), search_methods_(search_methods),
        position_offsets_(position_offsets), Kp_(Kp), Kd_(Kd),
        dt_(dt), t_(0.)
    {
        std::array<double, COUNT> gear_ratios = joints->get_gear_ratios();
        for (int i = 0; i < COUNT; i++)
        {
            if (search_methods_[i] == AUTO) {
                if (position_offsets[i] > (M_PI/2.) / gear_ratios[i]) {
                    search_methods_[i] = POSITIVE;
                } else if (position_offsets[i] < (-M_PI/2.) / gear_ratios[i]) {
                    search_methods_[i] = NEGATIVE;
                } else {
                    search_methods_[i] = ALTERNATIVE;
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
            joints_->disable_joint_limit_check();
            joints_->enable_index_offset_compensation();
            joints_->set_zero_gains();
            joints_->set_position_offsets(position_offsets_);
            initial_positions_ = joints_->get_positions();
        }

        std::array<double, COUNT> command;
        auto gear_ratios = joints_->get_gear_ratios();
        auto has_index_been_detected = joints_->has_index_been_detected();
        auto positions = joints_->get_positions();
        auto velocities = joints_->get_velocities();
        double T = 2.;
        double des_pos = 0.;
        for (int i = 0; i < COUNT; i++)
        {
            if (has_index_been_detected[i]) 
            {
                command[i] = 0.;
            } else {
                if (search_methods_[i] == ALTERNATIVE)
                {
                    if (t_ < T / 2.)
                    {
                        des_pos = 1.2 * M_PI * 0.5 * (1. - cos(2. * M_PI * (1. / T) * t_));
                    } else {
                        des_pos = 1.2 * M_PI * cos(2. * M_PI * (0.5 / T)*(t_-T/2.0));
                    }
                } else if (search_methods_[i] == POSITIVE) {
                    des_pos = 2.2 * M_PI * (1. - cos(2. * M_PI * (0.5 / T) * t_));
                } else {
                    des_pos = 2.2 * M_PI * (1. - cos(2. * M_PI * (0.5 / T) * t_));
                }
                // Compute desired gains using PD controller.
                command[i] = Kp_ * (des_pos/gear_ratios[i] + initial_positions_[i] - positions[i]) - Kd_ * velocities[i];
            }
        }
        joints_->set_torques(command);

        t_ += dt_;
        return is_calibration_done();
    }

    bool is_calibration_done()
    {
        return joints_->saw_all_indecies();
    }
};


} // namespace odri_control_interface
