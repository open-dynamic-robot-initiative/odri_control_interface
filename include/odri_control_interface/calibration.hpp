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

#include <algorithm>
#include <math.h>
#include <unistd.h>

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

namespace odri_control_interface
{
enum CalibrationMethod
{
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
    std::array<bool, COUNT> found_index_;
    std::array<double, COUNT> t_end_;
    double Kd_;
    double Kp_;
    double dt_;
    double t_;
    bool go_to_zero_position_;

public:
    JointCalibrator(
        std::shared_ptr<JointModules<COUNT> > joints,
        std::array<CalibrationMethod, COUNT>& search_methods,
        std::array<double, COUNT>& position_offsets,
        double Kp, double Kd, double dt
    ): joints_(joints), search_methods_(search_methods),
        position_offsets_(position_offsets), Kp_(Kp), Kd_(Kd),
        dt_(dt), t_(0.), go_to_zero_position_(false)
    {
        std::array<double, COUNT> gear_ratios = joints->GetGearRatios();
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

    void UpdatePositionOffsets(std::array<double, COUNT>& position_offsets)
    {
        position_offsets_ = position_offsets;
    }

    /**
     * @brief Runs the calibration procedure. Returns true if the calibration is done.
     */
    bool Run()
    {
        double T = 1.;
        if (t_ == 0.)
        {
            joints_->EnableIndexOffsetCompensation();
            joints_->SetZeroGains();
            joints_->SetPositionOffsets(position_offsets_);
            initial_positions_ = joints_->GetPositions();

            // If all the indices are already detected, then assume there
            // is nothing that needs to be done.
            if (joints_->SawAllIndices()) {
                joints_->SetZeroCommand();
                return true;
            }

            joints_->DisableJointLimitCheck();
        }

        std::array<double, COUNT> command;
        auto gear_ratios = joints_->GetGearRatios();
        auto has_index_been_detected = joints_->HasIndexBeenDetected();
        auto positions = joints_->GetPositions();
        auto velocities = joints_->GetVelocities();
        double des_pos = 0.;
        bool finished = true;
        for (int i = 0; i < COUNT; i++)
        {
            // As long as the index was not found, search for it.
            if (!found_index_[i]) {
                if (has_index_been_detected[i])
                {
                    found_index_[i] = true;
                    initial_positions_[i] = positions[i];
                    t_end_[i] = t_ + T;
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
                        des_pos = -2.2 * M_PI * (1. - cos(2. * M_PI * (0.5 / T) * t_));
                    }
                    command[i] = Kp_ * (des_pos/gear_ratios[i] + initial_positions_[i] - positions[i]) - Kd_ * velocities[i];
                }
                finished = false;
            // After the index was found, move to the initial zero position.
            } else {
                if (t_end_[i] > t_) {
                    des_pos = initial_positions_[i] * (t_end_[i] - t_)/T;
                    finished = false;
                } else {
                    des_pos = 0;
                }
                command[i] = Kp_ * (des_pos - positions[i]) - Kd_ * velocities[i];
            }
        }
        joints_->SetTorques(command);

        t_ += dt_;


        if (finished)
        {
            joints_->EnableJointLimitCheck();
        }
        return finished;
    }

    bool IsCalibrationDone()
    {
        return joints_->SawAllIndices();
    }
};


} // namespace odri_control_interface
