/**
 * @file calibration.cpp
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-12-05
 *
 * @brief Class for calibrating the joints.
 */

#include "odri_control_interface/calibration.hpp"
#include <iostream>

namespace odri_control_interface
{
JointCalibrator::JointCalibrator(
    const std::shared_ptr<JointModules>& joints,
    const std::vector<CalibrationMethod>& search_methods,
    RefVectorXd position_offsets,
    double Kp,
    double Kd,
    double T,
    double dt)
    : joints_(joints),
      search_methods_(search_methods),
      position_offsets_(position_offsets),
      Kp_(Kp),
      Kd_(Kd),
      T_(T),
      dt_(dt),
      t_(0.),
      go_to_zero_position_(false)
{
    gear_ratios_ = joints->GetGearRatios();
    n_ = static_cast<int>(gear_ratios_.size());

    if (static_cast<int>(search_methods.size()) != n_)
    {
        throw std::runtime_error(
            "Search methods has different size than motor numbers");
    }

    if (static_cast<int>(position_offsets.size()) != n_)
    {
        throw std::runtime_error(
            "Position offsets has different size than motor numbers");
    }

    for (int i = 0; i < n_; i++)
    {
        if (search_methods_[i] == AUTO)
        {
            if (position_offsets[i] > (M_PI / 2.) / gear_ratios_[i])
            {
                search_methods_[i] = POSITIVE;
            }
            else if (position_offsets[i] < (-M_PI / 2.) / gear_ratios_[i])
            {
                search_methods_[i] = NEGATIVE;
            }
            else
            {
                search_methods_[i] = ALTERNATIVE;
            }
        }
    }

    initial_positions_.resize(n_);
    t_end_.resize(n_);
    command_.resize(n_);

    found_index_.resize(n_);
    found_index_.fill(false);
}

void JointCalibrator::UpdatePositionOffsets(RefVectorXd position_offsets)
{
    position_offsets_ = position_offsets;
}

/**
 * @brief Runs the calibration procedure. Returns true if the calibration is
 * done.
 */
bool JointCalibrator::Run()
{
    if (t_ == 0.)
    {
        joints_->EnableIndexOffsetCompensation();
        joints_->SetZeroGains();
        joints_->SetPositionOffsets(position_offsets_);
        initial_positions_ = joints_->GetPositions();

        // If all the indices are already detected, then assume there
        // is nothing that needs to be done.
        if (joints_->SawAllIndices())
        {
            joints_->SetZeroCommands();
            return true;
        }

        joints_->DisableJointLimitCheck();
    }

    auto has_index_been_detected = joints_->HasIndexBeenDetected();
    auto positions = joints_->GetPositions();
    auto velocities = joints_->GetVelocities();
    double des_pos = 0.;
    bool finished = true;
    // std::cout << "JointCalibra::DesPos";
    for (int i = 0; i < n_; i++)
    {
        // As long as the index was not found, search for it.
        if (!found_index_[i])
        {
            if (has_index_been_detected[i])
            {
                found_index_[i] = true;
                initial_positions_[i] = positions[i];
                t_end_[i] = t_ + T_;
            }
            else
            {
                if (search_methods_[i] == ALTERNATIVE)
                {
                    if (t_ < T_ / 2.)
                    {
                        des_pos = 1.2 * M_PI * 0.5 *
                                  (1. - cos(2. * M_PI * (1. / T_) * t_));
                    }
                    else
                    {
                        des_pos = 1.2 * M_PI *
                                  cos(2. * M_PI * (0.5 / T_) * (t_ - T_ / 2.0));
                    }
                }
                else if (search_methods_[i] == POSITIVE)
                {
                    des_pos =
                        2.2 * M_PI * (1. - cos(2. * M_PI * (0.5 / T_) * t_));
                }
                else
                {
                    des_pos =
                        -2.2 * M_PI * (1. - cos(2. * M_PI * (0.5 / T_) * t_));
                }
                command_[i] = Kp_ * (des_pos / gear_ratios_[i] +
                                     initial_positions_[i] - positions[i]) -
                              Kd_ * velocities[i];
                // std::cout << des_pos << " ";
            }
            finished = false;
            // After the index was found, move to the initial zero position.
        }
        else
        {
            if (t_end_[i] > t_)
            {
                des_pos = initial_positions_[i] * (t_end_[i] - t_) / T_;
                finished = false;
            }
            else
            {
                des_pos = 0;
            }
            command_[i] = Kp_ * (des_pos - positions[i]) - Kd_ * velocities[i];
        }
    }

    joints_->SetTorques(command_);
    t_ += dt_;

    if (finished)
    {
        joints_->EnableJointLimitCheck();
    }
    return finished;
}

}  // namespace odri_control_interface
