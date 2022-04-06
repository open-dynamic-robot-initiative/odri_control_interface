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
      T_wait_(0.1),
      dt_(dt),
      t_(0.),
      go_to_zero_position_(false),
      all_indexes_detected_(false),
      t_all_indexes_detected_(0.),
      waiting_time_flag_(false)
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

    found_index_.resize(n_);
    found_index_.fill(false);

    zero_vector_.resize(n_);
    zero_vector_.fill(0.);

    pos_command_.resize(n_);
    vel_command_.resize(n_);
    kp_command_.resize(n_);
    kd_command_.resize(n_);
    pos_command_.fill(0.);
    vel_command_.fill(0.);
    kp_command_.fill(Kp);
    kd_command_.fill(Kd);
}

void JointCalibrator::UpdatePositionOffsets(ConstRefVectorXd position_offsets)
{
    position_offsets_ = position_offsets;
}

const VectorXd& JointCalibrator::GetPositionOffsets()
{
    return position_offsets_;
}

const double& JointCalibrator::dt()
{
    return dt_;
}

/**
 * @brief Runs the calibration procedure. Returns true if the calibration is
 * done.
 */
bool JointCalibrator::Run()
{
    return RunAndGoTo(zero_vector_);
}

/**
 * @brief Runs the calibration procedure. Returns true if the calibration is
 * done. Legs are placed at the target position at the end. Calibration happens in
 * three steps: looking for motor indexes, waiting to be sure index compensation has
 * been taken into account and finally going to the desired target positions at the
 * end of the calibration.
 *
 * @param target_positions target positions for the legs at the end of the calibration
 */
bool JointCalibrator::RunAndGoTo(VectorXd const& target_positions)
{
    if (t_ == 0.)
    {
        joints_->SetZeroGains();
        joints_->SetPositionOffsets(position_offsets_);
        initial_positions_ = joints_->GetPositions();

        // If all the indices are already detected, then we
        // do not need to search them.
        if (joints_->SawAllIndices())
        {
            SwitchToWaitingTime();
        }

        joints_->DisableJointLimitCheck();
    }

    auto has_index_been_detected = joints_->HasIndexBeenDetected();
    auto positions = joints_->GetPositions();
    auto velocities = joints_->GetVelocities();
    double des_pos = 0.;
    double des_vel = 0.;
    bool finished_indexes_search = true;
    bool finished_goto_initial = false;

    // If all indexes have not been found yet.
    if (!all_indexes_detected_)
    {
        for (int i = 0; i < n_; i++)
        {
            // As long as the index was not found, search for it.
            if (!found_index_[i])
            {
                if (has_index_been_detected[i])
                {
                    found_index_[i] = true;
                    initial_positions_[i] = positions[i];
                }
                else
                {
                    if (search_methods_[i] == ALTERNATIVE)
                    {
                        if (t_ < T_ / 2.)
                        {
                            des_pos = 1.2 * M_PI * 0.5 * (1. - cos(2. * M_PI * (1. / T_) * t_));
                            des_vel = 1.2 * M_PI * 0.5 * 2. * M_PI * (1. / T_) * sin(2. * M_PI * (1. / T_) * t_);
                        }
                        else
                        {
                            des_pos = 1.2 * M_PI * cos(2. * M_PI * (0.5 / T_) * (t_ - T_ / 2.0));
                            des_vel = 1.2 * M_PI * -2. * M_PI * (0.5 / T_) * sin(2. * M_PI * (0.5 / T_) * (t_ - T_ / 2.0));
                        }
                    }
                    else if (search_methods_[i] == POSITIVE)
                    {
                        des_pos = 2.2 * M_PI * (1. - cos(2. * M_PI * (0.5 / T_) * t_));
                        des_vel = 2.2 * M_PI * 2. * M_PI * (0.5 / T_) * sin(2. * M_PI * (0.5 / T_) * t_);
                    }
                    else
                    {
                        des_pos = -2.2 * M_PI * (1. - cos(2. * M_PI * (0.5 / T_) * t_));
                        des_vel = -2.2 * M_PI * 2. * M_PI * (0.5 / T_) * sin(2. * M_PI * (0.5 / T_) * t_);
                    }
                    pos_command_[i] = des_pos / gear_ratios_[i] + initial_positions_[i];
                    vel_command_[i] = des_vel / gear_ratios_[i];
                }
                finished_indexes_search = false;
            }
            else  // After the index was found, stay at current position.
            {
                pos_command_[i] = initial_positions_[i];
                vel_command_[i] = 0.0;
            }
        }
        if (finished_indexes_search)  // If all indexes have been found we start the waiting time.
        {
            SwitchToWaitingTime();
        }
    }
    else if (t_ - t_all_indexes_detected_ > T_wait_)  // Go to target positions after waiting time.
    {        
        if (!waiting_time_flag_)  // If the waiting time has just finished.
        {
            std::cout << std::endl;
            joints_->EnableJointLimitCheck();

            // Refresh initial position of the movement after the waiting time.
            for (int i = 0; i < n_; i++)
            {
                initial_positions_[i] = positions[i];
                pos_command_[i] = initial_positions_[i];
                vel_command_[i] = (target_positions[i] - initial_positions_[i]) / T_;
                kp_command_[i] = Kp_;
                kd_command_[i] = Kd_;
            }
            waiting_time_flag_ = true;
        }

        // Interpolation between initial positions and target positions.
        double alpha = (t_ - t_all_indexes_detected_ - T_wait_) / T_;
        if (alpha <= 1.0) 
        {
            for (int i = 0; i < n_; i++)
            {
                pos_command_[i] = initial_positions_[i] * (1.0 - alpha) + target_positions[i] * alpha;
            }
        }
        else  // We have reached the target positions (at least for the command).
        {
            finished_goto_initial = true;
        }
    }
    // else : Waiting time to be sure index compensation taken into account by all motors.

    // Set all command quantities.
    joints_->SetTorques(zero_vector_);
    joints_->SetDesiredPositions(pos_command_);
    joints_->SetDesiredVelocities(vel_command_);
    joints_->SetPositionGains(kp_command_);
    joints_->SetVelocityGains(kd_command_);

    t_ += dt_;

    if (finished_goto_initial)  // Set all command quantities to 0 when the calibration finishes.
    {
        joints_->SetZeroCommands();
    }
    return finished_goto_initial;
}

/**
 * @brief Start the calibration waiting time by setting
 * gains to zero and enable index compensation.
 */
void JointCalibrator::SwitchToWaitingTime()
{
    all_indexes_detected_ = true;
    t_all_indexes_detected_ = t_;
    kp_command_.fill(0.0);  // Zero gains during waiting time.
    kd_command_.fill(0.0);
    joints_->EnableIndexOffsetCompensation();  // Enable index compensation.
}

}  // namespace odri_control_interface
