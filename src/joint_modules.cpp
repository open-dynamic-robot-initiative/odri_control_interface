/**
 * @file joint_modules.cpp
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-11-27
 *
 * @brief Joint module abstraction for
 */

#include "odri_control_interface/joint_modules.hpp"

namespace odri_control_interface
{
JointModules::JointModules(
    const std::shared_ptr<MasterBoardInterface>& robot_if,
    ConstRefVectorXi motor_numbers,
    double motor_constants,
    double gear_ratios,
    double max_currents,
    ConstRefVectorXb reverse_polarities,
    ConstRefVectorXd lower_joint_limits,
    ConstRefVectorXd upper_joint_limits,
    double max_joint_velocities,
    double safety_damping)
    : robot_if_(robot_if),
      lower_joint_limits_(lower_joint_limits),
      upper_joint_limits_(upper_joint_limits),
      max_joint_velocities_(max_joint_velocities),
      check_joint_limits_(true)
{
    n_ = static_cast<int>(motor_numbers.size());
    nd_ = (n_ + 1) / 2;

    // Check input arrays for correct sizes.
    if (reverse_polarities.size() != n_)
    {
        throw std::runtime_error(
            "Motor polarities has different size than motor numbers");
    }

    if (lower_joint_limits.size() != n_)
    {
        throw std::runtime_error(
            "Lower joint limits has different size than motor numbers");
    }

    if (upper_joint_limits.size() != n_)
    {
        throw std::runtime_error(
            "Upper joint limits has different size than motor numbers");
    }

    // Resize and fill the vectors.
    gear_ratios_.resize(n_);
    motor_constants_.resize(n_);
    positions_.resize(n_);
    velocities_.resize(n_);
    sent_torques_.resize(n_);
    measured_torques_.resize(n_);
    index_been_detected_.resize(n_);
    index_been_detected_.fill(false);
    polarities_.resize(n_);
    ready_.resize(n_);
    ready_.fill(false);
    enabled_.resize(n_);
    enabled_.fill(false);
    zero_vector_.resize(n_);
    zero_vector_.fill(0.);
    safety_damping_.resize(n_);
    safety_damping_.fill(safety_damping);
    motor_driver_enabled_.resize(nd_);
    motor_driver_enabled_.fill(false);
    motor_driver_errors_.resize(nd_);
    motor_driver_errors_.fill(0);

    gear_ratios_.fill(gear_ratios);
    motor_constants_.fill(motor_constants);

    for (int i = 0; i < n_; i++)
    {
        motors_.push_back(robot_if_->GetMotor(motor_numbers[i]));
        polarities_(i) = reverse_polarities(i) ? -1 : 1;
    }

    SetMaximumCurrents(max_currents);
}

const VectorXd& JointModules::GetGearRatios()
{
    return gear_ratios_;
}

void JointModules::ParseSensorData()
{
    for (int i = 0; i < n_; i++)
    {
        positions_(i) =
            motors_[i]->get_position() * polarities_(i) / gear_ratios_(i);
        velocities_(i) =
            motors_[i]->get_velocity() * polarities_(i) / gear_ratios_(i);
        sent_torques_(i) = (motors_[i]->get_current_ref() * polarities_(i) *
                            gear_ratios_(i) * motor_constants_(i));
        measured_torques_(i) = (motors_[i]->get_current() * polarities_[i] *
                                gear_ratios_[i] * motor_constants_[i]);

        index_been_detected_(i) = motors_[i]->HasIndexBeenDetected();
        ready_(i) = motors_[i]->get_is_ready();
        enabled_(i) = motors_[i]->get_is_enabled();
    }

    for (int i = 0; i < nd_; i++)
    {
        motor_driver_enabled_(i) = robot_if_->motor_drivers[i].IsEnabled();
        motor_driver_errors_(i) = robot_if_->motor_drivers[i].GetErrorCode();
    }
}

void JointModules::Enable()
{
    // TODO: Enable this again.
    SetZeroCommands();

    // Enable all motors and cards.
    for (int i = 0; i < (n_ + 1) / 2; i++)
    {
        robot_if_->motor_drivers[i].motor1->Enable();
        robot_if_->motor_drivers[i].motor2->Enable();
        robot_if_->motor_drivers[i].EnablePositionRolloverError();
        robot_if_->motor_drivers[i].SetTimeout(5);
        robot_if_->motor_drivers[i].Enable();
    }
}

void JointModules::SetTorques(ConstRefVectorXd desired_torques)
{
    for (int i = 0; i < n_; i++)
    {
        motors_[i]->SetCurrentReference(
            polarities_(i) * desired_torques(i) /
            (gear_ratios_(i) * motor_constants_(i)));
    }
}

void JointModules::SetDesiredPositions(ConstRefVectorXd desired_positions)
{
    for (int i = 0; i < n_; i++)
    {
        motors_[i]->SetPositionReference(polarities_[i] * desired_positions[i] *
                                         gear_ratios_[i]);
    }
}

void JointModules::SetDesiredVelocities(ConstRefVectorXd desired_velocities)
{
    for (int i = 0; i < n_; i++)
    {
        motors_[i]->SetVelocityReference(
            polarities_[i] * desired_velocities[i] * gear_ratios_[i]);
    }
}

void JointModules::SetPositionGains(ConstRefVectorXd desired_gains)
{
    for (int i = 0; i < n_; i++)
    {
        motors_[i]->set_kp(
            desired_gains[i] /
            (gear_ratios_[i] * gear_ratios_[i] * motor_constants_[i]));
    }
}

void JointModules::SetVelocityGains(ConstRefVectorXd desired_gains)
{
    for (int i = 0; i < n_; i++)
    {
        motors_[i]->set_kd(
            desired_gains[i] /
            (gear_ratios_[i] * gear_ratios_[i] * motor_constants_[i]));
    }
}

void JointModules::SetZeroGains()
{
    SetPositionGains(zero_vector_);
    SetVelocityGains(zero_vector_);
}

void JointModules::SetZeroCommands()
{
    SetTorques(zero_vector_);
    SetDesiredPositions(zero_vector_);
    SetDesiredVelocities(zero_vector_);
    SetZeroGains();
}

void JointModules::RunSafetyController()
{
    SetZeroCommands();
    SetVelocityGains(safety_damping_);
}

void JointModules::SetPositionOffsets(ConstRefVectorXd position_offsets)
{
    for (int i = 0; i < n_; i++)
    {
        // REVIEW: Is the following correct?
        motors_[i]->SetPositionOffset(position_offsets(i) * polarities_(i) *
                                      gear_ratios_(i));
    }
    // Need to trigger a sensor parsing to update the joint positions.
    robot_if_->ParseSensorData();
    ParseSensorData();
}

void JointModules::EnableIndexOffsetCompensation()
{
    for (int i = 0; i < n_; i++)
    {
        motors_[i]->set_enable_index_offset_compensation(true);
    }
}

const VectorXb& JointModules::HasIndexBeenDetected()
{
    return index_been_detected_;
}

const VectorXb& JointModules::GetReady()
{
    return ready_;
}

const VectorXb& JointModules::GetEnabled()
{
    return enabled_;
}

const VectorXb& JointModules::GetMotorDriverEnabled()
{
    return motor_driver_enabled_;
}

const VectorXi& JointModules::GetMotorDriverErrors()
{
    return motor_driver_errors_;
}

bool JointModules::SawAllIndices()
{
    for (int i = 0; i < n_; i++)
    {
        if (!motors_[i]->get_has_index_been_detected())
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief Returns true once all motors are enabled and report ready.
 */
bool JointModules::IsReady()
{
    bool is_ready_ = true;

    for (int i = 0; i < n_; i++)
    {
        if (!motors_[i]->get_is_enabled() || !motors_[i]->get_is_ready())
        {
            is_ready_ = false;
            break;
        }
    }
    return is_ready_;
}

const VectorXd& JointModules::GetPositions()
{
    return positions_;
}

const VectorXd& JointModules::GetVelocities()
{
    return velocities_;
}

const VectorXd& JointModules::GetSentTorques()
{
    return sent_torques_;
}

const VectorXd& JointModules::GetMeasuredTorques()
{
    return measured_torques_;
}

void JointModules::DisableJointLimitCheck()
{
    check_joint_limits_ = false;
}

void JointModules::EnableJointLimitCheck()
{
    check_joint_limits_ = true;
}

/**
 * @brief Checks for errors and prints them
 */
bool JointModules::HasError()
{
    bool has_error = false;

    if (check_joint_limits_)
    {
        // Check for lower and upper joint limits.
        for (int i = 0; i < n_; i++)
        {
            if (positions_(i) > upper_joint_limits_(i))
            {
                has_error = true;
                if (upper_joint_limits_counter_++ % 2000 == 0)
                {
                    msg_out_ << "ERROR: Above joint limits at joint #" << (i)
                             << std::endl;
                    msg_out_ << "  Joints: ";
                    PrintVector(positions_);
                    msg_out_ << std::endl;
                    msg_out_ << "  Limits: ";
                    PrintVector(upper_joint_limits_);
                    msg_out_ << std::endl;
                }
                break;
            }
        }

        for (int i = 0; i < n_; i++)
        {
            if (positions_(i) < lower_joint_limits_(i))
            {
                has_error = true;
                if (lower_joint_limits_counter_++ % 2000 == 0)
                {
                    msg_out_ << "ERROR: Below joint limits at joint #" << (i)
                             << std::endl;
                    msg_out_ << "  Joints: ";
                    PrintVector(positions_);
                    msg_out_ << std::endl;
                    msg_out_ << "  Limits: ";
                    PrintVector(lower_joint_limits_);
                    msg_out_ << std::endl;
                }
                break;
            }
        }
    }

    // Check for joint velocities limtis.
    // Check the velocity only after the motors report ready to avoid
    // fast motions during the initialization phase detected as error.
    if (IsReady())
    {
        for (int i = 0; i < n_; i++)
        {
            if (std::abs(velocities_[i]) > max_joint_velocities_)
            {
                has_error = true;
                if (velocity_joint_limits_counter_++ % 2000 == 0)
                {
                    msg_out_ << "ERROR: Above joint velocity limits at joint #"
                             << (i) << std::endl;
                    msg_out_ << "  Joints: ";
                    PrintVector(velocities_);
                    msg_out_ << std::endl;
                    msg_out_ << "  Limit: " << max_joint_velocities_
                             << std::endl;
                }
                break;
            }
        }
    }

    // Check the status of the cards.
    bool print_error = false;
    for (int i = 0; i < nd_; i++)
    {
        if (robot_if_->motor_drivers[i].error_code != 0)
        {
            if (print_error || motor_drivers_error_counter++ % 2000 == 0)
            {
                print_error = true;
                msg_out_ << "ERROR at motor drivers #" << (i) << ": ";
                switch (robot_if_->motor_drivers[i].error_code)
                {
                    case UD_SENSOR_STATUS_ERROR_ENCODER1:
                        msg_out_ << "Encoder1 error";
                        break;
                    case UD_SENSOR_STATUS_ERROR_SPI_RECV_TIMEOUT:
                        msg_out_ << "SPI Receiver timeout";
                        break;
                    case UD_SENSOR_STATUS_ERROR_CRIT_TEMP:
                        msg_out_ << "Critical temperature";
                        break;
                    case UD_SENSOR_STATUS_ERROR_POSCONV:
                        msg_out_ << "SpinTAC Positon module";
                        break;
                    case UD_SENSOR_STATUS_ERROR_POS_ROLLOVER:
                        msg_out_ << "Position rollover occured";
                        break;
                    case UD_SENSOR_STATUS_ERROR_ENCODER2:
                        msg_out_ << "Encoder2 error";
                        break;
                    default:
                        msg_out_ << "Other error (" << robot_if_->motor_drivers[i].error_code << ")";
                        break;
                }
                msg_out_ << std::endl;
            }
            has_error = true;
        }
    }
    return has_error;
}

void JointModules::PrintVector(ConstRefVectorXd vector)
{
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    msg_out_ << vector.transpose().format(CleanFmt);
}

void JointModules::SetMaximumCurrents(double max_currents)
{
    for (int i = 0; i < n_; i++)
    {
        motors_[i]->set_current_sat(max_currents);
    }
}

}  // namespace odri_control_interface
