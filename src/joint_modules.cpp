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
#include <cstddef>

#include <fmt/format.h>

namespace
{
// local helper functions

/**
 * @brief Output comma-separated list of indices which elements match `val`.
 *
 * @param vec  Vector of elements.
 * @param val  Value that is searched for.
 * @param out  Output stream to which matching indices are written.
 *
 */
template <typename T>
std::stringstream write_matching_indices(std::ostream& out,
                                         const std::vector<T>& vec,
                                         T val)
{
    std::string_view sep = "";

    for (size_t i = 0; i < vec.size(); i++)
    {
        if (vec[i] == val)
        {
            out << sep << i;
            sep = ", ";
        }
    }
}

std::string_view describe_motor_driver_error(int error_code)
{
    switch (error_code)
    {
        case UD_SENSOR_STATUS_ERROR_NO_ERROR:
            return "No error";
        case UD_SENSOR_STATUS_ERROR_ENCODER1:
            return "Encoder A error";
        case UD_SENSOR_STATUS_ERROR_SPI_RECV_TIMEOUT:
            return "SPI Receiver timeout";
        case UD_SENSOR_STATUS_ERROR_CRIT_TEMP:
            return "Critical temperature";
        case UD_SENSOR_STATUS_ERROR_POSCONV:
            return "SpinTAC Position module";
        case UD_SENSOR_STATUS_ERROR_POS_ROLLOVER:
            return "Position rollover occurred";
        case UD_SENSOR_STATUS_ERROR_ENCODER2:
            return "Encoder B error";
        default:
            return "Other error";
    }
}

}  // namespace

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
      num_motors_(static_cast<int>(motor_numbers.size())),
      num_motor_drivers_((num_motors_ + 1) / 2),
      check_joint_limits_(true),
      error_data_(static_cast<size_t>(num_motors_),
                  static_cast<size_t>(num_motor_drivers_)),
      upper_joint_limits_counter_(0),
      lower_joint_limits_counter_(0),
      velocity_joint_limits_counter_(0),
      motor_drivers_error_counter(0)

{
    // Check input arrays for correct sizes.
    if (reverse_polarities.size() != num_motors_)
    {
        throw std::runtime_error(
            "Motor polarities has different size than motor numbers");
    }

    if (lower_joint_limits.size() != num_motors_)
    {
        throw std::runtime_error(
            "Lower joint limits has different size than motor numbers");
    }

    if (upper_joint_limits.size() != num_motors_)
    {
        throw std::runtime_error(
            "Upper joint limits has different size than motor numbers");
    }

    // Resize and fill the vectors.
    gear_ratios_.resize(num_motors_);
    motor_constants_.resize(num_motors_);
    positions_.resize(num_motors_);
    velocities_.resize(num_motors_);
    sent_torques_.resize(num_motors_);
    measured_torques_.resize(num_motors_);
    index_been_detected_.resize(num_motors_);
    index_been_detected_.fill(false);
    polarities_.resize(num_motors_);
    ready_.resize(num_motors_);
    ready_.fill(false);
    enabled_.resize(num_motors_);
    enabled_.fill(false);
    zero_vector_.resize(num_motors_);
    zero_vector_.fill(0.);
    safety_damping_.resize(num_motors_);
    safety_damping_.fill(safety_damping);
    motor_driver_enabled_.resize(num_motor_drivers_);
    motor_driver_enabled_.fill(false);
    motor_driver_errors_.resize(num_motor_drivers_);
    motor_driver_errors_.fill(0);

    gear_ratios_.fill(gear_ratios);
    motor_constants_.fill(motor_constants);

    for (int i = 0; i < num_motors_; i++)
    {
        motors_.push_back(robot_if_->GetMotor(motor_numbers[i]));
        polarities_(i) = reverse_polarities(i) ? -1 : 1;
    }

    SetMaximumCurrents(max_currents);
}

JointModules::~JointModules()
{
}

const VectorXd& JointModules::GetGearRatios()
{
    return gear_ratios_;
}

int JointModules::GetNumberMotors()
{
    return num_motors_;
}

void JointModules::ParseSensorData()
{
    for (int i = 0; i < num_motors_; i++)
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

    for (int i = 0; i < num_motor_drivers_; i++)
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
    for (int i = 0; i < (num_motors_ + 1) / 2; i++)
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
    for (int i = 0; i < num_motors_; i++)
    {
        motors_[i]->SetCurrentReference(
            polarities_(i) * desired_torques(i) /
            (gear_ratios_(i) * motor_constants_(i)));
    }
}

void JointModules::SetDesiredPositions(ConstRefVectorXd desired_positions)
{
    for (int i = 0; i < num_motors_; i++)
    {
        motors_[i]->SetPositionReference(polarities_[i] * desired_positions[i] *
                                         gear_ratios_[i]);
    }
}

void JointModules::SetDesiredVelocities(ConstRefVectorXd desired_velocities)
{
    for (int i = 0; i < num_motors_; i++)
    {
        motors_[i]->SetVelocityReference(
            polarities_[i] * desired_velocities[i] * gear_ratios_[i]);
    }
}

void JointModules::SetPositionGains(ConstRefVectorXd desired_gains)
{
    for (int i = 0; i < num_motors_; i++)
    {
        motors_[i]->set_kp(
            desired_gains[i] /
            (gear_ratios_[i] * gear_ratios_[i] * motor_constants_[i]));
    }
}

void JointModules::SetVelocityGains(ConstRefVectorXd desired_gains)
{
    for (int i = 0; i < num_motors_; i++)
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
    for (int i = 0; i < num_motors_; i++)
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
    for (int i = 0; i < num_motors_; i++)
    {
        motors_[i]->set_enable_index_offset_compensation(true);
    }
}

void JointModules::EnableIndexOffsetCompensation(int i)
{
    motors_[i]->set_enable_index_offset_compensation(true);
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
    for (int i = 0; i < num_motors_; i++)
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
bool JointModules::IsReady() const
{
    bool is_ready_ = true;

    for (int i = 0; i < num_motors_; i++)
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
    error_data_.reset();

    constexpr int PRINT_RATE = 2000;
    bool print_position_above_limit_error = false;
    bool print_position_below_limit_error = false;
    bool print_velocity_limit_error = false;
    bool print_motor_board_error = false;

    if (check_joint_limits_)
    {
        // Check for lower and upper joint limits.
        for (int i = 0; i < num_motors_; i++)
        {
            if (positions_(i) > upper_joint_limits_(i))
            {
                size_t ui = static_cast<size_t>(i);
                error_data_.joint_position_limit_exceeded[ui] = +1;

                has_error = true;
                print_position_above_limit_error = true;
            }
        }

        for (int i = 0; i < num_motors_; i++)
        {
            if (positions_(i) < lower_joint_limits_(i))
            {
                size_t ui = static_cast<size_t>(i);
                error_data_.joint_position_limit_exceeded[ui] = -1;

                has_error = true;
                print_position_below_limit_error = true;
            }
        }
    }

    // Check for joint velocities limits.
    // Check the velocity only after the motors report ready to avoid
    // fast motions during the initialization phase detected as error.
    if (IsReady())
    {
        for (int i = 0; i < num_motors_; i++)
        {
            if (std::abs(velocities_[i]) > max_joint_velocities_)
            {
                size_t ui = static_cast<size_t>(i);
                error_data_.joint_velocity_limit_exceeded[ui] = true;

                has_error = true;
                print_velocity_limit_error = true;
            }
        }
    }

    // Check the status of the cards.
    for (size_t i = 0; i < static_cast<size_t>(num_motor_drivers_); i++)
    {
        error_data_.motor_driver_error_codes[i] =
            robot_if_->motor_drivers[i].error_code;

        if (robot_if_->motor_drivers[i].error_code !=
            UD_SENSOR_STATUS_ERROR_NO_ERROR)
        {
            has_error = true;
            print_motor_board_error = true;
        }
    }

    // === print error messages ===
    // TODO: Do we really need the separate counters per error type?  If not, we
    // could use a single counter and simply use the output from
    // GetErrorDescription().

    if (print_position_above_limit_error &&
        upper_joint_limits_counter_++ % PRINT_RATE == 0)
    {
        msg_out_ << "ERROR: Above position limits at joint(s) ";
        write_matching_indices(
            msg_out_, error_data_.joint_position_limit_exceeded, +1);
        msg_out_ << std::endl;
        msg_out_ << "  Joints: ";
        PrintVector(positions_);
        msg_out_ << std::endl;
        msg_out_ << "  Limits: ";
        PrintVector(upper_joint_limits_);
        msg_out_ << std::endl;
    }

    if (print_position_below_limit_error &&
        lower_joint_limits_counter_++ % PRINT_RATE == 0)
    {
        msg_out_ << "ERROR: Below position limits at joint(s) ";
        write_matching_indices(
            msg_out_, error_data_.joint_position_limit_exceeded, -1);
        msg_out_ << std::endl;
        msg_out_ << "  Joints: ";
        PrintVector(positions_);
        msg_out_ << std::endl;
        msg_out_ << "  Limits: ";
        PrintVector(lower_joint_limits_);
        msg_out_ << std::endl;
    }

    if (print_velocity_limit_error &&
        velocity_joint_limits_counter_++ % PRINT_RATE == 0)
    {
        msg_out_ << "ERROR: Above joint velocity limits at joint(s) ";
        write_matching_indices(
            msg_out_, error_data_.joint_velocity_limit_exceeded, true);
        msg_out_ << std::endl;
        msg_out_ << "  Joints: ";
        PrintVector(velocities_);
        msg_out_ << std::endl;
        msg_out_ << "  Limit: " << max_joint_velocities_ << std::endl;
    }

    if (print_motor_board_error &&
        motor_drivers_error_counter++ % PRINT_RATE == 0)
    {
        for (size_t i = 0; i < error_data_.motor_driver_error_codes.size(); i++)
        {
            const int error_code = error_data_.motor_driver_error_codes[i];

            if (error_code != UD_SENSOR_STATUS_ERROR_NO_ERROR)
            {
                std::string_view error_desc =
                    describe_motor_driver_error(error_code);
                msg_out_ << fmt::format("ERROR at motor driver #{}: [{}] {}",
                                        i,
                                        error_code,
                                        error_desc);
            }
        }
    }

    return has_error;
}

std::optional<ErrorMessage> JointModules::GetError() const
{
    // Check the status of the cards.
    for (int i = 0; i < num_motor_drivers_; i++)
    {
        if (robot_if_->motor_drivers[i].error_code != 0)
        {
            std::string_view msg = describe_motor_driver_error(
                robot_if_->motor_drivers[i].error_code);

            return ErrorMessage("Motor Driver #{}: [{}] {}",
                                i,
                                robot_if_->motor_drivers[i].error_code,
                                msg);
        }
    }

    if (check_joint_limits_)
    {
        // Check for lower and upper joint limits.
        for (int i = 0; i < num_motors_; i++)
        {
            if (positions_(i) < lower_joint_limits_(i))
            {
                return ErrorMessage(
                    "Joint #{} is below position limit (actual: {:.3f}, limit: "
                    "{:.3f})",
                    i,
                    positions_(i),
                    lower_joint_limits_(i));
            }
            if (positions_(i) > upper_joint_limits_(i))
            {
                return ErrorMessage(
                    "Joint #{} is above position limit (actual: {:.3f}, limit: "
                    "{:.3f})",
                    i,
                    positions_(i),
                    upper_joint_limits_(i));
            }
        }
    }

    // Check for joint velocities limits.
    // Check the velocity only after the motors report ready to avoid
    // fast motions during the initialization phase detected as error.
    if (IsReady())
    {
        for (int i = 0; i < num_motors_; i++)
        {
            if (std::abs(velocities_[i]) > max_joint_velocities_)
            {
                return ErrorMessage(
                    "Joint {} exceeds velocity limit (actual: {:.3f}, limit: "
                    "{:.3f})",
                    i,
                    velocities_[i],
                    max_joint_velocities_);
            }
        }
    }

    return std::nullopt;
}

std::string JointModules::GetErrorDescription() const
{
    int error_count = 0;
    std::string msg = "";

    for (size_t i = 0; i < error_data_.motor_driver_error_codes.size(); i++)
    {
        const int error_code = error_data_.motor_driver_error_codes[i];

        if (error_code != UD_SENSOR_STATUS_ERROR_NO_ERROR)
        {
            std::string_view error_desc = describe_motor_driver_error(
                robot_if_->motor_drivers[i].error_code);

            error_count++;
            msg += fmt::format("Motor Driver {}: [{}] {}\n",
                               i,
                               error_data_.motor_driver_error_codes[i],
                               error_desc);
        }
    }

    for (size_t i = 0; i < error_data_.joint_position_limit_exceeded.size();
         i++)
    {
        if (error_data_.joint_position_limit_exceeded[i] > 0)
        {
            error_count++;
            msg += fmt::format(
                "Joint {} above position limit ({:.3f}, limit: {:.3f})\n",
                i,
                positions_(static_cast<int>(i)),
                upper_joint_limits_(static_cast<int>(i)));
        }
        else if (error_data_.joint_position_limit_exceeded[i] < 0)
        {
            error_count++;
            msg += fmt::format(
                "Joint {} below position limit ({:.3f}, limit: {:.3f})\n",
                i,
                positions_(static_cast<int>(i)),
                lower_joint_limits_(static_cast<int>(i)));
        }
    }

    for (size_t i = 0; i < error_data_.joint_velocity_limit_exceeded.size();
         i++)
    {
        if (error_data_.joint_velocity_limit_exceeded[i])
        {
            error_count++;
            msg += fmt::format(
                "Joint {} exceeds velocity limit ({:.3f}, limit: {:.3f})\n",
                i,
                velocities_[static_cast<int>(i)],
                max_joint_velocities_);
        }
    }

    if (error_count > 1)
    {
        msg = fmt::format("There are {} errors:\n{}", error_count, msg);
    }

    return msg;
}

void JointModules::PrintVector(ConstRefVectorXd vector)
{
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    msg_out_ << vector.transpose().format(CleanFmt);
}

void JointModules::SetMaximumCurrents(double max_currents)
{
    for (size_t i = 0; i < static_cast<size_t>(num_motors_); i++)
    {
        motors_[i]->set_current_sat(max_currents);
    }
}

}  // namespace odri_control_interface
