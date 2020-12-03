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

    std::ostream& msg_out_ = std::cout;

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
        SetMaximumCurrents(max_currents);
    };

    std::array<double, COUNT> GetGearRatios()
    {
        return gear_ratios_;
    }

    void Enable()
    {
        std::array<double, COUNT> zeros;
        for (int i = 0; i < COUNT; i++) {
            zeros[i] = 0.;
        }

        SetZeroCommand();

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

    void SetTorques(const std::array<double, COUNT>& desired_torques)
    {
        for (int i = 0; i < COUNT; i++)
        {
            motors_[i]->SetCurrentReference(
                polarities_[i] * desired_torques[i] /
                    (gear_ratios_[i] * motor_constants_[i]));
        }
    };

    void SetDesiredPositions(const std::array<double, COUNT>& desired_positions)
    {
        for (int i = 0; i < COUNT; i++)
        {
            motors_[i]->SetPositionReference(
                polarities_[i] * desired_positions[i] * gear_ratios_[i]);
        }
    };

    void SetDesiredVelocities(const std::array<double, COUNT>& desired_velocities)
    {
        for (int i = 0; i < COUNT; i++)
        {
            motors_[i]->SetVelocityReference(
                polarities_[i] * desired_velocities[i] * gear_ratios_[i]);
        }
    };

    void SetPositionGains(const std::array<double, COUNT>& desired_gains)
    {
       for (int i = 0; i < COUNT; i++)
        {
            motors_[i]->set_kp(
                desired_gains[i] / (gear_ratios_[i] * gear_ratios_[i] * motor_constants_[i]));
        }
    };

    void SetVelocityGains(const std::array<double, COUNT>& desired_gains)
    {
        for (int i = 0; i < COUNT; i++)
        {
            motors_[i]->set_kd(
                desired_gains[i] / (gear_ratios_[i] * gear_ratios_[i] * motor_constants_[i]));
        }
    };

   void SetMaximumCurrents(double max_currents)
    {
        for (int i = 0; i < COUNT; i++)
        {
            motors_[i]->set_current_sat(max_currents);
        }
    };

    /**
     * @brief Disables the position and velocity gains by setting them to zero.
     */
    void SetZeroGains()
    {
        std::array<double, COUNT> zeros;
        for (int i = 0; i < COUNT; i++) {
            zeros[i] = 0.;
        }
        SetPositionGains(zeros);
        SetVelocityGains(zeros);
    };

    void SetZeroCommand()
    {
        std::array<double, COUNT> zeros;
        for (int i = 0; i < COUNT; i++) {
            zeros[i] = 0.;
        }
        SetTorques(zeros);
        SetDesiredPositions(zeros);
        SetDesiredVelocities(zeros);
        SetZeroGains();
    }

    /**
     * @brief Overwrites the control commands for a default safety controller.
     * The safety controller applies a D control to all the joints based
     * on the provided `safety_damping`.
     */
    virtual void RunSafetyController()
    {
        SetZeroCommand();

        std::array<double, COUNT> kd_safety;
        for (int i = 0; i < COUNT; i++) {
            kd_safety[i] = safety_damping_;
        }
        SetVelocityGains(kd_safety);
    };

    // Used for calibration.
    void SetPositionOffsets(const std::array<double, COUNT>& position_offsets)
    {
        for (int i = 0; i < COUNT; i++)
        {
            // REVIEW: Is the following correct?
            motors_[i]->SetPositionOffset(position_offsets[i] *
                polarities_[i] * gear_ratios_[i]);
        }
        // Need to trigger a sensor parsing to update the joint positions.
        robot_if_->ParseSensorData();
    };

    void EnableIndexOffsetCompensation()
    {
        for (int i = 0; i < COUNT; i++)
        {
            motors_[i]->set_enable_index_offset_compensation(true);
        }
    };

    std::array<bool, COUNT> HasIndexBeenDetected()
    {
        std::array<bool, COUNT> index_detected;
        for (int i = 0; i < COUNT; i++)
        {
            index_detected[i] = motors_[i]->HasIndexBeenDetected();
        }
        return index_detected;
    }

    std::array<bool, COUNT> GetReady()
    {
        std::array<bool, COUNT> is_ready;
        for (int i = 0; i < COUNT; i++) {
            is_ready[i] = motors_[i]->get_is_ready();
        }
        return is_ready;
    }

    std::array<bool, COUNT> GetEnabled()
    {
        std::array<bool, COUNT> is_enabled;
        for (int i = 0; i < COUNT; i++) {
            is_enabled[i] = motors_[i]->get_is_enabled();
        }
        return is_enabled;
    }

    std::array<bool, (COUNT + 1)/2> GetMotorDriverEnabled()
    {
        std::array<bool, (COUNT + 1)/2> is_enabled;
        for (int i = 0; i < (COUNT + 1)/2; i++)
        {
            is_enabled[i] = robot_if_->motor_drivers[i].IsEnabled();
        }
        return is_enabled;
    }

    std::array<int, (COUNT + 1)/2> GetMotorDriverErrors()
    {
        std::array<int, (COUNT + 1)/2> errors;
        for (int i = 0; i < (COUNT + 1)/2; i++)
        {
            errors[i] = robot_if_->motor_drivers[i].GetErrorCode();
        }
        return errors;
    }

    bool SawAllIndices()
    {
        for (int i = 0; i < COUNT; i++)
        {
            if (!motors_[i]->get_has_index_been_detected()) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Returns true once all motors are enabled and report ready.
     */
    bool IsReady()
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

    std::array<double, COUNT> GetPositions()
    {
        std::array<double, COUNT> positions;
        for (int i = 0; i < COUNT; i++)
        {
            positions[i] = (motors_[i]->get_position() *
                polarities_[i] / gear_ratios_[i]);
        }
        return positions;
    };

    std::array<double, COUNT> GetVelocities()
    {
        std::array<double, COUNT> velocities;
        for (int i = 0; i < COUNT; i++)
        {
            velocities[i] = (motors_[i]->get_velocity() *
                polarities_[i] / gear_ratios_[i]);
        }
        return velocities;
    };

    std::array<double, COUNT> GetSentTorques()
    {
        std::array<double, COUNT> torques;
        for (int i = 0; i < COUNT; i++)
        {
            torques[i] = (motors_[i]->get_current_ref() *
                polarities_[i] * gear_ratios_[i] * motor_constants_[i]);
        }
        return torques;
    };

    std::array<double, COUNT> GetMeasuredTorques()
    {
        std::array<double, COUNT> torques;
        for (int i = 0; i < COUNT; i++)
        {
            torques[i] = (motors_[i]->get_current() *
                polarities_[i] * gear_ratios_[i] * motor_constants_[i]);
        }
        return torques;
    };

    void DisableJointLimitCheck()
    {
        check_joint_limits_ = false;
    };

    void EnableJointLimitCheck()
    {
        check_joint_limits_ = true;
    };

    /**
     * @brief Checks for errors and prints them
     */
    bool HasError()
    {
        bool has_error = false;

        if (check_joint_limits_)
        {
            auto pos = GetPositions();

            // Check for lower and upper joint limits.
            for (int i = 0; i < COUNT; i++)
            {
                if (pos[i] > upper_joint_limits_[i])
                {
                    has_error = true;
                    if (upper_joint_limits_counter_++ % 2000 == 0) {
                        msg_out_ << "ERROR: Above joint limits at joint #" << (i) << std::endl;
                        msg_out_ << "  Joints: "; PrintArray(pos); msg_out_ << std::endl;
                        msg_out_ << "  Limits: "; PrintArray(upper_joint_limits_); msg_out_ << std::endl;
                    }
                    break;
                }
            }

            for (int i = 0; i < COUNT; i++)
            {
                if (pos[i] < lower_joint_limits_[i])
                {
                    has_error = true;
                    if (lower_joint_limits_counter_++ % 2000 == 0) {
                        msg_out_ << "ERROR: Below joint limits at joint #" << (i) << std::endl;
                        msg_out_ << "  Joints: "; PrintArray(pos); msg_out_ << std::endl;
                        msg_out_ << "  Limits: "; PrintArray(lower_joint_limits_); msg_out_ << std::endl;
                    }
                    break;
                }
            }
        }

        // Check for joint velocities limtis.
        // Check the velocity only after the motors report ready to avoid
        // fast motions during the initialization phase detected as error.
        if (IsReady()) {
            auto vel = GetVelocities();
            for (int i = 0; i < COUNT; i++)
            {
                if (vel[i] > max_joint_velocities_ || vel[i] < -max_joint_velocities_)
                {
                    has_error = true;
                    if (velocity_joint_limits_counter_++ % 2000 == 0)
                    {
                        msg_out_ << "ERROR: Above joint velocity limits at joint #" << (i) << std::endl;
                        msg_out_ << "  Joints: "; PrintArray(vel); msg_out_ << std::endl;
                        msg_out_ << "  Limit: " << max_joint_velocities_ << std::endl;
                    }
                    break;
                }
            }
        }

        // Check the status of the cards.
        bool print_error = false;
        for (int i = 0; i < (COUNT + 1) / 2; i++)
        {
            if (robot_if_->motor_drivers[i].error_code != 0)
            {
                if (print_error || motor_drivers_error_counter++ % 2000 == 0)
                {
                    print_error = true;
                    msg_out_ << "ERROR at motor drivers #" << (i) << ": ";
                    switch(robot_if_->motor_drivers[i].error_code)
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
                            msg_out_ << "Other error";
                            break;
                    }
                    msg_out_ << std::endl;
                }
                has_error = true;
            }
        }
        return has_error;
    }

    void PrintArray(std::array<double, COUNT> arr)
    {
        for (int i = 0; i < COUNT; i++)
        {
            char buf[1024]; sprintf(buf, "%0.3f ", arr[i]);
            msg_out_ << std::string(buf);
        }
    }

protected:
    int upper_joint_limits_counter_;
    int lower_joint_limits_counter_;
    int velocity_joint_limits_counter_;
    int motor_drivers_error_counter;
};

}  // namespace odri_control_interface
