/**
 * @file
 * @author Felix Widmaier <felix.widmaier@tue.mpg.de>
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 *            Gesellschaft.
 */
#include <odri_control_interface/powerboard.hpp>

namespace odri_control_interface
{
PowerBoard::PowerBoard(const std::shared_ptr<MasterBoardInterface>& robot_if)
    : robot_if_(robot_if)
{
}

bool PowerBoard::HasError() const
{
    // FIXME: set proper values
    // TODO: Make configurable
    constexpr double MIN_VOLTAGE_V = 20.0, MAX_VOLTAGE_V = 25.0,
                     MAX_CURRENT_A = 15.0;
    constexpr int MAX_NUM_OF_NO_UPDATES = 5;

    bool has_error = false;

    if (voltage_ < MIN_VOLTAGE_V)
    {
        has_error = true;
        msg_out_ << "ERROR: Power board voltage too low: " << voltage_
                 << " V (limit: " << MIN_VOLTAGE_V << " V)" << std::endl;
    }
    else if (voltage_ > MAX_VOLTAGE_V)
    {
        has_error = true;
        msg_out_ << "ERROR: Power board voltage too high: " << voltage_
                 << " V (limit: " << MAX_VOLTAGE_V << " V)" << std::endl;
    }

    if (current_ > MAX_CURRENT_A)
    {
        has_error = true;
        msg_out_ << "ERROR: Power board current too high: " << current_
                 << " A (limit: " << MAX_CURRENT_A << " A)" << std::endl;
    }

    if (no_update_counter_ > MAX_NUM_OF_NO_UPDATES)
    {
        has_error = true;
        msg_out_ << "ERROR: No updates from power board." << std::endl;
    }

    return has_error;
}

void PowerBoard::ParseSensorData()
{
    voltage_ = robot_if_->powerboard_voltage();
    current_ = robot_if_->powerboard_current();
    energy_ = robot_if_->powerboard_energy();

    if (voltage_ == previous_voltage_ && current_ == previous_current_ &&
        energy_ == previous_energy_)
    {
        no_update_counter_++;
    }
    previous_voltage_ = voltage_;
    previous_current_ = current_;
    previous_energy_ = energy_;
}

const std::shared_ptr<MasterBoardInterface>&
PowerBoard::GetMasterBoardInterface() const
{
    return robot_if_;
}
double PowerBoard::GetVoltage() const
{
    return voltage_;
}
double PowerBoard::GetCurrent() const
{
    return current_;
}
double PowerBoard::GetEnergy() const
{
    return energy_;
}

}  // namespace odri_control_interface
