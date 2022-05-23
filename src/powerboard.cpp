/**
 * @file
 * @author Felix Widmaier <felix.widmaier@tue.mpg.de>
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 *            Gesellschaft.
 */
#include <odri_control_interface/powerboard.hpp>

#include <boost/iostreams/stream.hpp>

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
                     MAX_CURRENT_A = 10.0;
    // FIXME: what would be good value here?
    // How often is this method actually called?
    constexpr int MAX_NUM_OF_NO_UPDATES = 100;

    static int print_counter = 0;

    bool has_error = false;

    // null_sink will simply discard the output (similar to /dev/null).  Only
    // use the actual msg_out_ every few calls to reduce the output rate.
    boost::iostreams::stream<boost::iostreams::null_sink> null_sink{
        boost::iostreams::null_sink()};
    std::ostream& out = print_counter % 5000 == 0 ? msg_out_ : null_sink;

    if (no_update_counter_ > MAX_NUM_OF_NO_UPDATES)
    {
        has_error = true;
        out << "ERROR: No updates from power board since " << no_update_counter_
            << " iterations." << std::endl;
    }

    // only check value limits if actual values have been reported
    if (voltage_ != 0 || current_ != 0 || energy_ != 0)
    {
        if (voltage_ < MIN_VOLTAGE_V)
        {
            has_error = true;
            out << "ERROR: Power board voltage too low: " << voltage_
                << " V (limit: " << MIN_VOLTAGE_V << " V)" << std::endl;
        }
        else if (voltage_ > MAX_VOLTAGE_V)
        {
            has_error = true;
            out << "ERROR: Power board voltage too high: " << voltage_
                << " V (limit: " << MAX_VOLTAGE_V << " V)" << std::endl;
        }

        if (current_ > MAX_CURRENT_A)
        {
            has_error = true;
            out << "ERROR: Power board current too high: " << current_
                << " A (limit: " << MAX_CURRENT_A << " A)" << std::endl;
        }
    }

    if (has_error)
    {
        // TODO: There is a problem with this implementation: If multiple errors
        // occur for short times, not all of them may be printed.  Ideally,
        // there should be a separate counter for each error, but this would
        // bloat the code a bit.
        print_counter++;
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
    else
    {
        no_update_counter_ = 0;
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
