/**
 * @file
 * @author Felix Widmaier <felix.widmaier@tue.mpg.de>
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 *            Gesellschaft.
 *
 * @brief Power board abstraction.
 */

#pragma once
#include <iostream>
#include <memory>

#include <master_board_sdk/master_board_interface.h>

namespace odri_control_interface
{
/**
 * @brief Class for dealing with the power board.
 */
class PowerBoard
{
protected:
    std::ostream& msg_out_ = std::cout;

    std::shared_ptr<MasterBoardInterface> robot_if_;

    // Cache for the results.
    double voltage_ = 0.0;
    double current_ = 0.0;
    double energy_ = 0.0;

    double previous_voltage_ = 0.0;
    double previous_current_ = 0.0;
    double previous_energy_ = 0.0;

    int no_update_counter_ = 0;

public:
    /**
     * Lower bound for voltage.  If the measured voltage drops below the limit,
     * an error is triggered.
     */
    double limit_min_voltage_V = 22.0;
    /**
     * Upper bound for current.  If the measured current exceeds the limit, an
     * error is triggered.
     */
    double limit_max_current_A = 10.0;
    /**
     * Maximum number of iterations without new data from the power board.  If
     * this number is exceeded, an error is triggered.
     */
    // FIXME: what would be good value here?
    // How often is this method actually called?
    double limit_max_no_update_counter = 100;

    PowerBoard(const std::shared_ptr<MasterBoardInterface>& robot_if);

    bool HasError() const;

    void ParseSensorData();

    const std::shared_ptr<MasterBoardInterface>& GetMasterBoardInterface()
        const;

    //! Get latest voltage measurement (in V).
    double GetVoltage() const;
    //! Get latest current measurement (in A).
    double GetCurrent() const;
    //! Get latest energy measurement (in J since start of the system)
    double GetEnergy() const;
};

}  // namespace odri_control_interface
