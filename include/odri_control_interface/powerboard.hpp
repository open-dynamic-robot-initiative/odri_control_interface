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
    double voltage_;
    double current_;
    double energy_;

    double previous_voltage_;
    double previous_current_;
    double previous_energy_;

    int no_update_counter_;

public:
    PowerBoard(const std::shared_ptr<MasterBoardInterface>& robot_if);

    bool HasError() const;

    void ParseSensorData();

    const std::shared_ptr<MasterBoardInterface>& GetMasterBoardInterface()
        const;

    double GetVoltage() const;
    double GetCurrent() const;
    double GetEnergy() const;
};

}  // namespace odri_control_interface
