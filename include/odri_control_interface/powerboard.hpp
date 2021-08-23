/**
 * @file powerboard.hpp
 * @author Pierre-Alexandre Leziart (paleziart@laas.fr)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2021-08-23
 *
 * @brief Powerboard abstraction.
 */

#pragma once

#include <math.h>
#include <unistd.h>
#include <stdexcept>

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

#include <odri_control_interface/common.hpp>

namespace odri_control_interface
{
/**
 * @brief Class for dealing with the powerboard.
 */
class Powerboard
{
protected:
    std::shared_ptr<MasterBoardInterface> robot_if_;

    // Cache for the results.
    double current_;
    double voltage_;
    double energy_;

public:
    Powerboard(const std::shared_ptr<MasterBoardInterface>& robot_if);

    // If needed, add some error handling for the powerboard as well.
    // For instance, check for low voltage or high current
    bool HasError()
    {
        return false;
    }

    void ParseSensorData();

    const std::shared_ptr<MasterBoardInterface>& GetMasterBoardInterface();
    double GetCurrent();
    double GetVoltage();
    double GetEnergy();
};

}  // namespace odri_control_interface
