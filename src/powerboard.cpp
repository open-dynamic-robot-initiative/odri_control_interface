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

#include "odri_control_interface/powerboard.hpp"

#include <iostream>

namespace odri_control_interface
{
Powerboard::Powerboard(const std::shared_ptr<MasterBoardInterface>& robot_if)
    : robot_if_(robot_if)
{
}

const std::shared_ptr<MasterBoardInterface>& Powerboard::GetMasterBoardInterface()
{
    return robot_if_;
}

double Powerboard::GetCurrent()
{
    return current_;
}

double Powerboard::GetVoltage()
{
    return voltage_;
}

double Powerboard::GetEnergy()
{
    return energy_;
}

void Powerboard::ParseSensorData()
{
    current_ = robot_if_->powerboard_current();
    voltage_ = robot_if_->powerboard_voltage();
    energy_ = robot_if_->powerboard_energy();
}

}  // namespace odri_control_interface
