/**
 * @file utils.cpp
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-12-08
 *
 * @brief Different utils for using the framework.
 */

#pragma once

#include "odri_control_interface/calibration.hpp"
#include "odri_control_interface/robot.hpp"

namespace odri_control_interface
{
std::shared_ptr<Robot> RobotFromYamlFile(const std::string& if_name,
                                         const std::string& file_path);
std::shared_ptr<Robot> RobotFromYamlFile(const std::string& file_path);
std::shared_ptr<JointCalibrator> JointCalibratorFromYamlFile(
    const std::string& file_path, std::shared_ptr<JointModules> joints);
std::shared_ptr<MasterBoardInterface> CreateMasterBoardInterface(
    const std::string &if_name, bool listener_mode = false);
}  // namespace odri_control_interface
