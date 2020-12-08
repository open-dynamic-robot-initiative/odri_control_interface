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

#include "odri_control_interface/robot.hpp"
#include "odri_control_interface/calibration.hpp"

namespace odri_control_interface
{

Robot* RobotFromYamlFile(std::string file_path);
JointCalibrator* JointCalibratorFromYamlFile(std::string file_path, JointModules* joints);

}
