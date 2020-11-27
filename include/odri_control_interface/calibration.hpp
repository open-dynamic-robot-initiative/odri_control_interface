/**
 * @file joint_modules.hpp
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-11-27
 *
 * @brief Class for calibrating the joints.
 */

#pragma once

#include <unistd.h>

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

namespace odri_control_interface
{
enum CalibrationMethod { 
    POSITIVE,
    NEGATIVE,
    ALTERNATIVE
};

/**
 * @brief 
 */
template <int COUNT>
class JointCalibrator
{
public:
    JointCalibrator(
        std::array<CalibrationMethod, COUNT> search_methods,
        double Kd, double Kp, double dt
    );

    std::array<double, COUNT> run(
        const std::array<double, COUNT>& joint_positions,
        const std::array<double, COUNT>& joint_velocities,
        const std::array<bool, COUNT>& encoder_indices,
    );

    bool is_calibration_done();
};


} // namespace odri_control_interface
