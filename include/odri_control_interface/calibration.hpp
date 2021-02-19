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

#include <algorithm>
#include <math.h>
#include <unistd.h>

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

#include <odri_control_interface/common.hpp>
#include <odri_control_interface/joint_modules.hpp>

namespace odri_control_interface
{
enum CalibrationMethod
{
    AUTO,
    POSITIVE,
    NEGATIVE,
    ALTERNATIVE
};

/**
 * @brief
 */
class JointCalibrator
{
protected:
    std::shared_ptr<JointModules> joints_;
    std::vector<CalibrationMethod> search_methods_;
    VectorXd position_offsets_;
    VectorXd initial_positions_;
    VectorXb found_index_;
    VectorXd t_end_;
    VectorXd gear_ratios_;
    VectorXd command_;
    double Kp_;
    double Kd_;    
    double T_;
    double dt_;
    double t_;
    bool go_to_zero_position_;
    int n_;

public:
    JointCalibrator(
        const std::shared_ptr<JointModules>& joints,
        const std::vector<CalibrationMethod>& search_methods,
        RefVectorXd position_offsets,
        double Kp, double Kd, double T, double dt
    );

    void UpdatePositionOffsets(RefVectorXd position_offsets);

    /**
     * @brief Runs the calibration procedure. Returns true if the calibration is done.
     */
    bool Run();
};


} // namespace odri_control_interface
