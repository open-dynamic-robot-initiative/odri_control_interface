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

#include <math.h>
#include <unistd.h>
#include <algorithm>

#include "master_board_sdk/defines.h"
#include "master_board_sdk/master_board_interface.h"

#include <odri_control_interface/common.hpp>
#include <odri_control_interface/joint_modules.hpp>

namespace odri_control_interface
{
const int SEARCHING = 0;
const int WAITING = 1;
const int GOTO = 2;
const double AMPLITUDE = 1.5;

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
    VectorXi calib_order_;
    VectorXd calib_pos_;
    VectorXd initial_positions_;
    VectorXd target_positions_;
    VectorXb found_index_;
    VectorXd gear_ratios_;
    VectorXd pos_command_;
    VectorXd vel_command_;
    VectorXd kp_command_;
    VectorXd kd_command_;
    VectorXd zero_vector_;
    double Kp_;
    double Kd_;
    double T_;
    double T_wait_;
    double dt_;
    double t_;
    int n_;
    bool already_calibrated_;
    int calib_state_;
    int step_number_;
    int step_number_max_;
    bool step_indexes_detected_;
    double t_step_indexes_detected_;
    double t_step_end_;

public:

    JointCalibrator(const std::shared_ptr<JointModules>& joints,
                    const std::vector<CalibrationMethod>& search_methods,
                    RefVectorXd position_offsets,
                    RefVectorXi calib_order,
                    RefVectorXd calib_pos,
                    double Kp,
                    double Kd,
                    double T,
                    double dt);

    void UpdatePositionOffsets(ConstRefVectorXd position_offsets);

    /**
     * Return the dt used by the joint calibrator.
     */
    const double& dt();

    /**
     * @brief Runs the calibration procedure. Returns true if the calibration is
     * done. Legs are placed in zero position at the end.
     */
    bool Run();

    /**
     * @brief Runs the calibration procedure. Returns true if the calibration is
     * done. Legs are placed at the target position at the end.
     *
     * @param target_positions target positions for the legs at the end of the calibration
     */
    bool RunAndGoTo(VectorXd const& target_positions);

    /**
     * @brief Search the index using the desired
     * search method
     *
     * @param i the searching motor number
     */
    void SearchIndex(int i);

    /**
     * @brief Start the calibration waiting time by setting
     * gains to zero and enable index compensation.
     */
    void SwitchToWaiting();
};

}  // namespace odri_control_interface
