/**
 * @file joint_modules.hpp
 * @author Julian Viereck (jviereck@tuebingen.mpg.de)
 * license License BSD-3-Clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft.
 * @date 2020-12-04
 *
 * @brief Python bindings to the library.
 */


#include "my_bindings_headr.h"
#include <eigenpy/eigenpy.hpp>

using namespace boost::python;
using namespace odri_control_interface;

typedef Eigen::Matrix<long, Eigen::Dynamic, 1> VectorXl;
typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;

BOOST_PYTHON_MODULE(libodri_control_interface_pywrap)
{
    eigenpy::enableEigenPy();
    eigenpy::enableEigenPySpecific<VectorXl>();
    eigenpy::enableEigenPySpecific<VectorXb >();

    class_<JointModules>("JointModules", init<
            MasterBoardInterface*,
            VectorXl, double, double, double, VectorXl,
            Eigen::VectorXd, Eigen::VectorXd,
            double, double>())

        .def("enable", &JointModules::Enable)

        .def("set_torques", &JointModules::SetTorques)
        .def("set_desired_positions", &JointModules::SetDesiredPositions)
        .def("set_desired_velocities", &JointModules::SetDesiredVelocities)
        .def("set_position_gains", &JointModules::SetPositionGains)
        .def("set_velocity_gains", &JointModules::SetVelocityGains)

        .def("set_zero_gains", &JointModules::SetZeroGains)
        .def("set_zero_commands", &JointModules::SetZeroCommands)

        .def("set_position_offsets", &JointModules::SetPositionOffsets)
        .def("enable_index_offset_compensation", &JointModules::EnableIndexOffsetCompensation)
        .def("set_maximum_current", &JointModules::SetMaximumCurrents)

        .def("disable_joint_limit_check", &JointModules::DisableJointLimitCheck)
        .def("enable_joint_limit_check", &JointModules::EnableJointLimitCheck)

        .add_property("ready", &JointModules::GetReady)
        .add_property("enabled", &JointModules::GetEnabled)

        .add_property("saw_all_indices", &JointModules::SawAllIndices)
        .add_property("is_ready", &JointModules::IsReady)
        .add_property("has_error", &JointModules::HasError)

        .add_property("get_positions", &JointModules::GetPositions)
        .add_property("get_velocities", &JointModules::GetVelocities)
        .add_property("get_sent_torques", &JointModules::GetSentTorques)
        .add_property("get_measured_torques", &JointModules::GetMeasuredTorques)

        .add_property("gear_ratios", &JointModules::GetGearRatios);
}
