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

#include "bindings.h"
#include <boost/python/class.hpp>
#include <boost/python/overloads.hpp>
#include <eigenpy/eigenpy.hpp>

using namespace boost::python;
using namespace odri_control_interface;

typedef Eigen::Matrix<long, Eigen::Dynamic, 1> VectorXl;
typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;

std::shared_ptr<JointCalibrator> joint_calibrator_constructor(
    std::shared_ptr<JointModules> joints,
    boost::python::list search_methods,
    RefVectorXd position_offsets,
    RefVectorXi calib_order,
    RefVectorXd calib_pos,
    double Kp,
    double Kd,
    double T,
    double dt)
{
    boost::python::ssize_t len = boost::python::len(search_methods);
    std::vector<CalibrationMethod> search_method_vec;
    for (int i = 0; i < len; i++)
    {
        search_method_vec.push_back(
            boost::python::extract<CalibrationMethod>(search_methods[i]));
    }
    return std::make_shared<JointCalibrator>(
        joints, search_method_vec, position_offsets, calib_order,
        calib_pos, Kp, Kd, T, dt);
}

std::shared_ptr<JointModules> joint_modules_constructor(
    const std::shared_ptr<MasterBoardInterface>& robot_if,
    ConstRefVectorXl motor_numbers,
    ConstRefVectorXd motor_constants,
    ConstRefVectorXd gear_ratios,
    ConstRefVectorXd max_currents,
    ConstRefVectorXb reverse_polarities,
    ConstRefVectorXd lower_joint_limits,
    ConstRefVectorXd upper_joint_limits,
    double max_joint_velocities,
    double safety_damping)
{
    VectorXi motor_numbers_int = VectorXi::Zero(motor_numbers.size());
    for (Eigen::Index i = 0; i < motor_numbers.size(); ++i)
    {
        motor_numbers_int(i) = static_cast<int>(motor_numbers(i));
    }
    return std::make_shared<JointModules>(robot_if,
                                          motor_numbers_int,
                                          motor_constants,
                                          gear_ratios,
                                          max_currents,
                                          reverse_polarities,
                                          lower_joint_limits,
                                          upper_joint_limits,
                                          max_joint_velocities,
                                          safety_damping);
}

std::shared_ptr<MasterBoardInterface> CreateMasterBoardInterfaceDefaults(
    const std::string& if_name)
{
    return CreateMasterBoardInterface(if_name);
}

std::shared_ptr<Robot> RobotFromYamlFileAndIfConfig(
    const std::string& if_name, const std::string& file_path)
{
    return RobotFromYamlFile(if_name, file_path);
}

std::shared_ptr<Robot> RobotOnlyFromYamlFile(const std::string& file_path)
{
    return RobotFromYamlFile(file_path);
}

void ReportErrorVerbose(Robot& r, const std::string err_msg)
{
    r.ReportError(err_msg);
}

void ReportErrorQuiet(Robot& r)
{
    r.ReportError();
}

BOOST_PYTHON_MODULE(libodri_control_interface_pywrap)
{
    // Enabling eigenpy support, i.e. numpy/eigen compatibility.
    eigenpy::enableEigenPy();
    eigenpy::enableEigenPySpecific<VectorXi>();
    eigenpy::enableEigenPySpecific<VectorXl>();
    eigenpy::enableEigenPySpecific<VectorXb>();
    eigenpy::enableEigenPySpecific<Eigen::VectorXd>();

    /* Operation on the master_board_sdk
     * - Import the python binding of the master_board_sdk
     * - Make available the std::shared_ptr<MasterBoardInterface> type.
     */
    boost::python::import("libmaster_board_sdk_pywrap");
    register_ptr_to_python<std::shared_ptr<MasterBoardInterface>>();

    void (JointModules::*EnableIndexOffsetCompensation0)(int) =
        &JointModules::EnableIndexOffsetCompensation;
    void (JointModules::*EnableIndexOffsetCompensation1)() =
        &JointModules::EnableIndexOffsetCompensation;

    // JointModules bindings and it's std::shared_ptr.
    class_<JointModules>("JointModules", no_init)
        .def("__init__", make_constructor(&joint_modules_constructor))
        .def("enable", &JointModules::Enable)
        .def("set_torques", &JointModules::SetTorques)
        .def("set_desired_positions", &JointModules::SetDesiredPositions)
        .def("set_desired_velocities", &JointModules::SetDesiredVelocities)
        .def("set_position_gains", &JointModules::SetPositionGains)
        .def("set_velocity_gains", &JointModules::SetVelocityGains)
        .def("set_zero_gains", &JointModules::SetZeroGains)
        .def("set_zero_commands", &JointModules::SetZeroCommands)
        .def("set_position_offsets", &JointModules::SetPositionOffsets)
        .def("enable_index_offset_compensation", EnableIndexOffsetCompensation0)
        .def("enable_index_offset_compensation", EnableIndexOffsetCompensation1)
        .def("set_maximum_current", &JointModules::SetMaximumCurrents)
        .def("disable_joint_limit_check", &JointModules::DisableJointLimitCheck)
        .def("enable_joint_limit_check", &JointModules::EnableJointLimitCheck)
        .add_property(
            "ready",
            make_function(&JointModules::GetReady,
                          return_value_policy<copy_const_reference>()))
        .add_property(
            "enabled",
            make_function(&JointModules::GetEnabled,
                          return_value_policy<copy_const_reference>()))
        .add_property(
            "saw_all_indices", &JointModules::SawAllIndices)
        .add_property(
            "is_ready", &JointModules::IsReady)
        .add_property("has_error", &JointModules::HasError)
        .add_property("number_motors", &JointModules::GetNumberMotors)
        .add_property(
            "positions",
            make_function(&JointModules::GetPositions,
                          return_value_policy<copy_const_reference>()))
        .add_property(
            "velocities",
            make_function(&JointModules::GetVelocities,
                          return_value_policy<copy_const_reference>()))
        .add_property(
            "sent_torques",
            make_function(&JointModules::GetSentTorques,
                          return_value_policy<copy_const_reference>()))
        .add_property(
            "measured_torques",
            make_function(&JointModules::GetMeasuredTorques,
                          return_value_policy<copy_const_reference>()))
        .add_property(
            "gear_ratios",
            make_function(&JointModules::GetGearRatios,
                          return_value_policy<copy_const_reference>()));
    register_ptr_to_python<std::shared_ptr<JointModules>>();

    // JointModules bindings and it's std::shared_ptr.
    class_<IMU>("IMU", init<std::shared_ptr<MasterBoardInterface>>())
        .def(init<std::shared_ptr<MasterBoardInterface>,
                  RefVectorXl,
                  RefVectorXl>())
        .add_property("has_error", &IMU::HasError)
        .add_property(
            "robot_interface",
            make_function(&IMU::GetMasterBoardInterface,
                          return_value_policy<copy_const_reference>()))
        .add_property(
            "gyroscope",
            make_function(&IMU::GetGyroscope,
                          return_value_policy<copy_const_reference>()))
        .add_property(
            "accelerometer",
            make_function(&IMU::GetAccelerometer,
                          return_value_policy<copy_const_reference>()))
        .add_property(
            "linear_acceleration",
            make_function(&IMU::GetLinearAcceleration,
                          return_value_policy<copy_const_reference>()))
        .add_property(
            "attitude_euler",
            make_function(&IMU::GetAttitudeEuler,
                          return_value_policy<copy_const_reference>()))
        .add_property(
            "attitude_quaternion",
            make_function(&IMU::GetAttitudeQuaternion,
                          return_value_policy<copy_const_reference>()));
    register_ptr_to_python<std::shared_ptr<IMU>>();

    // CalibrationMethod enum bindings.
    enum_<CalibrationMethod>("CalibrationMethod")
        .value("auto", CalibrationMethod::AUTO)
        .value("positive", CalibrationMethod::POSITIVE)
        .value("negative", CalibrationMethod::NEGATIVE)
        .value("alternative", CalibrationMethod::ALTERNATIVE);

    bool (Robot::*RunCalibration0)(Eigen::VectorXd const&) = &Robot::RunCalibration;
    bool (Robot::*RunCalibration1)(const std::shared_ptr<JointCalibrator>&,
        Eigen::VectorXd const&) = &Robot::RunCalibration;

    class_<Robot>("Robot",
                  init<std::shared_ptr<MasterBoardInterface>,
                       std::shared_ptr<JointModules>,
                       std::shared_ptr<IMU>,
                       std::shared_ptr<JointCalibrator>>())
        .def("init", &Robot::Init)
        .def("sendInit", &Robot::SendInit)
        .def("start", &Robot::Start)
        .def("wait_until_ready", &Robot::WaitUntilReady)
        .def("initialize", &Robot::Initialize)
        .def("parse_sensor_data", &Robot::ParseSensorData)
        .def("send_command", &Robot::SendCommand)
        .def("send_command_and_wait_end_of_cycle",
             &Robot::SendCommandAndWaitEndOfCycle)
        .def("run_calibration", RunCalibration0)
        .def("run_calibration", RunCalibration1)
        .def("report_error", &ReportErrorVerbose)
        .def("report_error", &ReportErrorQuiet)
        .add_property(
            "robot_interface",
            make_function(&Robot::GetRobotInterface,
                          return_value_policy<copy_const_reference>()))
        .add_property(
            "joints",
            make_function(&Robot::GetJoints,
                          return_value_policy<copy_const_reference>()))
        .add_property(
            "imu",
            make_function(&Robot::GetIMU,
                          return_value_policy<copy_const_reference>()))
        .add_property("is_ready", &Robot::IsReady)
        .add_property("is_timeout", &Robot::IsTimeout)
        .add_property("is_ack_msg_received", &Robot::IsAckMsgReceived)
        .add_property("has_error", &Robot::HasError);
    register_ptr_to_python<std::shared_ptr<Robot>>();

    class_<JointCalibrator>("JointCalibrator", no_init)
        .def("__init__", make_constructor(&joint_calibrator_constructor))
        .def("run", &JointCalibrator::Run)
        .add_property(
            "position_offsets",
            make_function(&JointCalibrator::GetPositionOffsets,
                          return_value_policy<copy_const_reference>()))
        .add_property(
            "dt",
            make_function(&JointCalibrator::dt,
                          return_value_policy<copy_const_reference>()));
    register_ptr_to_python<std::shared_ptr<JointCalibrator>>();

    def("robot_from_yaml_file", &RobotOnlyFromYamlFile);
    def("robot_from_yaml_file", &RobotFromYamlFileAndIfConfig);
    def("joint_calibrator_from_yaml_file", &JointCalibratorFromYamlFile);
}
