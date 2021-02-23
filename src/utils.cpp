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

#include <yaml-cpp/yaml.h>

#include <odri_control_interface/common.hpp>
#include <odri_control_interface/utils.hpp>

namespace odri_control_interface
{
std::shared_ptr<JointModules> JointModulesFromYaml(
    std::shared_ptr<MasterBoardInterface> robot_if,
    const YAML::Node& joint_modules_yaml)
{
    const YAML::Node& motor_numbers = joint_modules_yaml["motor_numbers"];
    std::size_t n = motor_numbers.size();
    VectorXi motor_numbers_vec;
    motor_numbers_vec.resize(n);
    for (std::size_t i = 0; i < n; i++)
    {
        motor_numbers_vec(i) = motor_numbers[i].as<int>();
    }

    const YAML::Node& rev_polarities = joint_modules_yaml["reverse_polarities"];
    if (rev_polarities.size() != n)
    {
        throw std::runtime_error(
            "Motor polarities has different size than motor numbers");
    }
    VectorXb rev_polarities_vec;
    rev_polarities_vec.resize(n);
    for (std::size_t i = 0; i < n; i++)
    {
        rev_polarities_vec(i) = rev_polarities[i].as<bool>();
    }

    const YAML::Node& lower_joint_limits =
        joint_modules_yaml["lower_joint_limits"];
    if (lower_joint_limits.size() != n)
    {
        throw std::runtime_error(
            "Lower joint limits has different size than motor numbers");
    }
    VectorXd lower_joint_limits_vec;
    lower_joint_limits_vec.resize(n);
    for (std::size_t i = 0; i < n; i++)
    {
        lower_joint_limits_vec(i) = lower_joint_limits[i].as<double>();
    }

    const YAML::Node& upper_joint_limits =
        joint_modules_yaml["upper_joint_limits"];
    if (upper_joint_limits.size() != n)
    {
        throw std::runtime_error(
            "Upper joint limits has different size than motor numbers");
    }
    VectorXd upper_joint_limits_vec;
    upper_joint_limits_vec.resize(n);
    for (std::size_t i = 0; i < n; i++)
    {
        upper_joint_limits_vec(i) = upper_joint_limits[i].as<double>();
    }

    return std::make_shared<JointModules>(
        robot_if,
        motor_numbers_vec,
        joint_modules_yaml["motor_constants"].as<double>(),
        joint_modules_yaml["gear_ratios"].as<double>(),
        joint_modules_yaml["max_currents"].as<double>(),
        rev_polarities_vec,
        lower_joint_limits_vec,
        upper_joint_limits_vec,
        joint_modules_yaml["max_joint_velocities"].as<double>(),
        joint_modules_yaml["safety_damping"].as<double>());
}

std::shared_ptr<IMU> IMUFromYaml(std::shared_ptr<MasterBoardInterface> robot_if,
                                 const YAML::Node& imu_yaml)
{
    const YAML::Node& rotate_vector = imu_yaml["rotate_vector"];
    VectorXl rotate_vector_vec;
    rotate_vector_vec.resize(3);
    if (rotate_vector.size() != 3)
    {
        throw std::runtime_error("Rotate vector not of size 3.");
    }
    for (int i = 0; i < 3; i++)
    {
        rotate_vector_vec(i) = rotate_vector[i].as<long>();
    }

    const YAML::Node& orientation_vector = imu_yaml["orientation_vector"];
    VectorXl orientation_vector_vec;
    orientation_vector_vec.resize(4);
    if (orientation_vector.size() != 4)
    {
        throw std::runtime_error("Rotate vector not of size 3.");
    }
    for (int i = 0; i < 4; i++)
    {
        orientation_vector_vec(i) = orientation_vector[i].as<long>();
    }

    return std::make_shared<IMU>(
        robot_if, rotate_vector_vec, orientation_vector_vec);
}

std::shared_ptr<JointCalibrator> JointCalibratorFromYaml(
    std::shared_ptr<JointModules> joints, const YAML::Node& joint_calibrator)
{
    std::vector<CalibrationMethod> calib_methods;
    for (std::size_t i = 0; i < joint_calibrator["search_methods"].size(); i++)
    {
        std::string method =
            joint_calibrator["search_methods"][i].as<std::string>();
        if (method == "AUTO")
        {
            calib_methods.push_back(CalibrationMethod::AUTO);
        }
        else if (method == "POS")
        {
            calib_methods.push_back(CalibrationMethod::POSITIVE);
        }
        else if (method == "NEG")
        {
            calib_methods.push_back(CalibrationMethod::NEGATIVE);
        }
        else if (method == "ALT")
        {
            calib_methods.push_back(CalibrationMethod::ALTERNATIVE);
        }
        else
        {
            throw std::runtime_error("Unknown search method '" + method + "'.");
        }
    }

    const YAML::Node& position_offsets = joint_calibrator["position_offsets"];
    std::size_t n = position_offsets.size();
    VectorXd position_offsets_vec;
    position_offsets_vec.resize(n);
    for (std::size_t i = 0; i < n; i++)
    {
        position_offsets_vec(i) = position_offsets[i].as<double>();
    }

    return std::make_shared<JointCalibrator>(
        joints,
        calib_methods,
        position_offsets_vec,
        joint_calibrator["Kp"].as<double>(),
        joint_calibrator["Kd"].as<double>(),
        joint_calibrator["T"].as<double>(),
        joint_calibrator["dt"].as<double>());
}

std::shared_ptr<Robot> RobotFromYamlFile(const std::string& if_name,
                                         const std::string& file_path)
{
    YAML::Node param = YAML::LoadFile(file_path);

    // Parse the robot part.
    const YAML::Node& robot_node = param["robot"];

    // 1. Create the robot interface.
    std::shared_ptr<MasterBoardInterface> robot_if =
        std::make_shared<MasterBoardInterface>(if_name);

    // 2. Create the joint modules.
    std::shared_ptr<JointModules> joints =
        JointModulesFromYaml(robot_if, robot_node["joint_modules"]);

    std::shared_ptr<IMU> imu = IMUFromYaml(robot_if, robot_node["imu"]);

    // 3. Create the robot instance from the objects.
    return std::make_shared<Robot>(robot_if, joints, imu);
}

std::shared_ptr<Robot> RobotFromYamlFile(const std::string& file_path)
{
    YAML::Node param = YAML::LoadFile(file_path);

    // Parse the robot part.
    std::string if_name = param["robot"]["interface"].as<std::string>();

    return RobotFromYamlFile(if_name, file_path);
}

std::shared_ptr<JointCalibrator> JointCalibratorFromYamlFile(
    const std::string& file_path, std::shared_ptr<JointModules> joints)
{
    YAML::Node param = YAML::LoadFile(file_path);

    return JointCalibratorFromYaml(joints, param["joint_calibrator"]);
}

std::shared_ptr<MasterBoardInterface> CreateMasterBoardInterface(
    const std::string& if_name, bool listener_mode)
{
    return std::make_shared<MasterBoardInterface>(if_name, listener_mode);
}

}  // namespace odri_control_interface
