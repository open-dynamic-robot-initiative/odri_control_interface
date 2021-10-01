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

#include <fstream>
#include <odri_control_interface/common.hpp>
#include <odri_control_interface/utils.hpp>
#include <sstream>

namespace odri_control_interface
{
#define assert_yaml_parsing(yaml_node, parent_node_name, child_node_name)      \
    if (!yaml_node[child_node_name])                                           \
    {                                                                          \
        std::ostringstream oss;                                                \
        oss << "Error: Wrong parsing of the YAML file from src file: ["        \
            << __FILE__ << "], in function: [" << __FUNCTION__ << "], line: [" \
            << __LINE__ << ". Node [" << child_node_name                       \
            << "] does not exists under the node [" << parent_node_name        \
            << "].";                                                           \
        throw std::runtime_error(oss.str());                                   \
    }                                                                          \
    assert(true)

#define assert_file_exists(filename)                                    \
    std::ifstream f(filename.c_str());                                  \
    if (!f.good())                                                      \
    {                                                                   \
        std::ostringstream oss;                                         \
        oss << "Error: Problem opening the file [" << filename          \
            << "], from src file: [" << __FILE__ << "], in function: [" \
            << __FUNCTION__ << "], line: [" << __LINE__                 \
            << ". The file may not exists.";                            \
        throw std::runtime_error(oss.str());                            \
    }                                                                   \
    assert(true)

std::shared_ptr<JointModules> JointModulesFromYaml(
    std::shared_ptr<MasterBoardInterface> robot_if,
    const YAML::Node& joint_modules_yaml)
{
    assert_yaml_parsing(joint_modules_yaml, "joint_modules", "motor_numbers");
    const YAML::Node& motor_numbers = joint_modules_yaml["motor_numbers"];
    std::size_t n = motor_numbers.size();
    VectorXi motor_numbers_vec;
    motor_numbers_vec.resize(n);
    for (std::size_t i = 0; i < n; i++)
    {
        motor_numbers_vec(i) = motor_numbers[i].as<int>();
    }

    assert_yaml_parsing(
        joint_modules_yaml, "joint_modules", "reverse_polarities");
    const YAML::Node& rev_polarities = joint_modules_yaml["reverse_polarities"];
    if (rev_polarities.size() != n)
    {
        throw std::runtime_error(
            "Motor polarities has different size than motor "
            "numbers");
    }
    VectorXb rev_polarities_vec;
    rev_polarities_vec.resize(n);
    for (std::size_t i = 0; i < n; i++)
    {
        rev_polarities_vec(i) = rev_polarities[i].as<bool>();
    }

    assert_yaml_parsing(
        joint_modules_yaml, "joint_modules", "lower_joint_limits");
    const YAML::Node& lower_joint_limits =
        joint_modules_yaml["lower_joint_limits"];
    if (lower_joint_limits.size() != n)
    {
        throw std::runtime_error(
            "Lower joint limits has different size than motor "
            "numbers");
    }
    VectorXd lower_joint_limits_vec;
    lower_joint_limits_vec.resize(n);
    for (std::size_t i = 0; i < n; i++)
    {
        lower_joint_limits_vec(i) = lower_joint_limits[i].as<double>();
    }

    assert_yaml_parsing(
        joint_modules_yaml, "joint_modules", "upper_joint_limits");
    const YAML::Node& upper_joint_limits =
        joint_modules_yaml["upper_joint_limits"];
    if (upper_joint_limits.size() != n)
    {
        throw std::runtime_error(
            "Upper joint limits has different size than motor "
            "numbers");
    }
    VectorXd upper_joint_limits_vec;
    upper_joint_limits_vec.resize(n);
    for (std::size_t i = 0; i < n; i++)
    {
        upper_joint_limits_vec(i) = upper_joint_limits[i].as<double>();
    }

    assert_yaml_parsing(joint_modules_yaml, "joint_modules", "motor_constants");
    assert_yaml_parsing(joint_modules_yaml, "joint_modules", "gear_ratios");
    assert_yaml_parsing(joint_modules_yaml, "joint_modules", "max_currents");
    assert_yaml_parsing(
        joint_modules_yaml, "joint_modules", "max_joint_velocities");
    assert_yaml_parsing(joint_modules_yaml, "joint_modules", "safety_damping");
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
    assert_yaml_parsing(imu_yaml, "imu", "rotate_vector");
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

    assert_yaml_parsing(imu_yaml, "imu", "orientation_vector");
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
    assert_yaml_parsing(joint_calibrator, "joint_calibrator", "search_methods");
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

    assert_yaml_parsing(
        joint_calibrator, "joint_calibrator", "position_offsets");
    const YAML::Node& position_offsets = joint_calibrator["position_offsets"];
    std::size_t n = position_offsets.size();
    VectorXd position_offsets_vec;
    position_offsets_vec.resize(n);
    for (std::size_t i = 0; i < n; i++)
    {
        position_offsets_vec(i) = position_offsets[i].as<double>();
    }

    VectorXi calib_order_vec;
    if (joint_calibrator["calib_order"])
    {
        const YAML::Node& calib_order = joint_calibrator["calib_order"];
        std::size_t n_order = calib_order.size();
        calib_order_vec.resize(n_order);
        for (std::size_t i = 0; i < n_order; i++)
        {
            calib_order_vec(i) = calib_order[i].as<int>();
        }
    }
    else
    {
        calib_order_vec.resize(n);
        calib_order_vec.setZero();
    }

    VectorXd calib_pos_vec;
    if (joint_calibrator["calib_pos"])
    {
        const YAML::Node& calib_pos = joint_calibrator["calib_pos"];
        std::size_t n_order = calib_pos.size();
        calib_pos_vec.resize(n_order);
        for (std::size_t i = 0; i < n_order; i++)
        {
            calib_pos_vec(i) = calib_pos[i].as<double>();
        }
    }
    else
    {
        calib_pos_vec.resize(n);
        calib_pos_vec.setZero();
    }

    assert_yaml_parsing(joint_calibrator, "joint_calibrator", "Kp");
    assert_yaml_parsing(joint_calibrator, "joint_calibrator", "Kd");
    assert_yaml_parsing(joint_calibrator, "joint_calibrator", "T");
    assert_yaml_parsing(joint_calibrator, "joint_calibrator", "dt");
    return std::make_shared<JointCalibrator>(
        joints,
        calib_methods,
        position_offsets_vec,
        calib_order_vec,
        calib_pos_vec,
        joint_calibrator["Kp"].as<double>(),
        joint_calibrator["Kd"].as<double>(),
        joint_calibrator["T"].as<double>(),
        joint_calibrator["dt"].as<double>());
}

std::shared_ptr<Robot> RobotFromYamlFile(const std::string& if_name,
                                         const std::string& file_path)
{
    assert_file_exists(file_path);
    YAML::Node param = YAML::LoadFile(file_path);

    // Parse the robot part.
    assert_yaml_parsing(param, file_path, "robot");
    const YAML::Node& robot_node = param["robot"];

    // 1. Create the robot interface.
    std::shared_ptr<MasterBoardInterface> robot_if =
        std::make_shared<MasterBoardInterface>(if_name);

    // 2. Create the joint modules.
    assert_yaml_parsing(robot_node, "robot", "joint_modules");
    std::shared_ptr<JointModules> joints =
        JointModulesFromYaml(robot_if, robot_node["joint_modules"]);

    // 3. Create the imu.
    assert_yaml_parsing(robot_node, "robot", "imu");
    std::shared_ptr<IMU> imu = IMUFromYaml(robot_if, robot_node["imu"]);

    // 4. Create the calibrator procedure.
    assert_yaml_parsing(param, file_path, "joint_calibrator");
    std::shared_ptr<JointCalibrator> calibrator =
        JointCalibratorFromYaml(joints, param["joint_calibrator"]);

    // 5. Create the robot instance from the objects.
    return std::make_shared<Robot>(robot_if, joints, imu, calibrator);
}

std::shared_ptr<Robot> RobotFromYamlFile(const std::string& file_path)
{
    assert_file_exists(file_path);
    YAML::Node param = YAML::LoadFile(file_path);

    // Parse the robot part.
    assert_yaml_parsing(param, file_path, "robot");
    assert_yaml_parsing(param["robot"], "robot", "interface");
    std::string if_name = param["robot"]["interface"].as<std::string>();

    return RobotFromYamlFile(if_name, file_path);
}

std::shared_ptr<JointCalibrator> JointCalibratorFromYamlFile(
    const std::string& file_path, std::shared_ptr<JointModules> joints)
{
    assert_file_exists(file_path);
    YAML::Node param = YAML::LoadFile(file_path);

    assert_yaml_parsing(param, file_path, "joint_calibrator");
    return JointCalibratorFromYaml(joints, param["joint_calibrator"]);
}

std::shared_ptr<MasterBoardInterface> CreateMasterBoardInterface(
    const std::string& if_name, bool listener_mode)
{
    return std::make_shared<MasterBoardInterface>(if_name, listener_mode);
}

}  // namespace odri_control_interface
