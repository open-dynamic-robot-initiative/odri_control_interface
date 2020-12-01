#include <odri_control_interface/robot.hpp>

using namespace odri_control_interface;

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        throw std::runtime_error("Please provide the interface name "
                                 "(i.e. using 'ifconfig' on linux");
    }

    auto main_board_ptr_ = std::make_shared<MasterBoardInterface>(argv[1]);
    
    std::array<int, 12> motor_numbers = {0, 3, 2, 1, 5, 4, 6, 9, 8, 7, 11, 10};
    std::array<bool, 12> motor_reversed = {true, false, true, true, false, false, true, false, true, true, false, false};

    std::array<double, 12> joint_lower_limits = {
            -1.2, -1.7, -3.4, -1.2, -1.7, -3.4,
            -1.2, -1.7, -3.4, -1.2, -1.7, -3.4
    };
    std::array<double, 12> joint_upper_limits = {
             1.2,  1.7, +3.4, +1.2, +1.7, +3.4,
             1.2,  1.7, +3.4, +1.2, +1.7, +3.4
    };

    // Define the joint module.
    auto joints = std::make_shared<JointModules<12> >(
        main_board_ptr_,
        motor_numbers,
        0.025, 9., 9.,
        motor_reversed,
        joint_lower_limits, joint_upper_limits, 80., 0.5
    );

    // Define the IMU.
    // auto imu = std:make_shared<IMU>(main_board_ptr_,
    //     {0., 1., 2.}, {0., 1., 2., 3.});
    // Not defining an IMU for now.
    std::shared_ptr<IMU> imu = nullptr;
    
    // Define the robot.
    auto robot = std::make_shared<Robot<12> >(
        main_board_ptr_, joints, imu
    );

    // Start the robot.
    robot->start();

    // Define controller to calibrate the joints.
    // auto calib_ctrl = std::make_shared<JointCalibrator<12> >(
    //     joints, 
    //     {AUTO, AUTO, AUTO, AUTO, AUTO,  AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO},
    //     {-0.435291, -0.487721, -0.367817, -0.070000, -0.342973, -0.447689,
    //      -0.369107, -0.096331, -0.662285, +0.092616, -0.557637, -0.380633},
    //     2., 0.05, 0.001
    // );


    double kp = 2.;
    double kd = 0.05;
    // bool is_calibrated = false;
    while (!robot->has_error()) {
        robot->parse_sensor_data();

        // if (!is_calibrated) {
        //     is_calibrated = calib_ctrl->run();
        // } else {
            // Run the main controller.
            
        auto pos = robot->joints->get_positions();
        auto vel = robot->joints->get_velocities();
        std::array<double, 12> torques;
        // Reverse the positions;
        for(int i = 0; i < 12; i++) {
            torques[i] = -kp * pos[i] - kd * vel[i];
        }
        robot->joints->set_torques(torques);
        // }

        // Checks if the robot is in error state (that is, if any component
        // returns an error). If there is an error, the commands to send
        // are changed to send the safety control.
        if (!robot->send_command()) {
            // An error was detected.
            return -1;
        }
    }
    return 0;
}
