#include <odri_control_interface/robot.hpp>
#include <odri_control_interface/calibration.hpp>

using namespace odri_control_interface;

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        throw std::runtime_error("Please provide the interface name "
                                 "(i.e. using 'ifconfig' on linux");
    }

    nice(-20); // Give the process a high priority.

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
        0.025, 9., 1.,
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
    std::array<CalibrationMethod, 12> directions = {
        AUTO, AUTO, AUTO, AUTO, AUTO, AUTO,
        AUTO, AUTO, AUTO, AUTO, AUTO, AUTO
    };
    std::array<double, 12> position_offsets = {
        0.184, -0.370, 0.027, -.131,
        0., 0.310, 0., 0,
        0., 0., 0., 0
    };
    auto calib_ctrl = std::make_shared<JointCalibrator<12> >(
        joints,
        directions,
        position_offsets,
        2., 0.05, 0.001
    );

    double kp = 5.;
    double kd = 0.1;
    int c = 0;
    std::chrono::time_point<std::chrono::system_clock> last = std::chrono::system_clock::now();
    bool is_calibrated = false;
    while (!robot->is_timeout())
    {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > 0.001)
		{
            last = std::chrono::system_clock::now(); //last+dt would be better
            robot->parse_sensor_data();

            if (robot->is_ready()) {
                if (!is_calibrated) {
                    is_calibrated = calib_ctrl->run();
                    if (is_calibrated) {
                        std::cout << "Calibration is done." << std::endl;
                    }
                } else {
                    // Run the main controller.

                    auto pos = robot->joints->get_positions();
                    auto vel = robot->joints->get_velocities();
                    std::array<double, 12> torques;
                    // Reverse the positions;
                    for(int i = 0; i < 12; i++) {
                        torques[i] = -kp * pos[i] - kd * vel[i];
                    }
                    robot->joints->set_torques(torques);
                }
            }

            // Checks if the robot is in error state (that is, if any component
            // returns an error). If there is an error, the commands to send
            // are changed to send the safety control.
            robot->send_command();

            c++;
            if (c % 1000 == 0) {
                std::cout << "Joints: ";
                joints->print_array(joints->get_positions());
                std::cout << std::endl;
            }

        } else {
			std::this_thread::yield();
        }
    }
    std::cout << "Normal program shutdown." << std::endl;
    return 0;
}
