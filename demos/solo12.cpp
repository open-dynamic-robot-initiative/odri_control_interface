#include <odri_control_interface/robot.hpp>

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        throw std::runtime_error("Please provide the interface name "
                                 "(i.e. using 'ifconfig' on linux");
    }

    auto main_board_ptr_ = std::make_shared<MasterBoardInterface>(argc[1]);
    
    // Define the joint module.
    auto joints = std::make_shared<JointModules<12> >(
        main_board_ptr_,
        {0, 3, 2, 1, 5, 4, 6, 9, 8, 7, 11, 10},
        0.025, 9., 9.,
        {true, false, true, true, false, false, true, false, true, true, false, false},
        {
            -1.2, -1.7, -3.4, -1.2, -1.7, -3.4,
            -1.2, -1.7, -3.4, -1.2, -1.7, -3.4
        },
        {
             1.2,  1.7, +3.4, +1.2, +1.7, +3.4,
             1.2,  1.7, +3.4, +1.2, +1.7, +3.4
        },
        80., 0.5
    );

    // Define the IMU.
    auto imu = std:make_shared<IMU>(main_board_ptr_,
        {0., 1., 2.}, {0., 1., 2., 3.});
    
    // Define the robot.
    auto robot = std::make_shared<Robot>(
        main_board_ptr_, joints, imu
    );

    // Start the robot.
    robot->send_init();

    // Check if the robot status is okay.
    if (robot->has_error()) {
        return -1;
    }

    while (!robot->has_error()) {
        auto pos = robot->joint->get_joint_positions();
        // Reverse the positions;
        for(int i = 0; i < 12; i++) {
            pos[i] = -pos[i];
        }
        robot->joint->set_torques(pos);

        // Checks if the robot is in error state (that is, if any component
        // returns an error). If there is an error, the commands to send
        // are changed to send the safety control.
        if (!robot->send_command()) {
            // An error was detected.
            return -1;
        }
    }
}
