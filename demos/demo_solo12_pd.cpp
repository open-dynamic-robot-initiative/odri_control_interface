#include <odri_control_interface/calibration.hpp>
#include <odri_control_interface/robot.hpp>

using namespace odri_control_interface;

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        throw std::runtime_error(
            "Please provide the interface name "
            "(i.e. using 'ifconfig' on linux");
    }

    nice(-20);  // Give the process a high priority.

    auto main_board_ptr_ = std::make_shared<MasterBoardInterface>(argv[1]);

    std::array<int, 12> motor_numbers = {0, 3, 2, 1, 5, 4, 6, 9, 8, 7, 11, 10};
    std::array<bool, 12> motor_reversed = {true,
                                           false,
                                           true,
                                           true,
                                           false,
                                           false,
                                           true,
                                           false,
                                           true,
                                           true,
                                           false,
                                           false};

    std::array<double, 12> joint_lower_limits = {
        -1.2, -1.7, -3.4, -1.2, -1.7, -3.4, -1.2, -1.7, -3.4, -1.2, -1.7, -3.4};
    std::array<double, 12> joint_upper_limits = {
        1.2, 1.7, +3.4, +1.2, +1.7, +3.4, 1.2, 1.7, +3.4, +1.2, +1.7, +3.4};

    // Define the joint module.
    auto joints = std::make_shared<JointModules<12> >(main_board_ptr_,
                                                      motor_numbers,
                                                      0.025,
                                                      9.,
                                                      1.,
                                                      motor_reversed,
                                                      joint_lower_limits,
                                                      joint_upper_limits,
                                                      80.,
                                                      0.5);

    // Define the IMU.
    std::array<int, 3> rotate_vector = {1, 2, 3};
    std::array<int, 4> orientation_vector = {1, 2, 3, 4};
    auto imu = std::make_shared<IMU>(
        main_board_ptr_, rotate_vector, orientation_vector);

    // Define the robot.
    auto robot = std::make_shared<Robot<12> >(main_board_ptr_, joints, imu);

    // Start the robot.
    robot->Start();

    // Define controller to calibrate the joints.
    std::array<CalibrationMethod, 12> directions = {
        AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO};
    std::array<double, 12> position_offsets = {
        0.184, -0.370, -0.005, -.150, 0., 0.310, 0., 0, 0., 0., 0., 0};
    auto calib_ctrl = std::make_shared<JointCalibrator<12> >(
        joints, directions, position_offsets, 5., 0.05, 2., 0.001);

    std::array<double, 12> zero_array;
    std::array<double, 12> kp_array;
    std::array<double, 12> kd_array;
    kp_array.fill(5.0);
    kd_array.fill(0.1);
    zero_array.fill(0.);
    int c = 0;
    std::chrono::time_point<std::chrono::system_clock> last =
        std::chrono::system_clock::now();
    bool is_calibrated = false;
    while (!robot->IsTimeout())
    {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now() -
                                             last))
                .count() > 0.001)
        {
            last = std::chrono::system_clock::now();  // last+dt would be better
            robot->ParseSensorData();

            if (robot->IsReady())
            {
                if (!is_calibrated)
                {
                    is_calibrated = calib_ctrl->Run();
                    if (is_calibrated)
                    {
                        std::cout << "Calibration is done." << std::endl;
                    }
                }
                else
                {
                    // Run the main controller.

                    robot->joints->SetPositionGains(kp_array);
                    robot->joints->SetVelocityGains(kd_array);
                    robot->joints->SetDesiredPositions(zero_array);
                    robot->joints->SetDesiredVelocities(zero_array);
                    robot->joints->SetTorques(zero_array);
                }
            }

            // Checks if the robot is in error state (that is, if any component
            // returns an error). If there is an error, the commands to send
            // are changed to send the safety control.
            robot->SendCommand();

            c++;
            if (c % 1000 == 0)
            {
                std::cout << "Joints: ";
                joints->PrintArray(joints->GetPositions());
                std::cout << std::endl;
            }
        }
        else
        {
            std::this_thread::yield();
        }
    }
    std::cout << "Normal program shutdown." << std::endl;
    return 0;
}
