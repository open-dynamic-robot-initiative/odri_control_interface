#include <odri_control_interface/robot.hpp>
#include <odri_control_interface/calibration.hpp>

#include <odri_control_interface/utils.hpp>

using namespace odri_control_interface;

#include <stdexcept>
#include <iostream>

typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<bool, 12, 1> Vector12b;

typedef Eigen::Matrix<long, 3, 1> Vector3l;
typedef Eigen::Matrix<long, 4, 1> Vector4l;
typedef Eigen::Matrix<long, 12, 1> Vector12l;
typedef Eigen::Matrix<int, 12, 1> Vector12i;

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        throw std::runtime_error("Please provide the interface name "
                                 "(i.e. using 'ifconfig' on linux");
    }

    nice(-20); // Give the process a high priority.

    auto main_board_ptr_ = std::make_shared<MasterBoardInterface>(argv[1]);

    Vector12i motor_numbers;
    motor_numbers << 0, 3, 2, 1, 5, 4, 6, 9, 8, 7, 11, 10;
    Vector12b motor_reversed;
    motor_reversed << true, false, true, true, false, false, true, false, true, true, false, false;

    Vector12d joint_lower_limits;
    joint_lower_limits << -1.2, -1.7, -3.4, -1.2, -1.7, -3.4,
                          -1.2, -1.7, -3.4, -1.2, -1.7, -3.4;
    Vector12d joint_upper_limits;
    joint_upper_limits << 1.2,  1.7, +3.4, +1.2, +1.7, +3.4,
                          1.2,  1.7, +3.4, +1.2, +1.7, +3.4;

    // Define the joint module.
    auto joints = std::make_shared<JointModules>(
        main_board_ptr_,
        motor_numbers,
        0.025, 9., 1.,
        motor_reversed,
        joint_lower_limits, joint_upper_limits, 80., 0.5
    );

    // Define the IMU.
    Vector3l rotate_vector;
    Vector4l orientation_vector;
    rotate_vector << 1, 2, 3;
    orientation_vector << 1, 2, 3, 4;
    auto imu = std::make_shared<IMU>(main_board_ptr_, rotate_vector, orientation_vector);

    // Define the robot.
    auto robot = std::make_shared<Robot>(
        main_board_ptr_, joints, imu
    );

    // Start the robot.
    robot->Start();

    // Define controller to calibrate the joints.
    std::vector<CalibrationMethod> directions = {
                    AUTO, AUTO, AUTO, AUTO, AUTO, AUTO,
                    AUTO, AUTO, AUTO, AUTO, AUTO, AUTO
    };
    Vector12d position_offsets;
    position_offsets << 0.184, -0.370, -0.005, -.150,
                        0., 0.310, 0.165, 0.150,
                        0.365, 0.202, -0.175, -0.175;

    auto calib_ctrl = std::make_shared<JointCalibrator>(
        joints,
        directions,
        position_offsets,
        5., 0.05, 2., 0.001
    );

    Vector12d torques;

    double kp = 5.;
    double kd = 0.1;
    int c = 0;
    std::chrono::time_point<std::chrono::system_clock> last = std::chrono::system_clock::now();
    bool is_calibrated = false;
    while (!robot->IsTimeout())
    {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now() - last)).count() > 0.001)
		{
            last = std::chrono::system_clock::now(); //last+dt would be better
            robot->ParseSensorData();

            if (robot->IsReady()) {
                if (!is_calibrated) {
                    is_calibrated = calib_ctrl->Run();
                    if (is_calibrated) {
                        std::cout << "Calibration is done." << std::endl;
                    }
                } else {
                    // Run the main controller.
                    auto pos = robot->joints->GetPositions();
                    auto vel = robot->joints->GetVelocities();
                    // Reverse the positions;
                    for(int i = 0; i < 12; i++) {
                        torques[i] = -kp * pos[i] - kd * vel[i];
                    }
                    robot->joints->SetTorques(torques);
                }
            }

            // Checks if the robot is in error state (that is, if any component
            // returns an error). If there is an error, the commands to send
            // are changed to send the safety control.
            robot->SendCommand();

            c++;
            if (c % 1000 == 0) {
                std::cout << "Joints: ";
                joints->PrintVector(joints->GetPositions());
                std::cout << std::endl;
            }

        } else {
			std::this_thread::yield();
        }
    }
    std::cout << "Normal program shutdown." << std::endl;
    return 0;
}
