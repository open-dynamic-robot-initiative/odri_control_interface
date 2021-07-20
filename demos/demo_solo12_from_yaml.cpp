#include <odri_control_interface/calibration.hpp>
#include <odri_control_interface/robot.hpp>

#include <odri_control_interface/utils.hpp>

using namespace odri_control_interface;

#include <iostream>
#include <stdexcept>

typedef Eigen::Matrix<double, 12, 1> Vector12d;

int main()
{
    nice(-20);  // Give the process a high priority.

    // Define the robot from a yaml file.
    auto robot = RobotFromYamlFile(CONFIG_SOLO12_YAML);

    // Start the robot.
    robot->Start();

    // Define controller to calibrate the joints from yaml file.
    auto calib_ctrl =
        JointCalibratorFromYamlFile(CONFIG_SOLO12_YAML, robot->joints);

    Vector12d pos_init;  // Target position at the end of the calibration
    pos_init << 0.0, 0.7, -1.4, -0.0, 0.7, -1.4, 0.0, -0.7, +1.4, -0.0, -0.7,
        +1.4;
    double kp = 3.0;
    double kd = 0.05;
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
                    is_calibrated = calib_ctrl->RunAndGoTo(pos_init);
                    if (is_calibrated)
                    {
                        std::cout << "Calibration is done." << std::endl;
                    }
                }
                else
                {
                    // Set all command quantities
                    robot->joints->SetTorques(Eigen::VectorXd::Zero(12));
                    robot->joints->SetDesiredPositions(pos_init);
                    robot->joints->SetDesiredVelocities(
                        Eigen::VectorXd::Zero(12));
                    robot->joints->SetPositionGains(kp *
                                                    Eigen::VectorXd::Ones(12));
                    robot->joints->SetVelocityGains(kd *
                                                    Eigen::VectorXd::Ones(12));
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
                robot->joints->PrintVector(robot->joints->GetPositions());
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
