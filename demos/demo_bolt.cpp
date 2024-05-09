#include <odri_control_interface/calibration.hpp>
#include <odri_control_interface/robot.hpp>

#include <odri_control_interface/utils.hpp>

using namespace odri_control_interface;

#include <iostream>
#include <stdexcept>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

int main()
{
    nice(-20);  // Give the process a high priority.

    // Define the robot from a yaml file.
    auto robot = RobotFromYamlFile(CONFIG_BOLT_YAML);

    // Store initial position data.
    Vector6d des_pos;
    des_pos << 0.0, 0.8, -1.7, 0.0, 0.8, -1.7;

    // Initialize the communication, session, joints, wait for motors to be ready
    // and run the joint calibration.
    robot->Initialize(des_pos);

    // Initialize simple pd controller.
    Vector6d torques;
    double kp = 3.;
    double kd = 0.05;
    int c = 0;
    while (!robot->IsTimeout())
    {
        robot->ParseSensorData();

        // Run the main controller.
        auto pos = robot->joints->GetPositions();
        auto vel = robot->joints->GetVelocities();

        // Compute PD control on the zero position.
        for (int i = 0; i < 6; i++)
        {
            torques[i] = kp * (des_pos[i] - pos[i]) - kd * vel[i];
        }
        robot->joints->SetTorques(torques);

        // Checks if the robot is in error state (that is, if any component
        // returns an error). If there is an error, the commands to send
        // are changed to send the safety control.
        robot->SendCommandAndWaitEndOfCycle(0.001);

        c++;
        if (c % 1000 == 0)
        {
            std::cout << "Joints: ";
            robot->joints->PrintVector(robot->joints->GetPositions());
            std::cout << std::endl;
        }
    }

    printf("Timeout detected\n");

    return 0;
}
