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

    // Initialize the communication, session, joints, wait for motors to be ready
    // and run the joint calibration.ArithmeticError()
    robot->Initialize();

    // Initialize simple pd controller.
    Vector12d torques;
    double kp = 5.;
    double kd = 0.1;
    int c = 0;
    bool is_calibrated = false;
    while (!robot->IsTimeout() && !robot->HasError())
    {
        robot->ParseSensorData();

        // Run the main controller.
        auto pos = robot->joints->GetPositions();
        auto vel = robot->joints->GetVelocities();

        // Compute PD control on the zero position.
        for (int i = 0; i < 12; i++)
        {
            torques[i] = kp * (0 - pos[i]) - kd * vel[i];
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

    if (robot->IsTimeout()) {
        printf("Timeout detected\n");
    } else if (robot->HasError()) {
        printf("Error detected\n");
    }

    return 0;
}
