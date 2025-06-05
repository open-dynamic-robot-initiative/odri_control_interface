import numpy as np

np.set_printoptions(suppress=True, precision=2)

import libodri_control_interface_pywrap as oci

robot = oci.robot_from_yaml_file("config_testbench.yaml")

des_pos = np.array([np.pi / 2, -np.pi / 2])

robot.initialize(des_pos)

kp, kd = 0.125, 0.0025
c = 0
while not robot.is_timeout:
    robot.parse_sensor_data()

    positions = robot.joints.positions
    velocities = robot.joints.velocities

    # Compute the PD control on the zero position.
    torques = kp * (des_pos - positions) - kd * velocities

    robot.joints.set_torques(torques)
    robot.send_command_and_wait_end_of_cycle(0.001)

    c += 1

    if c % 1000 == 0:
        print("joint pos:   ", positions)
        print("joint vel:   ", velocities)
        print("torques:     ", torques)
        robot.robot_interface.PrintStats()

print("timeout detected")
