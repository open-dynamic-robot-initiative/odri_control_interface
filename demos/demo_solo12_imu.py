#! /usr/bin/env python
#
# Similar to `demo_solo12.py`. Uses the IMU attitude to control the desired
# joint positions.

import pathlib

import numpy as np
np.set_printoptions(suppress=True, precision=2)

import libodri_control_interface_pywrap as oci

# Create the robot object from yaml.
robot = oci.robot_from_yaml_file("config_solo12.yaml")

# Store initial position data.
init_imu_attitude = robot.imu.attitude_euler.copy()
des_pos = np.zeros(12)

# Initialize the communication, session, joints, wait for motors to be ready
# and run the joint calibration.ArithmeticError()
robot.initialize(des_pos)

c = 0
while not robot.is_timeout and not robot.has_error:
    robot.parse_sensor_data()

    imu_attitude = robot.imu.attitude_euler
    positions = robot.joints.positions
    velocities = robot.joints.velocities

    des_pos[:] = imu_attitude[2] - init_imu_attitude[2]
    torques = 5.0 * (des_pos - positions) - 0.1 * velocities
    robot.joints.set_torques(torques)

    robot.send_command_and_wait_end_of_cycle(0.001)
    c += 1

    if c % 2000 == 0:
        print("IMU attitude:", imu_attitude)
        print("joint pos:   ", positions)
        print("joint vel:   ", velocities)
        print("torques:     ", torques)
        robot.robot_interface.PrintStats()
