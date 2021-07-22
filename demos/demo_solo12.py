#! /usr/bin/env python

# Run a PD controller to keep the zero position.

import pathlib

import numpy as np
np.set_printoptions(suppress=True, precision=2)

import libodri_control_interface_pywrap as oci

# Create the robot object from yaml.
robot = oci.robot_from_yaml_file("config_solo12.yaml")

# Initialize the communication, session, joints, wait for motors to be ready
# and run the joint calibration.ArithmeticError()
robot.initialize()

# Store initial position data.
des_pos = np.zeros(12)

c = 0
while not robot.is_timeout and not robot.has_error:
    robot.parse_sensor_data()

    imu_attitude = robot.imu.attitude_euler
    positions = robot.joints.positions
    velocities = robot.joints.velocities

    # Compute the PD control.
    torques = 5.0 * (des_pos - positions) - 0.1 * velocities
    robot.joints.set_torques(torques)

    robot.send_command_and_wait_end_of_cycle(0.001)
    c += 1

    if c % 2000 == 0:
        print("joint pos:   ", positions)
        print("joint vel:   ", velocities)
        print("torques:     ", torques)
        robot.robot_interface.PrintStats()
