#! /usr/bin/env python

# Run a PD controller to keep the zero position.

import pathlib

import numpy as np
np.set_printoptions(suppress=True, precision=2)

import libodri_control_interface_pywrap as oci

# Create the robot object from yaml.
robot = oci.robot_from_yaml_file("config_testbench.yaml")

# Store initial position data.
des_pos = np.array(
    [0.0])

# Initialize the communication, session, joints, wait for motors to be ready
# and run the joint calibration.
robot.initialize(des_pos)

c = 0
while not robot.is_timeout:
    # read data
    robot.parse_sensor_data()
    positions = robot.joints.positions
    velocities = robot.joints.velocities

    # Compute the PD control
    des_pos = [0.3 * np.sin(c/100)]
    torques = 6.0 * (des_pos - positions) - 0.1 * velocities
    # torques = torques * np.array([0, 0, 0, 0, 0, 1])
    # torques = np.zeros(6)
    robot.joints.set_torques(torques)

    # send command
    robot.send_command_and_wait_end_of_cycle(0.001)
    c += 1

    # save cmd and measure
    pos_mea.append(positions)
    pos_cmd.append(des_pos)

    # Update the robot state in the visualizer
    robot_visualizer.set_joint_positions([0, 1, 2, 4, 5, 6], positions)
    robot_visualizer.set_base_orientation(imu_attitude)
    robot_visualizer.update_visualization()
