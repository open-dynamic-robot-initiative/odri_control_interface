#! /usr/bin/env python

import numpy as np

np.set_printoptions(suppress=True, precision=2)

import time

import libmaster_board_sdk_pywrap as mbs
import libodri_control_interface_pywrap as oci

# Testbench
robot_if = mbs.MasterBoardInterface("ens3")
joints = oci.JointModules(
    robot_if,
    np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]),
    0.025,
    1.0,
    1.0,
    np.array(
        [
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
        ]
    ),
    np.array(
        [
            -10.0,
            -10.0,
            -10.0,
            -10.0,
            -10.0,
            -10.0,
            -10.0,
            -10.0,
            -10.0,
            -10.0,
            -10.0,
            -10.0,
        ]
    ),
    np.array(
        [
            10.0,
            10.0,
            10.0,
            10.0,
            10.0,
            10.0,
            10.0,
            10.0,
            10.0,
            10.0,
            10.0,
            10.0,
        ]
    ),
    80.0,
    0.5,
)


imu = oci.IMU(robot_if)

robot = oci.Robot(robot_if, joints, imu)
robot.start()
robot.wait_until_ready()

# As the data is returned by reference, it's enough
# to get hold of the data one. It will update after
# each call to `robot.parse_sensor_data`.
imu_attitude = imu.attitude_euler
positions = joints.positions
velocities = joints.velocities

des_pos = np.zeros(12)

# Setup the calibration method
joint_offsets = np.array(
    [
        -1.58,
        1.24,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]
)
sdir = oci.CalibrationMethod.positive
joint_calibrator = oci.JointCalibrator(
    joints, 12 * [sdir], joint_offsets,
    np.zeros(12, dtype=int), np.zeros(12),
    5.0 / 20.0, 0.05 / 20.0, 2.0, 0.001
)

c = 0
dt = 0.001
calibration_done = False
next_tick = time.time()
while not robot.has_error:
    if time.time() >= next_tick:
        next_tick = next_tick + dt
        robot.parse_sensor_data()

        if c % 2000 == 0:
            print("IMU attitude:", imu_attitude)
            print("joint pos   :", positions)
            print("joint vel   :", velocities)

        if not calibration_done:
            if joint_calibrator.run():
                calibration_done = True
                joints.set_position_gains(5.0 * np.ones(12))
                joints.set_velocity_gains(0.001 * np.ones(12))
        else:
            des_pos[:] = imu_attitude[2]
            joints.set_desired_positions(des_pos)

        robot.send_command()
        c += 1
    else:
        time.sleep(0.0001)
