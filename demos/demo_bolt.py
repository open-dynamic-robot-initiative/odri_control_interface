#! /usr/bin/env python
import pathlib
import numpy as np
import multiprocessing as mp
import time
import libodri_control_interface_pywrap as oci
from RobotVisualizer import RobotVisualizer
import matplotlib.pyplot as plt

# Define shared data between processes using `multiprocessing`'s shared memory
shared_positions = mp.Array('d', 6)  # Array for positions (6 doubles)
shared_imu_attitude = mp.Array('d', 4)  # Array for IMU attitude quaternion (4 doubles)
stop_flag = mp.Value('b', False)  # Shared boolean flag for stopping processes
robot_ready = mp.Value('b', False)  # Flag to indicate if the robot is initialized

# Visualization process
def visualization_loop(shared_positions, shared_imu_attitude, stop_flag, robot_ready):
    robot_visualizer = RobotVisualizer("/Users/croux/Devel/odri-rl-control/bolt_description/urdf/bolt_description_isaac.urdf")

    while not stop_flag.value:
        if not robot_ready.value:
            # Fill with zeros until the robot is initialized
            robot_visualizer.set_joint_positions([0, 1, 2, 4, 5, 6], [0] * 6)
            robot_visualizer.set_base_orientation([0, 0, 0, 1])
        else:
            with shared_positions.get_lock():
                positions = list(shared_positions)

            with shared_imu_attitude.get_lock():
                imu_attitude = list(shared_imu_attitude)

            robot_visualizer.set_joint_positions([0, 1, 2, 4, 5, 6], positions)
            robot_visualizer.set_base_orientation(imu_attitude)

        robot_visualizer.update_visualization()
        time.sleep(0.01)  # Update rate of 100 Hz

# Main process (Control Loop)
def main():
    global shared_positions, shared_imu_attitude, stop_flag, robot_ready

    # Start the visualizer process
    visualization_process = mp.Process(
        target=visualization_loop,
        args=(shared_positions, shared_imu_attitude, stop_flag, robot_ready)
    )
    visualization_process.start()

    try:
        # Initialize the robot
        robot = oci.robot_from_yaml_file("config_bolt.yaml")
        robot.initialize(np.zeros(6))
        with robot_ready.get_lock():
            robot_ready.value = True
        pos_cmd = []
        pos_mea = []
        c = 0
        
        while not robot.is_timeout and c < 10000000:
            # Read data
            robot.parse_sensor_data()
            imu_attitude = robot.imu.attitude_quaternion
            imu_attitude_euler = robot.imu.attitude_euler
            positions = robot.joints.positions
            velocities = robot.joints.velocities

            # Update shared data
            with shared_positions.get_lock():
                for i in range(6):
                    shared_positions[i] = positions[i]

            with shared_imu_attitude.get_lock():
                for i in range(4):
                    shared_imu_attitude[i] = imu_attitude[i]

            # Compute the PD control
            des_pos = np.array([0.3 * np.sin(c / 30 + i * np.pi / 6) for i in range(6)])
            # des_pos *= np.array([0, 1, 0, 0, 1, 1])

            robot.joints.set_position_gains(np.array([4.] * 6))
            robot.joints.set_velocity_gains(np.array([0.2] * 6))
            # robot.joints.set_position_gains(np.array([0.0] * 6))
            # robot.joints.set_velocity_gains(np.array([0.0] * 6))
            robot.joints.set_desired_positions(des_pos)
            robot.joints.set_desired_velocities(np.zeros(6))
            robot.joints.set_torques(np.zeros(6))

            # Send command
            robot.send_command_and_wait_end_of_cycle(0.001)
            c += 1

            # Save cmd and measure
            pos_mea.append(positions)
            pos_cmd.append(des_pos)

            if c % 100 == 0:
                print("IMU attitude:   ", imu_attitude_euler * 180 / np.pi)

        # Save data after loop ends
        np.savetxt("old_driver_measures.txt", pos_mea)
        np.savetxt("old_driver_cmd.txt", pos_cmd)

    finally:
        stop_flag.value = True  # Signal the visualizer process to stop
        visualization_process.join()

if __name__ == "__main__":
    main()
