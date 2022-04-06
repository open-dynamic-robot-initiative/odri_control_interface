# coding: utf8

import argparse
import threading
import libodri_control_interface_pywrap as oci
import numpy as np


def get_input():
    keystrk = input()
    # thread doesn't continue until key is pressed
    # and so it remains alive


def control_loop(config_file_path):
    """Main function that initializes communication with the robot then waits
    until the user presses the Enter key to output joint positions offsets 

    Args:
        config_file_path (string): path to the yaml config file of the robot
    """

    # Load information about the robot from the yaml config file
    device = oci.robot_from_yaml_file(config_file_path)
    joint_calibrator = oci.joint_calibrator_from_yaml_file(config_file_path, device.joints)

    # Initiate communication with the device and calibrate encoders
    device.initialize(np.zeros(device.joints.number_motors))
    device.joints.set_zero_commands()
    device.parse_sensor_data()

    i = threading.Thread(target=get_input)
    i.start()
    print("Set all actuators to zero positions and press Enter")

    # Control loop
    while not device.is_timeout and i.is_alive():

        # Update sensor data (IMU, encoders, Motion capture)
        device.parse_sensor_data()

        # Send command to the robot
        device.send_command_and_wait_end_of_cycle(joint_calibrator.dt)

    m = np.pi / device.joints.gear_ratios
    offsets = np.mod(joint_calibrator.position_offsets - device.joints.positions + m, 2 * m) - m
    diff = np.mod(joint_calibrator.position_offsets - offsets + m, 2 * m) - m

    print("Difference with the previous position_offsets values:")
    print(np.array2string(np.abs(diff), precision=3, separator=', '))

    print("Values to copy-paste in the position_offsets field of the yaml config file:")
    print(np.array2string(offsets, precision=4, separator=', '))

    return 0


def main():
    """Main function
    """

    parser = argparse.ArgumentParser(description='Calibration script to get the joint position offsets of a robot.')
    parser.add_argument('-c',
                        '--config',
                        required=True,
                        help='Path of the config yaml file that contains information about the robot.')
    control_loop(parser.parse_args().config)
    quit()


if __name__ == "__main__":
    main()
