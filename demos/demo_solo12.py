import numpy as np
np.set_printoptions(suppress=True, precision=2)

import time

import libmaster_board_sdk_pywrap as mbs
import libodri_control_interface_pywrap as oci

robot = oci.robot_from_yaml_file('config_solo12.yaml')
joint_calibrator = oci.joint_calibrator_from_yaml_file('config_solo12.yaml', robot.joints)

robot.start()
robot.wait_until_ready()

robot.parse_sensor_data()

# Read the values once here. The returned values are views on the data and
# update after the call to `robot.parse_sensor_data()`.
imu_attitude = robot.imu.attitude_euler
positions = robot.joints.positions
velocities = robot.joints.velocities

des_pos = np.zeros(12)

c = 0
dt = 0.001
calibration_done = False
next_tick = time.time()
while not robot.is_timeout:
  if time.time() >= next_tick:
    next_tick = next_tick + dt
    robot.parse_sensor_data()

    if c % 2000 == 0:
      print('IMU attitude:', imu_attitude)
      print('joint pos   :', positions)
      print('joint vel   :', velocities)
      robot.robot_interface.PrintStats()

    if not calibration_done:
      if joint_calibrator.run():
        calibration_done = True
    else:
      des_pos[:] = imu_attitude[2]
      torques = 5. * (des_pos - positions) - 0.1 * velocities
      robot.joints.set_torques(torques)

    robot.send_command()
    c += 1
  else:
    time.sleep(0.0001)
