# Configuration file

### Robot

The robot node contains general information about the robot such as motor ordering, joint limits or orientation of the IMU with respect to the body frame.

|Field|Meaning|Example value (Solo 12)|
|---|---|---|
|interface|Name of the interface to communicate with the robot (you can check with `ifconfig` if you are unsure).|enp5s0f1|
|motor_numbers|Mapping between the [hardware order of the motors and their control order]((https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware/blob/master/mechanics/quadruped_robot_12dof_v1/README.md#micro-driver-stack-motor-assignment).|[0, 3, 2, 1, 5, 4, 6, 9, 8, 7, 11, 10]|
|motor_constants|Torque constant of the motors in **Nm/A**.|0.025|
|gear_ratios|Gear ratio of the actuator modules.|9|
|max_currents|Absolute limit for the current sent into the motors in **A**.|8|
|reverse_polarities|Whether motors have been assembled on the robot in such a way that their hardware positive direction of rotation is actually reversed compared to the control positive direction of rotation.|[false, true, true, true, false, false, false, true, true, true, false, false]|
|lower_joint_limits|Lower joint limits for the actuators, under which the low-level control automatically switches to a security mode which apply damping on all joints.|[-0.9, -1.45, -2.8, -0.9, -1.45, -2.8, -0.9, -1.45, -2.8, -0.9, -1.45, -2.8]|
|upper_joint_limits|Upper joint limits for the actuators, above which the low-level control automatically switches to a security mode which apply damping on all joints.|[0.9, 1.45, 2.8, 0.9, 1.45, 2.8, 0.9, 1.45, 2.8, 0.9, 1.45, 2.8]|
|max_joint_velocities|Absolute limit for the joint velocities, above which the low-level control automatically switches to a security mode which apply damping on all joints.|80|
|safety_damping|Value of the damping coefficient for the security mode, in Nm/(rad/s).|0.2|
|rotate_vector|Mapping from IMU axes to body axes.|[1, 2, 3]|
|orientation_vector|Mapping from IMU orientation (as a quaternion) to body orientation. Can be useful if the IMU is mounted upside-down on the robot.|[1, 2, 3, 4]|

### Joint Calibrator

The joint calibrator node contains information for the calibration step at startup during which the robot slightly moves its joints to find the indexes of the coding wheels.

|Field|Meaning|Example (Solo 12)
|---|---|---|
|search_methods|The index search strategy for each motor, can be POS, NEG, ALT, AUTO. If POS, the motor will search the index by going into the positive direction from its starting position until it reaches it. If NEG, it will go into the negative direction. If ALT, the motor will do back-and-forth movements of increasing amplitude around its starting position till the indexes is reached. AUTO automatically chooses the startegy based on the position offset value.|[AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO]|
|position_offsets|Angular position offset between the position of the index on the coding wheel and the zero position of the actuator.|[0.142, -0.251, 0.298, -0.240, -0.247, -0.267, -0.155, -0.109, -0.095, 0.098, 0.271, -0.2476]|
|calib_order|Optional calibration order for the joints. Joints with the lowest values start looking for the indexes first while the others are not moving. Once they have found it, they wait in their associated position in `calib_pos` field. You can set all values to 0 if you want to calibrate all joints at the same time. With the example values, the HFE and Knee joints of the leg are calibrated first, then the leg fold up. That way there is less risk to hit something while the HAA joints are moving to find their indexes.|[1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0]|
|calib_pos|The waiting position for joints after they have found their indexes (used with `calib_order`).|[0.0, 1.2, -2.4, 0.0, 1.2, -2.4, 0.0, -1.2, 2.4, 0.0, -1.2,  2.4]|
|Kp|The proportional gain of the PD controller used during the calibration process, in Nm/rad.|3|
|Kd|The derivative gain of the PD controller used during the calibration process, in Nm/(rad/s).|0.05|
|T|The period of the sinus movement used when looking for indexes, in seconds.|1|
|dt|The control time step during the calibration process, in seconds.|0.001|
