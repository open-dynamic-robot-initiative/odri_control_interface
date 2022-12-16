# Calibration procedure

* After assembly, a calibration procedure must be performed to retrieve the offsets between the **zero positions of the motors** (position of the indexes on the [coding wheels](https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware/blob/master/mechanics/actuator_module_v1/actuator_module_v1.1.md#description-odri-encoder-kit)) and the **positions you want to consider as zero** for your control.

* The [calibration script](https://github.com/open-dynamic-robot-initiative/odri_control_interface/blob/main/demos/robot_calibrator.py) fetches information from a [yaml configuration file](https://github.com/open-dynamic-robot-initiative/odri_control_interface/blob/main/demos/config_solo12.yaml), it can thus be used to calibrate all robots as long as the information contained in it is correct. Please check the [associated documentation](https://github.com/open-dynamic-robot-initiative/odri_control_interface/blob/main/documentation/configuration_file.dm) before proceeding further.

### Launching the script

* First, turn on the robot and roughtly place its joints in the configuration you want to consider as zero. 

<img src="https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware/blob/master/mechanics/quadruped_robot_12dof_v1/images/solo12_coordinate_systems.png" width="400"><br>*Solo 12 quadruped with its joints in zero positions.*<br>

* You can then launch the calibration script. The robot will slightly move its joints to find the indexes of the coding wheels. The calibration script can be launched as follows:
```
sudo -E python3 robot_calibrator.py -c path_to_yaml_config_file
```

* `path_to_yaml_config_file` is the path to the configuration file of the robot you want to calibrate. It can be for instance `config_solo12.yaml` if your script has been launched from the [demos folder](https://github.com/open-dynamic-robot-initiative/odri_control_interface/tree/main/demos) of this repository.

* Once all indexes have been found, the robot keeps running with **no commands sent to its joints** to let you manually place the joints in the configuration you want to consider as zero. If available, you can use the provided [calibration tools](https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware/blob/master/mechanics/general/robot_calibration.md#robot-calibration) to **mechanically** place the robot in zero positions and avoid relying on visual estimation which can lead to small alignement errors.

<img src="https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware/blob/master/mechanics/general/images/quadruped_12dof_calibration_1.jpg" width="400"><br>*Calibration tools placed on the Solo 12 quadruped.*<br>

* When you are satisfied with the joint placement, press the **Enter** key on your keyboard. This will stop the calibration script and output position offsets that you can copy-paste in the position_offsets field of the [yaml configuration file](https://github.com/open-dynamic-robot-initiative/odri_control_interface/blob/main/demos/config_solo12.yaml). These new offsets will be automatically taken into account by the communication interface after you restart the robot.

* For information purpose, the absolute difference between the offsets contained in the configuration file and the offsets found by the calibration will be displayed as well.
