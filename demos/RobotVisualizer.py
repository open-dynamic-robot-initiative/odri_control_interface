import pybullet as p
import pybullet_data
import time

class RobotVisualizer:
    def __init__(self, urdf_path):
        self.connect()
        self.load_robot(urdf_path)
        self.disable_gravity()

    def connect(self):
        # Connect to PyBullet
        p.connect(p.GUI)
        # Set the search path to find URDF files
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    def load_robot(self, urdf_path):
        # Load the robot URDF
        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True)
    
    def disable_gravity(self):
        # Disable gravity
        p.setGravity(0, 0, 0)

    def set_joint_positions(self, joint_indices, joint_positions):
        # Set the joint positions
        for i, idx in enumerate(joint_indices):
            p.resetJointState(self.robot_id, idx, joint_positions[i])

    def set_base_orientation(self, orientation):
        # Set the base orientation using quaternion (x, y, z, w)
        position, _ = p.getBasePositionAndOrientation(self.robot_id)
        p.resetBasePositionAndOrientation(self.robot_id, position, orientation)

    def update_visualization(self):
        p.stepSimulation()

