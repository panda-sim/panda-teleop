import pybullet as p
import pybullet_data
import numpy as np
import os
import time
import config
from robot import Panda
from teleop import KeyboardController, JoystickController


# create simulation and place camera
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# disable keyboard shortcuts so they do not interfere with keyboard control
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
p.resetDebugVisualizerCamera(cameraDistance=config.cameraDistance, 
                                cameraYaw=config.cameraYaw,
                                cameraPitch=config.cameraPitch, 
                                cameraTargetPosition=config.cameraTargetPosition)

# load the objects
urdfRootPath = pybullet_data.getDataPath()
plane = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.625])
table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.625])
cube = p.loadURDF(os.path.join(urdfRootPath, "cube_small.urdf"), basePosition=[0.6, -0.2, 0.05])

# load the robot
panda = Panda(basePosition=config.baseStartPosition,
                baseOrientation=p.getQuaternionFromEuler(config.baseStartOrientationE),
                jointStartPositions=config.jointStartPositions)

# teleoperation interface
teleop = KeyboardController()

# run simulation
state = panda.get_state()
target_position = state["ee-position"]
target_quaternion = state['ee-quaternion']
while True:
    # update the target pose
    action = teleop.get_action()
    target_position = target_position + action[0:3]
    target_quaternion = p.multiplyTransforms([0, 0, 0], p.getQuaternionFromEuler(action[3:6]),
                                                [0, 0, 0], target_quaternion)[1]
    # move to the target pose
    panda.move_to_pose(ee_position=target_position, ee_quaternion=target_quaternion)

    # open or close the gripper
    if action[-1] == -1:
        panda.open_gripper()
    elif action[-1] == +1:
        panda.close_gripper()

    # step simulation
    p.stepSimulation()
    time.sleep(config.control_dt)