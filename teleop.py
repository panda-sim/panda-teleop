import pybullet as p
import numpy as np
import pygame


# class to handle keyboard inputs and convert to robot actions
# actions include end-effector delta pose + gripper command
class KeyboardController:

    # provide the step size for translation and rotation actions. Changing this will control how fast or slow the robot moves
    # maintain a map from keyboard keys to robot actions
    def __init__(self, pos_step=0.001, rot_step=0.001):
        self.pos_step = pos_step
        self.rot_step = rot_step

        self.key_map = {ord("w"): np.array([+1, 0, 0, 0, 0, 0, 0]),  # +x
                        ord("s"): np.array([-1, 0, 0, 0, 0, 0, 0]),  # -x
                        ord("a"): np.array([0, +1, 0, 0, 0, 0, 0]),  # +y
                        ord("d"): np.array([0, -1, 0, 0, 0, 0, 0]),  # -y
                        ord("e"): np.array([0, 0, +1, 0, 0, 0, 0]),  # +z
                        ord("q"): np.array([0, 0, -1, 0, 0, 0, 0]),  # -z

                        ord("i"): np.array([0, 0, 0, +1, 0, 0, 0]),  # roll +
                        ord("k"): np.array([0, 0, 0, -1, 0, 0, 0]),  # roll -
                        ord("j"): np.array([0, 0, 0, 0, +1, 0, 0]),  # pitch +
                        ord("l"): np.array([0, 0, 0, 0, -1, 0, 0]),  # pitch -
                        ord("u"): np.array([0, 0, 0, 0, 0, +1, 0]),  # yaw +
                        ord("o"): np.array([0, 0, 0, 0, 0, -1, 0]),  # yaw -

                        ord("z"): np.array([0, 0, 0, 0, 0, 0, -1]),  # open
                        ord("x"): np.array([0, 0, 0, 0, 0, 0, +1]),  # close
                        }

    # reads the keyboard inputs and converts them to robot actions
    # returns an array 
    def get_action(self):
        keys = p.getKeyboardEvents()
        action = np.zeros(7, dtype=np.float32)

        for k, v in self.key_map.items():
            if k in keys and (keys[k] & p.KEY_IS_DOWN):
                action += v

        action[:3] *= self.pos_step
        action[3:6] *= self.rot_step
        return action


# class to handle joystick inputs and convert to robot actions
# actions include end-effector delta pose + gripper command
class JoystickController():

    # provide the step size for translation and rotation actions. Changing this will control how fast or slow the robot moves
    # maintain a map from keyboard keys to robot actions
    def __init__(self, pos_step=0.001, rot_step=0.001):
        self.pos_step = pos_step
        self.rot_step = rot_step

        pygame.init()
        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        self.position_control = True

    # reads the keyboard inputs and converts them to robot actions
    # returns an array 
    # NOTE: You will need to modify the button and gamepad numbers to match the joystick you are using
    # this setup is for the [Logitech F310]
    def get_action(self):
        pygame.event.get()

        # switch between position and rotation control
        toggle_rotation = self.gamepad.get_button(4)
        toggle_position = self.gamepad.get_button(5)
        if not self.position_control and toggle_position:
            self.position_control = True
        elif self.position_control and toggle_rotation:
            self.position_control = False

        # press A to close gripper, B to open, joysticks to move
        # NOTE: You may want to change the signs/order for z1, z2, and z3
        # based on your perspective when teleoperating the robot
        self.A_pressed = self.gamepad.get_button(0)
        self.B_pressed = self.gamepad.get_button(1)
        gripper_action = 0
        if self.A_pressed:
            gripper_action = +1
        elif self.B_pressed:
            gripper_action = -1
        z1 = self.gamepad.get_axis(1) # Left stick (up-down) : (-1 to 1)
        z2 = self.gamepad.get_axis(0) # Left stick (left-right) : (-1 to 1)
        z3 = -self.gamepad.get_axis(4) # Right stick (up-down) : (-1 to 1)

        # convert to robot action
        if self.position_control:
            return np.array([self.pos_step*z1, self.pos_step*z2, self.pos_step*z3, 0, 0, 0, gripper_action])
        else:
            return np.array([0, 0, 0, self.rot_step*z1, self.rot_step*z2, self.rot_step*z3, gripper_action])