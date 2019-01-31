import json
import math
import os
from time import sleep

import roslib.packages as rospkg
import rospy
from std_msgs.msg import String

from op3_walking_module_msgs.msg import WalkingParam

from op3_module import OP3Module

class WalkingModule(OP3Module):
    
    def __init__(self):
        super(WalkingModule, self).__init__(module_name='walking_module')

        self.is_initial_walk = False

        # Initialize OP3 ROS publishers
        self.set_walking_command_pub = rospy.Publisher('/robotis/walking/command', String, queue_size=0)
        self.set_walking_param_pub = rospy.Publisher('/robotis/walking/set_params', WalkingParam, queue_size=0)
        self.module_control_pub = rospy.Publisher('/robotis/enable_ctrl_module', String, queue_size=0)

        # Initialize configuration param variables
        self.current_walking_param = WalkingParam()
        self.initial_param = None
        self.marching_param = None
        self.forward_param = None
        self.turn_left_param = None
        self.turn_right_param = None

        # Load configurations
        config_file_path = os.path.join(rospkg.get_pkg_dir('navigation'), 'config', 'walking_configurations.json')
        self.load_configuration(file_path=config_file_path)

    def enable(self):
        self.is_initial_walk = True

        message = String()
        message.data = 'walking_module'

        self.module_control_pub.publish(message)
        
        sleep(2)

    def load_configuration(self, file_path):
        rospy.loginfo('Loading walking configurations.')

        with open(file_path, 'r') as file:
            configuration = json.load(file)

            # Generic walking parameters
            self.current_walking_param.init_x_offset = configuration['init_x_offset']
            self.current_walking_param.init_y_offset = configuration['init_y_offset']
            self.current_walking_param.init_z_offset = configuration['init_z_offset']
            self.current_walking_param.init_roll_offset = configuration['init_roll_offset']
            self.current_walking_param.init_pitch_offset = configuration['init_pitch_offset']
            self.current_walking_param.init_yaw_offset = configuration['init_yaw_offset']
            self.current_walking_param.dsp_ratio = configuration['dsp_ratio']
            self.current_walking_param.step_fb_ratio = configuration['step_fb_ratio']
            self.current_walking_param.move_aim_on = configuration['move_aim_on']
            self.current_walking_param.balance_enable = configuration['balance_enable']
            self.current_walking_param.balance_hip_roll_gain = configuration['balance_hip_roll_gain']
            self.current_walking_param.balance_knee_gain = configuration['balance_knee_gain']
            self.current_walking_param.balance_ankle_roll_gain = configuration['balance_ankle_roll_gain']
            self.current_walking_param.balance_ankle_pitch_gain = configuration['balance_ankle_pitch_gain']
            self.current_walking_param.y_swap_amplitude = configuration['y_swap_amplitude']
            self.current_walking_param.z_swap_amplitude = configuration['z_swap_amplitude']
            self.current_walking_param.arm_swing_gain = configuration['arm_swing_gain']
            self.current_walking_param.pelvis_offset = configuration['pelvis_offset']
            self.current_walking_param.hip_pitch_offset = math.radians(configuration['hip_pitch_offset'])
            self.current_walking_param.p_gain = configuration['p_gain']
            self.current_walking_param.i_gain = configuration['i_gain']
            self.current_walking_param.d_gain = configuration['d_gain']

            # Initial dx, dy, xz, dtheta, period_time
            self.initial_param = {
                'x': configuration['initial']['x_move_amplitude'],
                'y': configuration['initial']['y_move_amplitude'],
                'z': configuration['initial']['z_move_amplitude'],
                'angle': math.radians(configuration['initial']['angle_move_amplitude']),
                'period_time': configuration['initial']['period_time']
            }

            # Marching dx, dy, dz, dtheta, period_time
            self.marching_param = {
                'x': configuration['marching']['x_move_amplitude'],
                'y': configuration['marching']['y_move_amplitude'],
                'z': configuration['marching']['z_move_amplitude'],
                'angle': math.radians(configuration['marching']['angle_move_amplitude']),
                'period_time': configuration['marching']['period_time']
            }

            # Forward dx, dy, dz, dtheta, period_time
            self.forward_param = {
                'x': configuration['forward']['x_move_amplitude'],
                'y': configuration['forward']['y_move_amplitude'],
                'z': configuration['forward']['z_move_amplitude'],
                'angle': math.radians(configuration['forward']['angle_move_amplitude']),
                'period_time': configuration['forward']['period_time']
            }

            # Turn left dx, dy, dz, dtheta, period_time
            self.turn_left_param = {
                'x': configuration['turn_left']['x_move_amplitude'],
                'y': configuration['turn_left']['y_move_amplitude'],
                'z': configuration['turn_left']['z_move_amplitude'],
                'angle': math.radians(configuration['turn_left']['angle_move_amplitude']),
                'period_time': configuration['turn_left']['period_time']
            }

            # Turn right dx, dy, dz, dtheta, period_time
            self.turn_right_param = {
                'x': configuration['turn_right']['x_move_amplitude'],
                'y': configuration['turn_right']['y_move_amplitude'],
                'z': configuration['turn_right']['z_move_amplitude'],
                'angle': math.radians(configuration['turn_right']['angle_move_amplitude']),
                'period_time': configuration['turn_right']['period_time']
            }

    def __update_walking_param(self, walking_param):
        self.current_walking_param.x_move_amplitude = walking_param['x']
        self.current_walking_param.y_move_amplitude = walking_param['y']
        self.current_walking_param.z_move_amplitude = walking_param['z']
        self.current_walking_param.angle_move_amplitude = walking_param['angle']
        self.current_walking_param.period_time = walking_param['period_time']

        self.set_walking_param_pub.publish(self.current_walking_param)
        sleep(1)

    def __send_walking_command(self, command='start'):
        message = String()
        message.data = command

        self.set_walking_command_pub.publish(message)
        sleep(1)

    def __walk(self, param_name, walking_param):
        rospy.loginfo('Switching walking gait: {0}'.format(param_name))

        if self.is_initial_walk:
            self.__update_walking_param(walking_param=self.initial_param)
            self.__send_walking_command(command='start')
            self.is_initial_walk = False
            sleep(2)

        self.__update_walking_param(walking_param=walking_param)

    def march(self):
        self.__walk(param_name='marching', walking_param=self.marching_param)

    def forward(self):
        self.__walk(param_name='forward', walking_param=self.forward_param)

    def turn_left(self):
        self.__walk(param_name='turn_left', walking_param=self.turn_left_param)

    def turn_right(self):
        self.__walk(param_name='turn_right', walking_param=self.turn_right_param)

    def stop(self):
        rospy.loginfo('Stopping walk.')
        self.__send_walking_command(command='stop')



