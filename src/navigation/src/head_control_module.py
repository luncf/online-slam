#! /usr/bin/env python

import json
import os
from time import sleep

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from op3_module import OP3Module

class HeadControlModule(OP3Module):

    def __init__(self):
        super(HeadControlModule, self).__init__('head_control_module')

        # Initialize OP3 ROS publishers and service clients
        self.set_joint_state_pub = rospy.Publisher('/robotis/head_control/set_joint_states', JointState, queue_size=0)

    def enable(self):
        return super(HeadControlModule, self).enable(joints=[joint for joint in OP3Module.joint_list if joint.startswith('head')])

    def __move_actuator(self, actuator_names, positions, velocity=0):
        header = Header()
        header.seq = 0
        header.stamp = rospy.Time.now()
        header.frame_id = ''
        
        message = JointState()
        message.header = header
        message.name = actuator_names
        message.position = positions
        message.velocity = [velocity for _ in range(len(actuator_names))]
        message.effort = [0 for _ in range((len(actuator_names)))]

        self.set_joint_state_pub.publish(message)
        sleep(1)

    def move_head(self, pan_position, tilt_position):
        self.__move_actuator(actuator_names=['head_pan', 'head_tilt'],
                             positions=[pan_position, tilt_position])

    def look_up(self):
        self.move_head(pan_position=0, tilt_position=0)

    def look_down(self):
        self.move_head(pan_position=0, tilt_position=-0.7)

    def look_left(self):
        self.move_head(pan_position=0.75, tilt_position=-0.55)

    def look_right(self):
        self.move_head(pan_position=-0.75, tilt_position=-0.55)
