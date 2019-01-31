#!/usr/bin/env python

from time import sleep

import rospy
from std_msgs.msg import String

from robotis_controller_msgs.srv import SetJointModule


class OP3Module(object):

    joint_list = [
        'head_pan', 'head_tilt',
        'l_sho_pitch', 'l_sho_roll', 'l_el', 'r_sho_pitch', 'r_sho_roll', 'r_el',
        'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw', 'l_knee', 'l_ank_pitch', 'l_ank_roll',
        'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ank_pitch', 'r_ank_roll'
    ]


    def __init__(self, module_name):
        # Initialize OP3 ROS publishers
        self.module_name = module_name
        self.set_joint_module_client = rospy.ServiceProxy('/robotis/set_present_joint_ctrl_modules', SetJointModule)

    def enable(self, joints=joint_list):
        # Call service to enable module
        service_name = '/robotis/set_present_joint_ctrl_modules'

        rospy.wait_for_service(service_name)

        try:
            self.set_joint_module_client.call(joints, [self.module_name for _ in range(len(joints))])
            sleep(0.5)
        except rospy.ServiceException:
            rospy.logerr('Failed to call service: {0}'.format(service_name))

