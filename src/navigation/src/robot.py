from enum import Enum
import json

import rospy

from slam.msg import RobotPosition


class RobotMovement(Enum):
    OBSTACLE = 0
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3


class Robot(object):

    def __init__(self, rate_config_path):
        self.row = -1
        self.column = -1
        self.angle = -1

        with open(rate_config_path) as config:
            rates = json.load(config)
        self.forward_rate = rates['forward']
        self.turn_left_rate = rates['turn_left']
        self.turn_right_rate = rates['turn_right']

    def update_pose(self, msg):
        self.row = msg.row
        self.column = msg.column
        self.angle = msg.angle
        rospy.loginfo('[Navigation] robot pose: row - {0:2d} column - {1:2d} angle - {2:1d}'. \
                      format(self.row, self.column, self.angle))

    def next_movement(self, avoid_obstacle, num_map_rows, num_map_columns, bound_offset=2):
        movement = None

        at_top_bound = self.row <= bound_offset
        at_bottom_bound = self.row >= num_map_rows - 1 - bound_offset
        at_left_bound = self.column <= bound_offset
        at_right_bound = self.column >= num_map_columns - 1 - bound_offset

        top_field = self.row <= num_map_rows / 2
        bottom_field = self.row > num_map_rows / 2
        left_field = self.column <= num_map_columns / 2
        right_field = self.column > num_map_columns / 2

        if avoid_obstacle:
            movement = RobotMovement.OBSTACLE

        elif (self.angle == 0 and bottom_field and at_right_bound) or \
                (self.angle == 1 and right_field and at_top_bound) or \
                (self.angle == 2 and top_field and at_left_bound) or \
                (self.angle == 3 and left_field and at_bottom_bound):
            movement = RobotMovement.TURN_LEFT

        elif (self.angle == 0 and top_field and at_right_bound) or \
                (self.angle == 1 and left_field and at_top_bound) or \
                (self.angle == 2 and bottom_field and at_left_bound) or \
                (self.angle == 3 and right_field and at_bottom_bound):
            movement = RobotMovement.TURN_RIGHT
        
        else:
            movement = RobotMovement.FORWARD

        return movement



