#!/usr/bin/env python

from enum import Enum
import os
from time import sleep, time

import rosnode
import rospy
import roslib.packages as rospkg
from std_msgs.msg import Bool, String

from head_control_module import HeadControlModule
from walking_module import WalkingModule
from obstacle import Obstacle
from robot import Robot, RobotMovement
from slam.srv import ToggleLocalization
from vision.msg import DetectedObstacle, DetectedLines
from slam.msg import RobotPosition


class RobotMode(Enum):
    READY = 0
    RUNNING = 1


def button_callback(msg):
    global robot_mode, mode_change

    if msg.data == 'start':
        # Start button pressed
        # Send robot to RUNNING mode
        if robot_mode == RobotMode.READY:
            robot_mode = RobotMode.RUNNING
            mode_change = True
        elif robot_mode == RobotMode.RUNNING:
            rospy.logerr('Robot already in running mode.')

    elif msg.data == 'mode':
        # Mode button pressed
        # Send robot to READY mode
        robot_mode = RobotMode.READY
        mode_change = True


def wait_for_node(node_name):
    # Wait for node to start
    while node_name not in rosnode.get_node_names():
        rospy.logwarn('Node is not running: {0}'.format(node_name))
        sleep(1)

    rospy.loginfo('Node is running: {0}'.format(node_name))


def memorize_lines(msg):
    global lines, look_at_lines

    if look_at_lines:
        if 'T' in msg.boundary_line and 'L' in msg.goal_line:
            lines += ['TL']
        elif 'L' in msg.goal_line:
            lines += ['gL']
        elif 'center' in msg.center_line:
            lines += ['center']
        else:
            lines += ['']


def split_list(alist, wanted_parts=1):
    length = len(alist)
    return [alist[i*length // wanted_parts: (i+1)*length // wanted_parts] for i in range(wanted_parts)]


def avoid_obstacles():
    global mode_change, walking_module, obstacle, rate, toggle_localization_srv, walking_rates

    previous_movement = None

    while not rospy.is_shutdown() and not mode_change:
        # Get next movement of robot based on obstacle and map
        movement = robot.next_movement(avoid_obstacle=obstacle.is_center, num_map_rows=NUM_MAP_ROWS, 
                                       num_map_columns=NUM_MAP_COLUMNS)

        if movement == RobotMovement.OBSTACLE:
            # Avoid obstacle right in front of robot with localization turned off
            rospy.loginfo('[Navigation] avoid obstacle')

            toggle_localization_srv(pause=True, obstacle=True, start=False, turn_left=False, turn_right=False, place_left=False, place_right=False, place_center=False)
            walking_module.turn_right()
            sleep(robot.turn_right_rate / 4.0)
            walking_module.forward()
            sleep(robot.forward_rate * 8)
            walking_module.turn_left()
            sleep(robot.turn_left_rate / 4.0)
            toggle_localization_srv(pause=False)

        elif movement == RobotMovement.FORWARD:
            # Continue forward
            rospy.loginfo('[Navigation] forward')

            if previous_movement != RobotMovement.FORWARD:
                walking_module.forward()

        elif movement == RobotMovement.TURN_LEFT:
            # Turn left
            rospy.loginfo('[Navigation] turn left')

            toggle_localization_srv(pause=True, turn_left=True, start=False, obstacle=False, turn_right=False, place_left=False, place_right=False, place_center=False)
            walking_module.turn_left()
            sleep(robot.turn_left_rate)
            toggle_localization_srv(pause=False)

        elif movement == RobotMovement.TURN_RIGHT:
            # Turn right
            rospy.loginfo('[Navigation] turn right')

            toggle_localization_srv(pause=True, turn_right=True, start=False, turn_left=False, obstacle=False, place_left=False, place_right=False, place_center=False)
            walking_module.turn_right()
            sleep(robot.turn_right_rate)
            toggle_localization_srv(pause=False)

        previous_movement = movement

        rate.sleep()


if __name__ == '__main__':

    # Initalize global variables
    robot_mode = RobotMode.READY
    mode_change = False
    robot = Robot(rate_config_path=os.path.join(rospkg.get_pkg_dir('navigation'), 'config', 'walking_rate.json'))
    obstacle = Obstacle()
    SPIN_RATE = 0.5
    NUM_MAP_ROWS, NUM_MAP_COLUMNS = 19, 28
    look_at_lines = False
    lines = []

    # Wait for other nodes to launch
    wait_for_node('/op3_manager')
    wait_for_node('/vision_node')
    wait_for_node('/slam_node')

    # Initialze ROS node
    rospy.init_node('navigation_node')
    rate = rospy.Rate(SPIN_RATE)
    
    # Setup ROS publisher
    toggle_localization_srv = rospy.ServiceProxy('/slam/toggle_localization', ToggleLocalization)

    # Initialize OP3 subscriber
    button_sub = rospy.Subscriber('/robotis/open_cr/button', String, button_callback)

    # Initialze vision subscriber
    line_vision_sub = rospy.Subscriber('/vision/lines', DetectedLines, memorize_lines)
    obstacle_vision_sub = rospy.Subscriber('/vision/obstacle', DetectedObstacle, obstacle.update)

    # Initialize SLAM subscriber
    robot_position_sub = rospy.Subscriber('/slam/robot_position', RobotPosition, robot.update_pose)

    # Initializing modules
    head_control_module = HeadControlModule()
    walking_module = WalkingModule()

    # Enabling walking module
    sleep(10)
    rospy.loginfo('Press start button to begin.')

    while not rospy.is_shutdown():
        if mode_change:
            mode_change = False

            if robot_mode == RobotMode.READY:
                rospy.loginfo('Resetting OP3. Press start button to begin.')

                walking_module.enable()
                walking_module.stop()
                head_control_module.enable()
                head_control_module.look_down()

            elif robot_mode == RobotMode.RUNNING:
                rospy.loginfo('Starting navigating soccer field.')

                walking_module.enable()
                head_control_module.enable()

                # Try to find starting cell
                head_control_module.look_left()
                sleep(1.5)
                look_at_lines = True
                head_control_module.look_right()
                sleep(2.5)
                look_at_lines = False

                left_lines, center_lines, right_lines = split_list(lines, wanted_parts=3)
                lines = [max(left_lines, key=left_lines.count),
                         max(center_lines, key=center_lines.count),
                         max(right_lines, key=right_lines.count)]

                if lines[0] == 'TL':
                    toggle_localization_srv(place_left=True, start=False, pause=True, obstacle=False, turn_left=False, turn_right=False, place_center=False, place_right=False)
                elif lines[2] == 'TL':
                    toggle_localization_srv(place_right=True, start=False, pause=True, obstacle=False, turn_left=False, turn_right=False, place_left=False, place_center=False)
                else:
                    toggle_localization_srv(place_center=True, start=False, pause=True, obstacle=False, turn_left=False, turn_right=False, place_left=False, place_right=False)

                walking_module.enable()
                walking_module.march()
                head_control_module.enable()
                head_control_module.look_down()
                sleep(3)

                toggle_localization_srv(start=True)
                avoid_obstacles()

        rate.sleep()
