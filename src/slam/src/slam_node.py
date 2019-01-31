#!/usr/bin/env python

import json
import math
import os
from time import sleep, time

import cv2 as cv
import roslib.packages as rospkg
import rosnode
import rospy
import numpy as np

from map_components import Robot, Obstacles, Particle
from particles import Particles
from probability_map import CENTER_LINE_FEATURE_PROBABILITY_MAP, BOUNDARY_T_FEATURE_PROBABILITY_MAP, \
                            BOUNDARY_L_FEATURE_PROBABILITY_MAP, GOAL_L_FEATURE_PROBABILITY_MAP, \
                            BOUNDARY_L_GOAL_L_FEATURE_PROBABILITY_MAP, PARALLEL_FEATURE_PROBABILITY_MAP, \
                            BOUNDARY_T_GOAL_L_FEATURE_PROBABILITY_MAP
from vision.msg import DetectedObstacle, DetectedLines
from slam.srv import ToggleLocalization
from slam.msg import RobotPosition


def set_init_robot_position(event, x, y, flags, param):
    global robot

    if event == cv.EVENT_LBUTTONDOWN:
        robot.set_init_position(x=x, y=y)


def toggle_localization(request):
    global pause_localization, start_localization, movement_avoid_obstacle, movement_turn_left, movement_turn_right

    pause_localization = request.pause
    start_localization = request.start

    if request.place_left:
        robot.row = 18
        robot.column = 8
    elif request.place_center:
        robot.row = 18
        robot.column = 13
    elif request.place_right:
        robot.row = 18
        robot.column = 19

    movement_avoid_obstacle = request.obstacle
    movement_turn_left = request.turn_left
    movement_turn_right = request.turn_right

    return True


def update_probability_map(msg):
    global robot, probability_map

    map_idx = str(int(round(math.degrees(int(robot.angle) * math.pi/2))))

    if 'T' in msg.boundary_line and 'L' in msg.goal_line:
        probability_map = BOUNDARY_T_GOAL_L_FEATURE_PROBABILITY_MAP[map_idx]
    elif 'L' in msg.boundary_line and 'L' in msg.goal_line:
        probability_map = BOUNDARY_L_GOAL_L_FEATURE_PROBABILITY_MAP[map_idx]
    elif 'parallel' in msg.boundary_line or 'parallel' in msg.goal_line:
        probability_map = PARALLEL_FEATURE_PROBABILITY_MAP[map_idx]
    elif 'T' in msg.boundary_line:
        probability_map = BOUNDARY_T_FEATURE_PROBABILITY_MAP[map_idx]
    elif 'L' in msg.boundary_line:
        probability_map = BOUNDARY_L_FEATURE_PROBABILITY_MAP[map_idx]
    elif 'L' in msg.goal_line:
        probability_map = GOAL_L_FEATURE_PROBABILITY_MAP[map_idx]
    elif 'center' in msg.center_line:
        probability_map = CENTER_LINE_FEATURE_PROBABILITY_MAP[map_idx]
    else:
        probability_map = None


def add_obstacle_to_map(msg):
    global obstacles, map_obstacle, map_obstacle_counter

    if map_obstacle:
        # Erase obstacles in front of robot
        # obstacles.update_obstacle_list(row=robot.row, column=robot.column, angle=robot.angle)

        # Add obstacle based on vision
        if (msg.is_left or msg.is_center or msg.is_right) and map_obstacle_counter == 0:
            obstacles.add_obstacle(row=robot.row, column=robot.column, angle=robot.angle,
                                   is_left=msg.is_left, is_center=msg.is_center, is_right=msg.is_right)
            map_obstacle_counter = 5


def draw_grid(field, grid_size, colour):
    field_height, field_width, _ = field.shape

    for x in range(field_width / GRID_SIZE):
        cv.line(field, pt1=(GRID_SIZE * (x + 1), 0),
                pt2=(GRID_SIZE * (x + 1), field_height), color=colour)
    for y in range(field_height / GRID_SIZE):
        cv.line(field, pt1=(0, GRID_SIZE * (y + 1)),
                pt2=(field_width, GRID_SIZE * (y + 1)), color=colour)

    return field


def draw_field(field, robot=None, particles=None, obstacles=None):
    if particles is not None:
        for particle in particles.particles:
            field = Particle(row=particle.row, column=particle.column, angle=particle.angle).draw(field)
    
    if obstacles is not None:
        field = obstacles.draw(field)

    if robot is not None:
        field = robot.draw(field)

    return field


def localize():
    global robot, particles, t0, WALK_FORWARD_RATE, robot_position_pub
    cells_per_second = 1 / (WALK_FORWARD_RATE / 0.65)
    estimated_num_cells = 0.0

    # Wait until robot walks at least 1 cell
    while estimated_num_cells < 1 and not rospy.is_shutdown():
        time_delta = time() - t0
        t0 = time()
        estimated_num_cells += time_delta * cells_per_second
    estimated_num_cells = int(round(estimated_num_cells))

    # Randomly generate particles and move based on probability
    best_particle = particles.random_sample(row=robot.row, column=robot.column,
                                            estimated_num_cells=estimated_num_cells,
                                            probability_map=probability_map)
    
    # Choose best particle and move robot accordingly
    rospy.loginfo('[SLAM] particle: prob - {0:0.2f} particles - {1:2d}' \
                  .format(best_particle.probability, len(particles.particles)))
    robot.update_position(row=best_particle.row, column=best_particle.column, angle=best_particle.angle)

    # Update navigation with robot pose
    robot_position_pub.publish(row=robot.row, column=robot.column, angle=robot.angle)


if __name__ == '__main__':
    # Load configurations
    with open(os.path.join(rospkg.get_pkg_dir('navigation'), 'config', 'walking_rate.json')) as config:
        walking_rates = json.load(config)
    WALK_FORWARD_RATE = walking_rates['forward']
    TURN_LEFT_RATE = walking_rates['turn_left']
    TURN_RIGHT_RATE = walking_rates['turn_right']

    # Declare constants
    GRID_SIZE = 30
    GRID_COLOUR = (128, 128, 128)
    NUM_MAP_ROWS, NUM_MAP_COLUMNS = 19, 28

    # Initalize global variables
    robot = Robot(angle=1)
    obstacles = Obstacles(num_map_rows=NUM_MAP_ROWS, num_map_columns=NUM_MAP_COLUMNS)
    particles = None
    probability_map = None

    # Localization control variables
    start_localization = False
    pause_localization = True
    started_once = False
    movement_avoid_obstacle = False
    movement_turn_left = False
    movement_turn_right = False
    map_obstacle = True
    map_obstacle_counter = 0

    # Initialze ROS node
    SPIN_RATE = 10
    rospy.init_node('slam_node')
    rate = rospy.Rate(SPIN_RATE)

    # SLAM services and publishers
    toggle_localization_srv = rospy.Service('/slam/toggle_localization', ToggleLocalization, toggle_localization)
    robot_position_pub = rospy.Publisher('/slam/robot_position', RobotPosition, queue_size=1)

    # Vision subscribers
    vision_line_sub = rospy.Subscriber('/vision/lines', DetectedLines, update_probability_map)
    vision_obstacle_sub = rospy.Subscriber('/vision/obstacle', DetectedObstacle, add_obstacle_to_map)

    # Load map
    FIELD = cv.imread(os.path.join(rospkg.get_pkg_dir('slam'), 'static', 'soccer_field.png'), cv.IMREAD_UNCHANGED)
    FIELD = draw_grid(FIELD, grid_size=GRID_SIZE, colour=GRID_COLOUR)

    # Create map window and callbacks
    cv.namedWindow('map')
    cv.setMouseCallback('map', on_mouse=set_init_robot_position)
    cv.imshow('map', FIELD)

    while not rospy.is_shutdown():

        if map_obstacle_counter > 0:
            map_obstacle_counter -= 1

        cv.imshow('map', draw_field(field=FIELD.copy(), robot=robot, particles=particles, obstacles=obstacles))
        user_input = cv.waitKey(1)

        if start_localization:
            cv.setMouseCallback('map', on_mouse=lambda event, x, y, flags, param: None)

            # Create particles at the current robot pose
            particles = Particles(row=robot.row, column=robot.column, angle=robot.angle,
                                  map_rows=NUM_MAP_ROWS, map_columns=NUM_MAP_COLUMNS)

            rospy.loginfo('[SLAM] start localization')
            t0 = time()
            start_localization = False
            pause_localization = False
            started_once = True

        elif not pause_localization:
            localize()

        elif pause_localization and started_once:
            # Pause localization and update with "actions"

            if movement_avoid_obstacle:
                # Turn right and navigate around obstacle
                map_obstacle = False
                movement_avoid_obstacle = False

                cv.imshow('map', draw_field(field=robot.draw_turn_right45(FIELD.copy()), obstacles=obstacles))
                cv.waitKey(1)
                robot_position_pub.publish(row=robot.row, column=robot.column, angle=robot.angle)
                sleep(TURN_RIGHT_RATE / 2.0)

                robot.move_forward(steps=1, right_offset=1)
                cv.imshow('map', draw_field(field=FIELD.copy(), robot=robot, obstacles=obstacles))
                cv.waitKey(1)
                robot_position_pub.publish(row=robot.row, column=robot.column, angle=robot.angle)
                sleep(WALK_FORWARD_RATE + TURN_LEFT_RATE / 2.0)

                num_steps = 2
                for _ in range(num_steps):
                    robot.move_forward(steps=1)
                    cv.imshow('map', draw_field(field=FIELD.copy(), robot=robot, obstacles=obstacles))
                    cv.waitKey(1)
                    robot_position_pub.publish(row=robot.row, column=robot.column, angle=robot.angle)
                    sleep(WALK_FORWARD_RATE / float(num_steps))

                map_obstacle = True

            elif movement_turn_left:
                # Turn left
                movement_turn_left = False

                robot.turn_left()
                cv.imshow('map', draw_field(field=FIELD.copy(), robot=robot, particles=None, obstacles=obstacles))
                cv.waitKey(1)
                robot_position_pub.publish(row=robot.row, column=robot.column, angle=robot.angle)
                sleep(TURN_LEFT_RATE)

            elif movement_turn_right:
                # Turn right
                movement_turn_right = False

                robot.turn_right()
                cv.imshow('map', draw_field(field=FIELD.copy(), robot=robot, particles=None, obstacles=obstacles))
                cv.waitKey(1)
                robot_position_pub.publish(row=robot.row, column=robot.column, angle=robot.angle)
                sleep(TURN_RIGHT_RATE)

            # RESET T0!!!!!
            if particles is not None:
                particles.randomize(row=robot.row, column=robot.column, angle=robot.angle)
                t0 = time()

        else:
            # Modify robot angle for initial positioning
            robot.set_init_position(turn_left=(user_input == 13))
            # robot.set_init_position(turn_left=(user_input == 91), turn_right=(user_input == 93))

        rate.sleep()

