#!/usr/bin/env python

from collections import OrderedDict
import os

import cv2 as cv
import rospy
import roslib.packages as rospkg
import numpy as np

from cv_mat import CVMat
from field_object import Field, Lines, Obstacle
from line_segment_detection import LineSegmentDetector
from obstacle_detection import ObstacleDetector
import opencv_gui as gui
from configuration import Configuration


def get_region_colour_space_values_cb(event, x, y, _frame):
    global colour_space_roi, configuration, selected_object

    # Retrieve average values for colour space
    if event == cv.EVENT_LBUTTONDOWN:
        colour_space_roi = (x, y)
    elif event == cv.EVENT_LBUTTONUP:
        roi_mean = cv.mean(_frame[colour_space_roi[1]:y, colour_space_roi[0]:x])
        selected_object.set_colour_space_value(roi_mean[:-1])
        configuration.update(selected_object.name, selected_object.export_configuration())
        colour_space_roi = None


def set_colour_space_threshold_cb(threshold):
    global selected_object, configuration

    # Change colour space threshold for selected object
    selected_object.set_colour_space_threshold(threshold=threshold)
    configuration.update(selected_object.name, selected_object.export_configuration())


def switch_selected_obstacle_cb(value):
    global selected_object, field_object_list

    # Switch the tuning object
    selected_object = field_object_list.values()[value]
    colour_space_threshold = selected_object.threshold
    gui.set_colour_space_threshold_trackbar_position(colour_space_threshold)


def track_lines(lines, lines_mat, field_frame=None):
    global field_object_list, output, selected_object

    lines_mat.extract_lines(thresh_lb=lines.lower_bound, thresh_ub=lines.upper_bound, field=field_frame)

    # Find the lines and corners, classify each line
    lines_mat.lsd(max_distance_apart=lines.max_distance_apart, min_length=lines.min_length)
    lines_mat.find_corners(max_distance_apart=lines.corner_max_distance_apart)
    field_lines = lines_mat.classify_lines()

    # Draw lines if found
    if field_lines:
        for line_type in field_lines.keys():
            if line_type == 'boundary':
                colour = lines.output_boundary_colour
            elif line_type == 'goal_area':
                colour = lines.output_goal_area_colour
            elif line_type == 'center':
                colour = lines.output_center_colour
            else:
                colour = lines.output_undefined_colour

            for line_set in field_lines[line_type][0]:
                for line in line_set[0]:
                    pt1, pt2 = line.to_cv_line()
                    cv.line(output.frame, pt1=pt1, pt2=pt2, color=colour, thickness=3)

    # Publish lines found
    lines.publish_msg(field_lines if field_lines else [])

    # Check if the obstacle is selected, show in debug if selected
    if selected_object == field_object_list['lines']:
        gui.show_debug_window(lines_mat.frame, lines.name)


def track_obstacle(obstacle, obstacle_mat):
    global field_object_list, output, selected_object

    # Extract obstacle from frame
    obstacle_mat.extract_obstacle(thresh_lb=obstacle.lower_bound, thresh_ub=obstacle.upper_bound)

    # Find the obstacle using bounding rectangle, draw any if found
    positions, position_text, area = obstacle_mat.bounding_rect(min_area=obstacle.min_area, max_area=obstacle.max_area,
                                                                frame_height=FRAME_HEIGHT, frame_width=FRAME_WIDTH,
                                                                flush_width=flush_width)

    # Draw rectangle and publish ROS message
    if positions is not None:
        for tl_pt, br_pt in positions:
            cv.rectangle(output.frame, tl_pt, br_pt, obstacle.output_colour, thickness=2)
    obstacle.publish_msg(position_text, area)

    # Check if the obstacle is selected, show in debug if selected
    if selected_object == field_object_list[obstacle.name]:
        gui.show_debug_window(obstacle_mat.frame, obstacle.name)


if __name__ == '__main__':
    # Load configurations
    configuration = Configuration(configuration_directory=os.path.join(rospkg.get_pkg_dir('vision'), 'config'))
    configuration.load()
    colour_space_roi = None

    # Initialize ROS
    rospy.init_node('vision_node')
    rate = rospy.Rate(10)

    # Initialize obstacles/lines and load configurations
    field = Field(configuration=configuration.config['field'])
    lines = Lines(configuration=configuration.config['lines'])
    obstacle = Obstacle(configuration=configuration.config['obstacle'])
    field_object_list = OrderedDict([('field', field), ('lines', lines), ('obstacle', obstacle)])
    selected_object = field

    # Create GUI and set callbacks
    gui.set_mouse_cb('camera', lambda event, x, y, flags, params:
                     get_region_colour_space_values_cb(event, x, y, original.colour_space_frame))
    gui.create_trackbar(gui.DEBUG_WINDOW_NAME, trackbar_name='Obstacles', default_value=0,
                        max_value=len(field_object_list.values()) - 1, callback=switch_selected_obstacle_cb)
    gui.create_colour_space_threshold_trackbar(selected_object.threshold, set_colour_space_threshold_cb)

    # Open camera device
    cap = cv.VideoCapture(configuration.config['camera_index'])
    original_height, original_width = cap.read()[1].shape[:2]
    FRAME_HEIGHT = configuration.config['resized_frame_height']
    FRAME_WIDTH = int(round(FRAME_HEIGHT * (original_width / float(original_height))))
    del original_height, original_width

    # Create flush mask
    flush_width = int(round(FRAME_WIDTH * configuration.config['flush_ratio'] / 2))
    flush_mask = np.ones((FRAME_HEIGHT, FRAME_WIDTH, 1), dtype=np.uint8) * 255
    flush_mask = cv.rectangle(flush_mask, pt1=(0, 0), pt2=(flush_width, FRAME_HEIGHT), color=(0,0,0), thickness=-1)
    flush_mask = cv.rectangle(flush_mask, pt1=(FRAME_WIDTH - flush_width, 0), pt2=(FRAME_WIDTH, FRAME_HEIGHT), color=(0,0,0), thickness=-1)

    try:
        while not rospy.is_shutdown():
            ret_code, raw_image = cap.read()

            if raw_image.data:
                original = CVMat(raw_image, height=FRAME_HEIGHT)

                # Create an output frame from a clone of the original
                output = original.clone()

                # Extract background from soccer field
                field_mat = original.clone()
                field_mat.background_mask(thresh_lb=field.lower_bound, thresh_ub=field.upper_bound,
                                          min_area=field.min_area, line_width=lines.max_width)
                if selected_object == field_object_list['field']:
                    gui.show_debug_window(field_mat.frame, field.name)
                # output.frame = cv.bitwise_and(output.frame, output.frame, mask=field_mat.frame)

                # Extract lines from soccer field and detect lines using LSD
                lines_mat = LineSegmentDetector(frame=original.frame)
                track_lines(lines=lines, lines_mat=lines_mat, field_frame=field_mat.frame)

                # Extract obstacles from field
                # flush_frame = cv.bitwise_and(original.frame, original.frame, mask=flush_mask)
                obstacle_mat = ObstacleDetector(frame=original.frame)
                track_obstacle(obstacle=obstacle, obstacle_mat=obstacle_mat)

                # Display GUI windows
                gui.show('camera', original.frame)
                gui.show('output', output.frame)

            cv.waitKey(1)
    except rospy.ROSInterruptException:
        # Clean up
        gui.teardown()
        cap.release()
