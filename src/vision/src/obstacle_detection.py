import math

import cv2 as cv
import numpy as np
import rospy

from cv_mat import CVMat


class ObstacleDetector(CVMat):
    RECTANGLE_RATIO = 1.618

    def __init__(self, frame):
        CVMat.__init__(self, frame=frame)

    def extract_obstacle(self, thresh_lb, thresh_ub):
        self.extract_object(thresh_lb=thresh_lb, thresh_ub=thresh_ub, field=None, kernel_size=3)

    def bounding_rect(self, min_area, max_area, frame_height, frame_width, flush_width):
        # Find contours
        _, contours, hierarchy = cv.findContours(self.frame, mode=cv.RETR_EXTERNAL, method=cv.CHAIN_APPROX_SIMPLE)

        if contours:
            positions = []
            position_text = ''
            contours = sorted(contours, key=cv.contourArea, reverse=True)

            for contour in contours:
                if min_area <= cv.contourArea(contour) <= max_area:
                    x1, y1, w, h = cv.boundingRect(contour)
                    x2, y2 = x1+w, y1+h
                    mid_x, mid_y = ((x2 - x1) / 2.0) + x1, ((y2 - y1) / 2.0) + y1
                    area = cv.contourArea(contour)

                    # Check for direction between self and obstacle
                    left_bound = (frame_width -  2*flush_width) * (1.0/4.0) + flush_width
                    right_bound = (frame_width - 2*flush_width) * (3.0/4.0) + flush_width

                    if mid_x <= left_bound:
                        position_text += 'left_'
                    elif mid_x > left_bound and mid_x <= right_bound:
                        position_text += 'center_'
                    elif mid_x > right_bound:
                        position_text += 'right_'

                    positions += [((x1, y1), (x2, y2))]
            
            obstacles = (positions, position_text, cv.contourArea(contours[0]))
        else:
            obstacles = (None, 'none', -1)

        return obstacles
