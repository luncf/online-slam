import math

import cv2 as cv
import numpy as np
import rospy


class MapComponent(object):
    MARKER_SIZE = 30

    def __init__(self, row=-1, column=-1):
        self.row = row
        self.column = column

class TriangleMapComponent(MapComponent):

    # OpenCV mat 90 degrees = Cartesian 270 degrees
    ANGLES = [0, 3*math.pi/2, math.pi, math.pi/2]

    def __init__(self, row, column, angle, colour):
        super(TriangleMapComponent, self).__init__(row=row, column=column)
        self.angle = angle

        self.marker = np.array([(10, 0), (-7, -6), (-7, 6)], np.int32)
        self.colour = colour


    def draw(self, img, row_offset=0, column_offset=0, angle_offset=0):
        row = self.row + row_offset
        column = self.column + column_offset
        angle = self.ANGLES[self.angle] + angle_offset

        # Compute rotation matrix
        cos_angle, sin_angle = np.cos(angle), np.sin(angle)
        rotation_matrix = np.array(((cos_angle, sin_angle), (-sin_angle, cos_angle)))

        # Dot matrix to transform (rotate) robot
        pts = np.dot(self.marker, rotation_matrix).astype(np.int32).reshape((-1, 1, 2))
        pts = np.add(pts, np.array([(column * self.MARKER_SIZE) + self.MARKER_SIZE/2,
                                    (row * self.MARKER_SIZE) + self.MARKER_SIZE/2]).reshape(-1, 2))

        return cv.fillPoly(img, pts=[pts], color=self.colour)


class Particle(TriangleMapComponent):
    def __init__(self, row, column, angle):
        super(Particle, self).__init__(row=row, column=column, angle=angle, colour=(0, 0, 0))


class Robot(TriangleMapComponent):

    def __init__(self, angle=0):
        super(Robot, self).__init__(row=-1, column=-1, angle=angle, colour=(0, 0, 255))

    def set_init_position(self, x=-1, y=-1, turn_left=False, turn_right=False):
        if x != -1 and y != -1:
            self.row = y / self.MARKER_SIZE
            self.column = x / self.MARKER_SIZE
        elif turn_left:
            self.angle = (self.angle + 1) % len(self.ANGLES)
        elif turn_right:
            self.angle = (self.angle - 1) % len(self.ANGLES)

    def update_position(self, row, column, angle):
        # Move robot based on probability of best particle
        self.row = row
        self.column = column
        self.angle = angle

    def move_forward(self, steps, right_offset=0):
        if self.angle == 0:
            self.row += right_offset
            self.column += steps
        elif self.angle == 1:
            self.row -= steps
            self.column += right_offset
        elif self.angle == 2:
            self.row -= right_offset
            self.column -= steps
        elif self.angle == 3:
            self.row += steps
            self.column -= right_offset

    def turn_left(self):
        self.angle = (self.angle + 1) % 4

    def turn_right(self):
        self.angle = (self.angle - 1) % 4

    def draw_turn_left45(self, img):
        return self.draw(img=img, angle_offset=-math.pi/4)

    def draw_turn_right45(self, img):
        return self.draw(img=img, angle_offset=math.pi/4)

    # def draw_left_offset(self, img):
    #     robot_map = None

    #     if self.angle == 0:
    #         robot_map = self.draw(img=img, row_offset=-1)
    #     elif self.angle == 1:
    #         robot_map = self.draw(img=img, column_offset=-1)
    #     elif self.angle == 2:
    #         robot_map = self.draw(img=img, row_offset=1)
    #     elif self.angle == 3:
    #         robot_map = self.draw(img=img, column_offset=1)

    #     return robot_map

    # def draw_right_offset(self, img):
    #     robot_map = None

    #     if self.angle == 0:
    #         robot_map = self.draw(img=img, row_offset=1)
    #     elif self.angle == 1:
    #         robot_map = self.draw(img=img, column_offset=1)
    #     elif self.angle == 2:
    #         robot_map = self.draw(img=img, row_offset=-1)
    #     elif self.angle == 3:
    #         robot_map = self.draw(img=img, column_offset=-1)

    #     return robot_map


class Obstacles(object):

    def __init__(self, num_map_rows, num_map_columns):
        self.obstacles = []
        self.colour = (255, 0, 255)
        self.num_map_rows = num_map_rows
        self.num_map_columns = num_map_columns

    def add_obstacle(self, row, column, angle, offset=1, grid_size=3, is_left=False, is_center=False, is_right=False):
        obstacle_row, obstacle_column = self.robot_pose_to_grid_origin(row=row, column=column, angle=angle,
                                                                       offset=offset, grid_size=grid_size)

        if is_left:
            if angle == 0:
                obstacle_row -= 1
            elif angle == 1:
                obstacle_column -= 1
            elif angle == 2:
                obstacle_row += 1
            elif angle == 3:
                obstacle_column += 1
        elif is_right:
            if angle == 0:
                obstacle_row += 1
            elif angle == 1:
                obstacle_column += 1
            elif angle == 2:
                obstacle_row -= 1
            elif angle == 3:
                obstacle_column -= 1

        if 0 <= obstacle_row < self.num_map_rows and 0 <= obstacle_column < self.num_map_columns:
            self.obstacles += [MapComponent(row=obstacle_row, column=obstacle_column)]

    def update_obstacle_list(self, row, column, angle, offset=2, grid_size=3):
        grid_row, grid_column = self.robot_pose_to_grid_origin(row=row, column=column, angle=angle,
                                                               offset=offset, grid_size=grid_size)
        grid = [(grid_row + row_idx, grid_column + column_idx) for row_idx in range(-1, 2) for column_idx in range(-1, 2)]
                
        for obstacle in list(self.obstacles):
            if (obstacle.row, obstacle.column) in grid:
                self.obstacles.remove(obstacle)

    def robot_pose_to_grid_origin(self, row, column, angle, offset, grid_size):
        if grid_size % 2 == 0:
            rospy.logerr('Obstacle mapping: grid_size must be an odd number.')

        offset = offset + grid_size/2 + 1
        if angle == 0:
            column += offset
        elif angle == 1:
            row -= offset
        elif angle == 2:
            column -= offset
        elif angle == 3:
            row += offset

        return row, column

    def draw(self, img):
        for obstacle in self.obstacles:
            img = cv.circle(img=img, radius=10, color=self.colour, thickness=-1,
                            center=((obstacle.column * obstacle.MARKER_SIZE) + obstacle.MARKER_SIZE/2,
                                    (obstacle.row * obstacle.MARKER_SIZE) + obstacle.MARKER_SIZE/2))

        return img
