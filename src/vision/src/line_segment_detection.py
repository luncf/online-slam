from bisect import insort
from enum import Enum
import math
import sys

import cv2 as cv
import rospy

from cv_mat import CVMat


class LineSegment(object):

    def __init__(self, x1, y1, x2, y2):
        pts = sorted([(x1, y1), (x2, y2)], key=lambda pt: pt[0])
        self.x1 = pts[0][0]
        self.y1 = pts[0][1]
        self.x2 = pts[1][0]
        self.y2 = pts[1][1]
        self.length = self.length(x1=self.x1, y1=self.y1, x2=self.x2, y2=self.y2)
        self.angle = self.angle()
        self.position = None

    def __lt__(self, other_line_segment):
        return self.length < other_line_segment.length

    def distance_to_point(self, pt_x=0, pt_y=0):
        t = -((self.x1 - pt_x) * (self.x2 - self.x1) + (self.y1 - pt_y) * (self.y2 - self.y1)) / \
            (math.pow(self.x2 - self.x1, 2) + math.pow(self.y2 - self.y1, 2))

        if 0 <= t <= 1:
            distance = math.fabs((self.x2 - self.x1) * (self.y1 - pt_y) - (self.y2 - self.y1) * (self.x1 - pt_x)) / \
                math.sqrt(math.pow(self.x2 - self.x1, 2) + math.pow(self.y2 - self.y1, 2))
        else:
            distance = min(math.sqrt(math.pow(self.x1 - pt_x, 2) + math.pow(self.y1 - pt_y, 2)),
                           math.sqrt(math.pow(self.x2 - pt_x, 2) + math.pow(self.y2 - pt_y, 2)))

        return distance

    def to_cv_line(self):
        return (self.to_pixel_value(self.x1), self.to_pixel_value(self.y1)), \
               (self.to_pixel_value(self.x2), self.to_pixel_value(self.y2))

    def angle(self):
        angle = math.atan2((self.y2 - self.y1), (self.x2 - self.x1))

        if self.x2 != self.x1 and (self.y2 - self.y1) / (self.x2 - self.x1) < 0:
            angle = -math.fabs(angle)

        return angle

    @staticmethod
    def distance(x1, y1, x2=0, y2=0):
        return math.sqrt(math.pow(math.fabs(x2 - x1), 2) + math.pow(math.fabs(y2 - y1), 2))

    @staticmethod
    def length(x1, y1, x2, y2):
        return LineSegment.distance(x1=x1, y1=y1, x2=x2, y2=y2)

    @staticmethod
    def to_pixel_value(pt):
        return max(0, int(round(pt)))


class Corner(object):

    CornerType = Enum('CornerType', 'l_shape, t_shape')

    def __init__(self, x, y, line1, line2=None):
        self.x = x
        self.y = y

        if line2 is not None:
            min_line = min(line1, line2, key=lambda line: line.length)
            max_line = line1 if min_line == line2 else line2
        else:
            min_line = line1
            max_line = None

        self.line1 = min_line
        self.line2 = max_line


class FieldLines(object):

    def __init__(self, lines, l_corners=0, t_corners=0):
        self.lines = lines
        self.l_corners = l_corners
        self.t_corners = t_corners

    def add_l_corner(self, line):
        self.lines += [line]
        self.l_corners += 1

    def add_t_corner(self):
        self.t_corners += 1

    def has_line(self, line):
        return line in self.lines


class LineSegmentDetector(CVMat):
    __OPENCV_LSD = cv.createLineSegmentDetector(_refine=cv.LSD_REFINE_STD)

    def __init__(self, frame):
        CVMat.__init__(self, frame=frame)
        self.line_segments = []
        self.horizontal = []
        self.vertical = []
        self.field_lines = []
        self.extra_lines = []

    def extract_lines(self, thresh_lb, thresh_ub, field):
        self.extract_object(thresh_lb=thresh_lb, thresh_ub=thresh_ub, field=field, kernel_size=3)

    @staticmethod
    def __filter_by_length(lines, min_length=-1):
        # Return only lines that are longer than min_length
        filtered_lines = []

        for line in lines:
            if LineSegment.length(x1=line[0, 0], y1=line[0, 1], x2=line[0, 2], y2=line[0, 3]) > min_length:
                insort(filtered_lines, LineSegment(x1=line[0, 0], y1=line[0, 1], x2=line[0, 2], y2=line[0, 3]))
        filtered_lines.reverse()

        return filtered_lines

    @staticmethod
    def __categorize_by_angle(lines):
        horizontal = []
        vertical = []

        if len(lines) > 0:
            # Find the average of the min_angle and max_angle, use that to offset the x-axis
            max_angle = max(line.angle for line in lines)
            min_angle = min(line.angle for line in lines)
            min_max_average = (math.fabs(max_angle) + math.fabs(min_angle)) / 2.0
            x_axis_offset = math.pi / 4 if min_max_average > math.pi / 3 else 0

            for line in lines:
                angle = line.angle if x_axis_offset == 0 else math.fabs(line.angle)
                if angle > x_axis_offset:
                    vertical += [line]
                else:
                    horizontal += [line]

        return horizontal, vertical

    @staticmethod
    def __categorize_by_distance_apart(lines, max_distance_apart, frame_height):
        categorize_distance = []

        for line in lines:
            closest_group_index = None
            shortest_distance_apart = sys.maxint

            # Find a group for the line based on the shortest distance existing grouped lines
            for group_index, group in enumerate(categorize_distance):
                for reference_line in group:
                    distance_apart = min(reference_line.distance_to_point(pt_x=line.x1, pt_y=line.y1),
                                         reference_line.distance_to_point(pt_x=line.x2, pt_y=line.y2),
                                         line.distance_to_point(pt_x=reference_line.x1, pt_y=reference_line.y1),
                                         line.distance_to_point(pt_x=reference_line.x2, pt_y=reference_line.y2))

                    # Calculate a new max_distance_apart based on the average value of y
                    # bottom of image --> larger y --> line segments further apart
                    average_y = (line.y1 + line.y2 + reference_line.y1 + reference_line.y2) / 4.0
                    distance_apart_threshold = 1 + average_y / frame_height / 2

                    if distance_apart < shortest_distance_apart and \
                            distance_apart < max_distance_apart * distance_apart_threshold:
                        closest_group_index = group_index
                        shortest_distance_apart = distance_apart

            # Group two lines together if a grouping was found, otherwise, start a new group
            if closest_group_index is not None:
                categorize_distance[closest_group_index] += [line]
            else:
                categorize_distance += [[line]]

        return categorize_distance

    @staticmethod
    def __merge_grouped_lines__(groups):
        merged_lines = []

        # Find the average of the lines in the group
        # Create a new line based on the average
        for group in groups:
            if len(group) > 1:
                x_coordinates = []
                y_coordinates = []
                average_angle = 0

                for line in group:
                    x_coordinates += [line.x1, line.x2]
                    y_coordinates += [line.y1, line.y2]
                    average_angle += line.angle

                min_x = min(x_coordinates)
                max_x = max(x_coordinates)
                min_y = min(y_coordinates)
                max_y = max(y_coordinates)
                angle = average_angle / len(group)

                if angle > 0:
                    merged_lines += [LineSegment(x1=min_x, y1=min_y, x2=max_x, y2=max_y)]
                else:
                    merged_lines += [LineSegment(x1=min_x, y1=max_y, x2=max_x, y2=min_y)]

        return merged_lines

    def lsd(self, max_distance_apart=10, min_length=15):
        # Find all line segment with OpenCV's LSD
        lines = self.__OPENCV_LSD.detect(self.frame)[0]

        if lines is not None:
            # Filter out lines that do not meet min_length condition
            self.line_segments = self.__filter_by_length(lines, min_length=min_length)

            # Classify the lines by angle
            horizontal, vertical = self.__categorize_by_angle(self.line_segments)

            # Classify the lines by distance apart
            horizontal = self.__categorize_by_distance_apart(horizontal, max_distance_apart, self.frame.shape[0])
            vertical = self.__categorize_by_distance_apart(vertical, max_distance_apart, self.frame.shape[0])

            # Find average line in classified groups
            horizontal = self.__merge_grouped_lines__(horizontal)
            vertical = self.__merge_grouped_lines__(vertical)

            self.horizontal = horizontal
            self.vertical = vertical

    @staticmethod
    def classify_corner_shape(line, point, max_distance_apart):
        # Determine whether corner is L shaped or T shaped
        distance_apart = min(LineSegment.distance(x1=line.x1, y1=line.y1, x2=point[0], y2=point[1]),
                             LineSegment.distance(x1=line.x2, y1=line.y2, x2=point[0], y2=point[1]))

        return Corner.CornerType.l_shape if distance_apart <= max_distance_apart else Corner.CornerType.t_shape

    @staticmethod
    def __find_corners(lines, reference_lines, max_distance_apart):
        l_shape_corners = []
        t_shape_corners = []

        remaining_reference_lines = list(reference_lines)

        for reference_line in reference_lines:
            for line in lines:
                distance_pt1 = reference_line.distance_to_point(pt_x=line.x1, pt_y=line.y1)
                distance_pt2 = reference_line.distance_to_point(pt_x=line.x2, pt_y=line.y2)
                distance_apart = min(distance_pt1, distance_pt2)

                # Find the shortest distance between the reference line and two endpoints of line
                # Classify as corner if distance_apart is shorter than max_distance_apart
                if distance_apart < max_distance_apart:
                    (pt_x, pt_y) = (line.x1, line.y1) if distance_apart == distance_pt1 else (line.x2, line.y2)
                    corner_type = LineSegmentDetector.classify_corner_shape(line=reference_line, point=(pt_x, pt_y),
                                                                            max_distance_apart=max_distance_apart)
                    if corner_type == Corner.CornerType.l_shape:
                        l_shape_corners += [Corner(x=pt_x, y=pt_y, line1=line, line2=reference_line)]
                        if reference_line in remaining_reference_lines:
                            remaining_reference_lines.remove(reference_line)
                    else:
                        t_shape_corners += [Corner(x=pt_x, y=pt_y, line1=reference_line)]
                        if reference_line in remaining_reference_lines:
                            remaining_reference_lines.remove(reference_line)

        return l_shape_corners, t_shape_corners, remaining_reference_lines

    def find_corners(self, max_distance_apart=5):
        self.field_lines = []

        # Find all possible T and L shape corners
        horizontal_l_shape, horizontal_t_shape, extra_vertical = self.__find_corners(lines=self.horizontal,
                                                                                     reference_lines=self.vertical,
                                                                                     max_distance_apart=max_distance_apart)
        vertical_l_shape, vertical_t_shape, extra_horizontal = self.__find_corners(lines=self.vertical,
                                                                                   reference_lines=self.horizontal,
                                                                                   max_distance_apart=max_distance_apart)

        # Merge L shape corners
        l_shape_corners = []
        for horizontal in horizontal_l_shape:
            for vertical in vertical_l_shape:
                distance_apart = LineSegment.distance(x1=horizontal.x, y1=horizontal.y, x2=vertical.x, y2=vertical.y)
                if distance_apart < max_distance_apart:
                    found_matching_line = False

                    for existing_l_shape in l_shape_corners:
                        if existing_l_shape.has_line(horizontal.line1):
                            existing_l_shape.add_l_corner(horizontal.line2)
                            found_matching_line= True
                        elif existing_l_shape.has_line(horizontal.line2):
                            existing_l_shape.add_l_corner(horizontal.line1)
                            found_matching_line = True

                    if not found_matching_line:
                        l_shape_corners += [FieldLines(lines=[horizontal.line1, horizontal.line2], l_corners=1)]

        # Merge T shape corners
        t_shape_corners = []
        for corner in horizontal_t_shape + vertical_t_shape:
            found_matching_line = False

            for existing_t_shape in t_shape_corners:
                if existing_t_shape.has_line(corner.line1):
                    existing_t_shape.add_t_corner()
                    found_matching_line = True

            if not found_matching_line:
                t_shape_corners += [FieldLines(lines=[corner.line1], t_corners=1)]

        # Merge T shape corners with L shape corners into lines
        for l_shape in l_shape_corners:
            for t_shape in list(t_shape_corners):
                if l_shape.has_line(t_shape.lines[0]):
                    l_shape.add_t_corner()
                    t_shape_corners.remove(t_shape)

        self.field_lines = l_shape_corners + t_shape_corners
        self.extra_lines = extra_horizontal + extra_vertical

    def classify_lines(self):
        boundary_line = []
        goal_area_line = []
        center_line = []
        undefined_lines = []

        all_lines = [line for line_set in self.field_lines for line in line_set.lines] + self.extra_lines

       # Center line
        if len(all_lines) >= 5 or len(self.extra_lines) >= 3:
            center_line = [([max(all_lines, key=lambda line: line.length)], 'center')]
            self.field_lines = []
            self.extra_lines = []

        # Lines with T corners must be boundary
        if len(self.field_lines) > 0:
            self.field_lines = sorted(self.field_lines, key=lambda line: line.t_corners, reverse=True)
            if 0 < self.field_lines[0].t_corners <= 2:
                boundary_line += [(self.field_lines.pop(0).lines, 'T')]

        # Lines with L corners can be boundary or goal area
        if len(self.field_lines) > 0:
            self.field_lines = sorted(self.field_lines, key=lambda line: line.l_corners, reverse=True)
            l_corner_lines = [line for line in self.field_lines if line.l_corners > 0]

            if len(l_corner_lines) > 0:
                if len(boundary_line) > 0:
                    # This L corner must be a goal area since we already have a boundary
                    goal_area_line += [(self.field_lines.pop(0).lines, 'L')]
                elif len(l_corner_lines) == 2:
                    # The top corner point must be the boundary
                    # The bottom corner point must be the goal area
                    line0_y = []
                    for line in l_corner_lines[0].lines:
                        line0_y += [line.y1, line.y2]
                    line0_y = sorted(line0_y)
                    line1_y = []
                    for line in l_corner_lines[1].lines:
                        line1_y += [line.y1, line.y2]
                    line1_y = sorted(line1_y)

                    if line0_y[0] < line1_y[0]:
                        boundary_line += [(self.field_lines.pop(0).lines, 'L')]
                        goal_area_line += [(self.field_lines.pop(0).lines, 'L')]
                    else:
                        goal_area_line += [(self.field_lines.pop(0).lines, 'L')]
                        boundary_line += [(self.field_lines.pop(0).lines, 'L')]

        if len(self.field_lines) > 0:
            # Lines that are left over are unknown
            for line in self.field_lines:
                undefined_lines += line.lines
        undefined_lines += self.extra_lines

        # 2 Parellel lines = goal area
        if len(undefined_lines) == 2 and len(boundary_line) == 0 and len(goal_area_line) == 0 and \
                -0.5 <= math.fabs(undefined_lines[0].angle) - math.fabs(undefined_lines[1].angle) <= 0.5 and \
                ((undefined_lines[0].angle >= 0 and undefined_lines[1].angle >= 0) or (undefined_lines[0].angle < 0 and undefined_lines[1].angle < 0)):
            line_0_distance = undefined_lines[0].distance_to_point()
            line_1_distance = undefined_lines[1].distance_to_point()

            if undefined_lines[0].angle >= 0.3 or undefined_lines[1].angle >= 0.3:
                if line_0_distance < line_1_distance:
                    boundary_line += [([undefined_lines[1]], 'parallel')]
                    goal_area_line += [([undefined_lines[0]], 'parallel')]
                else:
                    boundary_line += [([undefined_lines[0]], 'parallel')]
                    goal_area_line += [([undefined_lines[1]], 'parallel')]
            else:
                if line_0_distance >= line_1_distance:
                    boundary_line += [([undefined_lines[1]], 'parallel')]
                    goal_area_line += [([undefined_lines[0]], 'parallel')]
                else:
                    boundary_line += [([undefined_lines[0]], 'parallel')]
                    goal_area_line += [([undefined_lines[1]], 'parallel')]

            undefined_lines = []

        if len(undefined_lines) > 0:
            undefined_lines = [(undefined_lines, 'unknown')]

        return {
            'boundary': (boundary_line, self.get_position(boundary_line)),
            'goal_area': (goal_area_line, self.get_position(goal_area_line)),
            'center': (center_line, self.get_position(center_line)),
            'undefined': (undefined_lines, self.get_position(undefined_lines))
        }

    def get_position(self, lines):
        position_text = ''

        # if len(lines) > 0:
        #     avg_x = sum(line.x1 + line.x2 for line in lines) / (len(lines) * 2)

        #     # Check for direction between self and obstacle
        #     _, frame_width = self.frame.shape
        #     left_bound = frame_width * (1.0/4.0)
        #     right_bound = frame_width * (3.0/4.0)

        #     if avg_x <= left_bound:
        #         position_text = 'left'
        #     elif avg_x > left_bound and avg_x <= right_bound:
        #         position_text = 'center'
        #     elif avg_x > right_bound:
        #         position_text += 'right'

        return position_text

