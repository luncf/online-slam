import numpy as np

import rospy
from vision.msg import DetectedLines, DetectedObstacle

class FieldObject(object):

    def __init__(self, configuration, name):
        self.name = name

        self.threshold = np.array(configuration['threshold'])
        self.value = np.array(configuration['value'])
        self.lower_bound, self.upper_bound = self.get_colour_space_bounds()

    def get_colour_space_bounds(self):
        threshold_lower_bound = []
        threshold_upper_bound = []

        for index in range(0, len(self.value)):
            threshold_lower_bound += [self.round_int(self.value[index] - self.threshold[index])]
            threshold_upper_bound += [self.round_int(self.value[index] + self.threshold[index])]

        return np.array(threshold_lower_bound), np.array(threshold_upper_bound)

    def set_colour_space_value(self, values):
        self.value = np.array([self.round_int(value) for value in values])
        self.lower_bound, self.upper_bound = self.get_colour_space_bounds()

    def set_colour_space_threshold(self, threshold):
        self.threshold = np.array([self.round_int(value) if value >= 0 else self.threshold[index]
                                   for index, value in enumerate(threshold)])
        self.lower_bound, self.upper_bound = self.get_colour_space_bounds()

    def export_configuration(self):
        return {
            'threshold': self.threshold, 'value': self.value,
        }

    @staticmethod
    def round_int(num):
        return max(0, int(round(num)))


class Field(FieldObject):

    def __init__(self, configuration, name='field'):
        FieldObject.__init__(self, name=name, configuration=configuration)
        self.min_area = configuration['min_area']

    def export_configuration(self):
        configuration = super(Field, self).export_configuration()
        configuration['min_area'] = self.min_area

        return configuration


class Lines(FieldObject):

    def __init__(self, configuration, name='lines'):
        FieldObject.__init__(self, name=name, configuration=configuration)

        self.pub = rospy.Publisher('/vision/' + name, DetectedLines, queue_size=1)

        self.max_distance_apart = configuration['max_distance_apart']
        self.min_length = configuration['min_length']
        self.max_width = configuration['max_width']
        self.corner_max_distance_apart = configuration['corner_max_distance_apart']
        self.output_center_colour = configuration["output_center_line_colour"]
        self.output_goal_area_colour = configuration["output_goal_area_line_colour"]
        self.output_boundary_colour = configuration["output_boundary_line_colour"]
        self.output_undefined_colour = configuration["output_undefined_line_colour"]

    def export_configuration(self):
        configuration = super(Lines, self).export_configuration()

        configuration['max_distance_apart'] = self.max_distance_apart
        configuration['min_length'] = self.min_length
        configuration['max_width'] = self.max_width
        configuration['corner_max_distance_apart'] = self.corner_max_distance_apart
        configuration["output_center_line_colour"] = self.output_center_colour
        configuration["output_goal_area_line_colour"] = self.output_goal_area_colour
        configuration["output_boundary_line_colour"] = self.output_boundary_colour
        configuration["output_undefined_line_colour"] = self.output_undefined_colour

        return configuration

    def publish_msg(self, lines):
        message = DetectedLines()

        message.boundary_line = '|'.join([line_description[1] for line_description in lines['boundary'][0]])
        message.goal_line = '|'.join([line_description[1] for line_description in lines['goal_area'][0]])
        message.center_line = '|'.join([line_description[1] for line_description in lines['center'][0]])
        message.undefined_lines = '|'.join([line_description[1] for line_description in lines['undefined'][0]])
            
        self.pub.publish(message)


class Obstacle(FieldObject):

    def __init__(self, configuration, name='obstacle'):
        FieldObject.__init__(self, name=name, configuration=configuration)

        self.pub = rospy.Publisher('/vision/' + name, DetectedObstacle, queue_size=1)

        self.min_area = configuration['min_area']
        self.max_area = configuration['max_area']

        self.output_colour = configuration['output_colour']

    def export_configuration(self):
        configuration = super(Obstacle, self).export_configuration()

        configuration['min_area'] = self.min_area
        configuration['max_area'] = self.max_area
        configuration['output_colour'] = self.output_colour

        return configuration

    def publish_msg(self, position, area):
        position = position.lower()

        message = DetectedObstacle()
        message.is_left = 'left' in position
        message.is_center = 'center' in position
        message.is_right = 'right' in position
        message.area = area
        
        self.pub.publish(message)
