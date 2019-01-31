class Obstacle(object):

    def __init__(self):
        self.is_on_left = False
        self.is_on_right = False
        self.is_center = False

    def update(self, msg):
        self.is_on_left = msg.is_left
        self.is_center = msg.is_center
        self.is_on_right = msg.is_right