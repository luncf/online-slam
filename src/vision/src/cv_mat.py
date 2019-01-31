import copy

import cv2 as cv
import numpy as np


class CVMat(object):

    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)

    def __init__(self, frame, width=0, height=0):
        self.frame = self.resize(frame, width=width, height=height) if width > 0 or height > 0 else frame
        # self.frame = self.add_border()
        self.colour_space_frame = cv.cvtColor(self.frame, code=cv.COLOR_BGR2LAB)

    def clone(self):
        return copy.deepcopy(self)

    @staticmethod
    def resize(frame, width=0, height=0):
        # Resize an image based on the given width OR height
        # Will choose width if both width and height are given

        (_height, _width) = frame.shape[:2]
        ratio = _width / float(_height)

        if width > 0:
            height = int(round(width / ratio))
        elif height > 0:
            width = int(round(ratio * height))
        else:
            width, height = _width, _height
        
        return cv.resize(frame, (width, height), interpolation=cv.INTER_AREA)

    def add_border(self):
        # Add a black border around frame
        border_size = 10
        return cv.copyMakeBorder(self.frame, borderType=cv.BORDER_CONSTANT, value=[0, 0, 0],
            top=border_size, bottom=border_size, left=border_size, right=border_size)

    def background_mask(self, thresh_lb, thresh_ub, min_area, line_width=40):
        # Blur and threshold the original image
        self.frame = cv.GaussianBlur(self.colour_space_frame, ksize=(25, 25), sigmaX=0)
        self.frame = cv.inRange(self.frame, lowerb=thresh_lb, upperb=thresh_ub)

        # Remove any noise
        kernel_ellipse_3 = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))
        self.frame = cv.morphologyEx(self.frame, op=cv.MORPH_OPEN, kernel=kernel_ellipse_3)
        self.frame = cv.morphologyEx(self.frame, op=cv.MORPH_CLOSE, kernel=kernel_ellipse_3)

        # Find contours that fulfill the min_area requirement
        _, contours, _ = cv.findContours(self.frame, mode=cv.RETR_EXTERNAL, method=cv.CHAIN_APPROX_SIMPLE)
        contours = [contour for contour in contours if cv.contourArea(contour) > min_area] if contours else None

        # Create a blank frame and fill in the contours
        self.frame = np.zeros(self.frame.shape[:2], dtype=np.uint8)
        if contours:
            cv.fillPoly(self.frame, pts=contours, color=self.WHITE)

            # Morph close to fill in the lines
            self.frame = cv.morphologyEx(self.frame, cv.MORPH_CLOSE,
                                         cv.getStructuringElement(cv.MORPH_RECT, (line_width, line_width)))

    def extract_object(self, thresh_lb, thresh_ub, field=None, kernel_size=3):
        # Blur and threshold the original image
        blur = cv.GaussianBlur(self.colour_space_frame, ksize=(5, 5), sigmaX=0)
        threshold = cv.inRange(blur, lowerb=thresh_lb, upperb=thresh_ub)

        # Apply field mask if given
        self.frame = cv.bitwise_and(field, field, mask=threshold) if field is not None else threshold

        # Remove any noise
        kernel_ellipse = cv.getStructuringElement(cv.MORPH_ELLIPSE, (kernel_size, kernel_size))
        self.frame = cv.morphologyEx(self.frame, op=cv.MORPH_OPEN, kernel=kernel_ellipse)
        self.frame = cv.morphologyEx(self.frame, op=cv.MORPH_CLOSE, kernel=kernel_ellipse)

