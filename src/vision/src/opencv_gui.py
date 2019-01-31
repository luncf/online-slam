import cv2 as cv


COLOUR_SPACE_CHANNELS = ('H', 'S', 'V')
COLOUR_SPACE_MAX_VALUES = (255, 255, 255)


DEBUG_WINDOW_NAME = 'debug'
COLOUR_SPACE_THRESHOLD_TRACKBAR_NAME_FORMAT = '{0} threshold'


def create_window(name, x=0, y=0):
    # Create a window with name at position (x, y)
    cv.namedWindow(name)
    cv.moveWindow(name, x, y)


def create_trackbar(window_name, trackbar_name, default_value, max_value, callback):
    # Create a trackbar in window
    create_window(window_name)
    cv.createTrackbar(trackbar_name, window_name, default_value, max_value, callback)


def set_trackbar_position(trackbar_name, window_name, position):
    # Set trackbar position
    cv.setTrackbarPos(trackbar_name, window_name, position)


def set_mouse_cb(window_name, callback):
    # Set mouse callback
    create_window(window_name)
    cv.setMouseCallback(window_name, callback)


def show(window_name, frame):
    # Show any window
    cv.imshow(window_name, frame)


def show_debug_window(frame, text):
    # Show the debug window with the selected object name
    frame = cv.cvtColor(frame, code=cv.COLOR_GRAY2BGR)
    cv.putText(frame, text=text, org=(10, 30), fontFace=cv.FONT_HERSHEY_DUPLEX, fontScale=1, color=(0, 0, 255))
    cv.imshow(DEBUG_WINDOW_NAME, frame)


def create_colour_space_threshold_trackbar(default_colour_space_threshold, callback):
    create_window(DEBUG_WINDOW_NAME)

    # Create the appropriate colour space threshold trackbars in the object window
    cv.createTrackbar(COLOUR_SPACE_THRESHOLD_TRACKBAR_NAME_FORMAT.format(COLOUR_SPACE_CHANNELS[0]),
                      DEBUG_WINDOW_NAME, default_colour_space_threshold[0], COLOUR_SPACE_MAX_VALUES[0],
                      lambda value: callback([value, -1, -1]))
    cv.createTrackbar(COLOUR_SPACE_THRESHOLD_TRACKBAR_NAME_FORMAT.format(COLOUR_SPACE_CHANNELS[1]),
                      DEBUG_WINDOW_NAME, default_colour_space_threshold[1], COLOUR_SPACE_MAX_VALUES[1],
                      lambda value: callback([-1, value, -1]))
    cv.createTrackbar(COLOUR_SPACE_THRESHOLD_TRACKBAR_NAME_FORMAT.format(COLOUR_SPACE_CHANNELS[2]),
                      DEBUG_WINDOW_NAME, default_colour_space_threshold[2], COLOUR_SPACE_MAX_VALUES[2],
                      lambda value: callback([-1, -1, value]))


def set_colour_space_threshold_trackbar_position(colour_space_threshold_values):
    # Modify trackbar positions
    for index, channel in enumerate(COLOUR_SPACE_CHANNELS):
        set_trackbar_position(COLOUR_SPACE_THRESHOLD_TRACKBAR_NAME_FORMAT.format(channel),
                              DEBUG_WINDOW_NAME, colour_space_threshold_values[index])


def teardown():
    cv.destroyAllWindows()
