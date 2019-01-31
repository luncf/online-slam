# Simultaneous Localization and Mapping

This is a navigation system for the robot that incorprates SLAM which is able to locate itself and update the obstacles in a pre-known map (soccer field), using **particle filter (probability)** method.

The vision module is from Assignment 1 with some minor bug fixes and part of the navigation module is adapted from Assignment 2. The SLAM module will try to estimate the robot's position based on the probability maps and map objects within it's view.


## Getting Started

### Requirements

This event has the following requirements:

1. OpenCV 3.2+
2. Python 2.7
3. Darwin-OP 3 along with the framework

### Features

1. Localization based on probability maps
2. Mapping predefined obstacles with vision


### How to run the project

1. Copy the files over to Darwin-OP 3 into `~/catkin_ws/src/`
2. Compile
```bash
cd catkin_ws
catkin_make
```
3. Use the ROS launch file to start the program
```bash
roslaunch slam slam.launch
```
4. Press the `mode` button on the Darwin-OP 3 to reset
5. Press the `start` button on the Darwin-OP 3 to start the event

The vision can be tuned via the GUI. To start, select the desired object to tune using the `Object` trackbar in the `debug` window. Change the average colour space value by selecting a range in the `camera` window. Modify the threshold values via the other three trackbars.

### Configuration

#### Vision module

The vision module has a configuration file under `config/configuration.json`. Use this file to tune the vision based on the environment it is used in.

The configuration file gives the ability to change the setting of the
camera image and the detected features.

Camera:

- camera_index: the index of the video camera device (default: 0)
- resized_frame_height: height of the resized frame, useful
    to reduce processing load (default: 320)

Obstacles:

- min_area: minimum area of contours (default: 5000)
- max_area: maximum area of contours (default: 100000)
- output_colour: line colour of the object in the output frame (default: obstacle colour)
- threshold: value added to calculate the min/max rnage of the colour space (default: [0, 0, 0])
- value: average value of the selected range in the colour space (default: [0, 0, 0])

Field:

- min_area: smallest contour area to consider as part of the field (default: 1500)
- threshold: value added to calculate the min/max range of the colour space (default: [0, 0, 0])
- value: average value of the selected range in the colour space (default: [0, 0, 0])

Lines:

- corner_max_distance_apart: maximum distance between lines to be consider as a corner (default: 20)
- max_distance_apart: maximum distance between line segments to be considered as a line (default: 20)
- max_width: maximum width of each line (default: 20)
- min_length: minimum length of each line segment (default: 20)
- output_boundary_line_colour: line colour of the boundary lines in the output frame (default: blue)
- output_center_line_colour: line colour of the center line in the output frame (default: cyan)
- output_goal_area_line_colour: line colour of the goal area lines in the output frame (default: magenta)
- output_undefined_line_colour: line colour of any lines that can not be classified (default: red)
- threshold: value added to calculate the min/max range of the colour space (default: [0, 0, 0])
- value: average value of the selected range in the colour space (default: [0, 0, 0])

#### Navigation module
The navigation module has a configuration file under `config/`. Use this file to tune the walk based on the environment it is in.

- **walking_configurations.json**: This configuration file lists the walking gaits for each different type of walk. See the Darwin-OP 3 guide to understand each parameter.

#### SLAM module
The SLAM module has a configuration file under `config/`. Use this file to tune the walking rates based on the walking configuration of the navigation module.


## Future additions
- [ ] Different movement based on obstacle location
- [ ] Include lines seen in navigation decision  
- [ ] Better initial localization (currently only supports 3 points)
