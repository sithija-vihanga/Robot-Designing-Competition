# Robot Controller for Object Detection, Navigation, Line Following, and Chess Board Solving

This repository contains a robot controller designed for the Webots environment, which utilizes OpenCV for image processing tasks. The controller incorporates various algorithms, such as Gaussian blur and contour detection, to enable efficient object detection within a box arena.

## Installation

To set up the standalone controller for Webots, follow these steps:

1. Download the MinGW files for OpenCV and install them on your system.
2. Clone this repository and navigate to its directory.
3. Place the provided make file in the appropriate location, as described in the Webots documentation.

## Functionality

The robot controller is equipped with a movable camera, allowing it to observe its surroundings by panning up/down or left/right. The image processing capabilities of OpenCV are utilized to accomplish the following tasks:

### Box Detection and Capture

Using Gaussian blur and contour detection algorithms, the robot can detect and capture boxes within the box arena as part of its assigned task. This enables it to identify and interact with the target objects effectively.

### Exit Detection and Navigation

The controller employs OpenCV features to detect the exit from the box arena. By leveraging image processing techniques, the robot can identify the designated white line and navigate towards it autonomously.

### Broken Bridge Avoidance

Front ultrasonic sensors are utilized to detect a broken part of the bridge, and image processing techniques are employed to accurately orient the robot to avoid the damaged section. This ensures that the robot can navigate safely and successfully place the box in the exact required position.

### Line Following

For line following, the robot utilizes infrared line sensors to detect black and white lines. The analog inputs from the sensors are processed using a threshold to determine the line color. A PID motion controller is implemented to ensure precise navigation, especially in areas with dotted lines. To reduce computational load, the camera inputs are turned off during line following.

### Chess Board Solving

For chess board solving, the robot leverages a camera. Initially, gripper arms are used to hoist the first rook into position. The camera is then used to detect the colors of chess pieces on the board. A sharp IR sensor assists in measuring the distance to the chess pieces and the angle of the sensor to the top of each piece. This information is used to identify and map the chess pieces on the board. The solving algorithm is initiated based on this mapping. A string variable, "location," is updated throughout the task to represent the current position of the robot.

## Usage

To use this robot controller, follow the installation instructions provided above. Additionally, ensure that Webots is properly configured and compatible with the installed OpenCV version.

Once the controller is set up, you can run the program in the Webots environment, and the robot will perform the object detection, navigation, line following, and chess board solving tasks as described.

Feel free to customize and modify the code to suit your specific requirements and enhance the capabilities of the robot controller.

Please refer to the documentation and code comments within the repository for further details and instructions.

## Contributions

Contributions to this project are welcome! If you encounter any issues or have suggestions for improvement, please feel free to submit a pull request or open an issue in the repository.

We appreciate your interest and hope this robot controller proves to be a valuable tool for your Webots projects. Happy coding!
