# Webots Robot Controller for Object Detection, Navigation, Line Following, and Chess Board Solving

This repository contains a robot controller designed for the Webots environment, which utilizes OpenCV for image processing tasks. The controller incorporates various algorithms, such as Gaussian blur and contour detection, to enable efficient object detection within a box arena.

## Robot 3D Design

![8](https://github.com/sithija-vihanga/Robot-Designing-Competition/assets/115767667/9be356e7-b973-4958-8c12-939d63fa7010)

![10](https://github.com/sithija-vihanga/Robot-Designing-Competition/assets/115767667/27c95544-99f0-4333-94a9-468b1a5c647a)

![13](https://github.com/sithija-vihanga/Robot-Designing-Competition/assets/115767667/14b3fa94-97a1-4450-a688-603a0e136ade)

![14](https://github.com/sithija-vihanga/Robot-Designing-Competition/assets/115767667/00868ce2-8ca6-46de-9c34-60831faa9127)

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





## Navigating on the Ramp

https://github.com/sithija-vihanga/Robot-Designing-Competition/assets/116638289/3660a243-abc7-417c-9a33-be8e391b1c9f






### Line Following

For line following, the robot utilizes infrared line sensors to detect black and white lines. The analog inputs from the sensors are processed using a threshold to determine the line color. A PID motion controller is implemented to ensure precise navigation, especially in areas with dotted lines. To reduce computational load, the camera inputs are turned off during line following.

### Wall Following

The robot uses infrared distance sensors to detect walls and obstacles, maintaining a safe distance from them in the wall-following process. A proportional-Integral-derivative (PID) control algorithm ensures precise navigation. The gains are carefully tuned for optimal performance. This enables the robot to follow walls smoothly and confidently while avoiding collisions and efficiently navigating complex environments.


https://github.com/sithija-vihanga/Robot-Designing-Competition/assets/110180949/b2181521-9dcc-49c7-9574-2067553f0a92

### Dotted Line Following


https://github.com/sithija-vihanga/Robot-Designing-Competition/assets/110180949/143af73f-b127-4746-9edb-153be27e1ba3



### Chess Board Solving

For chess board solving, the robot leverages a camera. Initially, gripper arms are used to hoist the first rook into position. The camera is then used to detect the colors of chess pieces on the board. A sharp IR sensor assists in measuring the distance to the chess pieces and the angle of the sensor to the top of each piece. This information is used to identify and map the chess pieces on the board. The solving algorithm is initiated based on this mapping. A string variable, "location," is updated throughout the task to represent the current position of the robot.

![location](https://github.com/sithija-vihanga/Robot-Designing-Competition/assets/116638289/dbc29525-5db6-4e3b-95b7-bfeaf747d488)


### 
## Usage

To use this robot controller, follow the installation instructions provided above. Additionally, ensure that Webots is properly configured and compatible with the installed OpenCV version.

Once the controller is set up, you can run the program in the Webots environment, and the robot will perform the object detection, navigation, line following, and chess board solving tasks as described.

Feel free to customize and modify the code to suit your specific requirements and enhance the capabilities of the robot controller.

Please refer to the documentation and code comments within the repository for further details and instructions.

# Autonomous Physical Robot 



This GitHub repository contains the code and documentation for an autonomous robot capable of performing four main tasks: one-sided wall following, line following, maze solving, and navigating through a blind box area.

## Overview

The autonomous robot is built using an Arduino Mega controller board, equipped with 2 metal gear motors for driving the robot. The robot has 3 wheels, with the front wheel acting as a free wheel, while the rear two wheels are connected to the motors for propulsion. It uses 7 infrared (IR) sensors to detect lines and 5 ultrasonic sensors to detect walls and objects in its surroundings.

## Tasks:

1. **One-Sided Wall Following**: The robot can navigate through an environment where one side has a wall and the other side is marked by a red line. The robot follows the red line while keeping the wall on one side.

2. **Line Following**: In a black area, the robot can precisely follow a white line using its IR sensors, maintaining its course along the path.

3. **Maze Solving and Optimal Return Path**: The robot is capable of exploring a line maze autonomously and finding the optimal return path to its starting position.

4. **Blind Box Navigation**: The robot can navigate through a box-like area with random obstacles and multiple exits, successfully finding the correct outlet.

## Hardware Testing

### IR Sensor Array
https://github.com/sithija-vihanga/Robot-Designing-Competition/assets/106132194/85d465b3-0766-4df0-9459-13d2586e0e0c 

### Motors and Motor Driver
https://github.com/sithija-vihanga/Robot-Designing-Competition/assets/106132194/11cafb27-a3c2-4245-8556-287d934b2579


### Line Following Task 
https://github.com/sithija-vihanga/Robot-Designing-Competition/assets/106132194/b9650d9f-ea88-46c7-bc12-cc00e7f0c44a 


### Wall Following Task
https://github.com/sithija-vihanga/Robot-Designing-Competition/assets/106132194/638f4621-4716-47ac-b6b9-7b8c9b40d2a5



## Getting Started

### Prerequisites

To run the robot's code on your Arduino Mega, you'll need the following:

- Arduino IDE (or any suitable Arduino programming environment) installed on your computer.
- USB cable to connect the Arduino Mega to your computer.

### Installation

1. Clone this GitHub repository to your local machine.
2. Open the Arduino IDE and load the robot's code from the repository.
3. Connect your Arduino Mega to your computer using the USB cable.
4. Compile the code and upload it to the Arduino Mega board.

## Robot Configuration

To set up the robot for each task, follow the specific instructions provided in the respective code files. Ensure that the wheels and sensors are correctly connected, and the robot is placed in the designated starting position for each task.

## Contributions

Contributions to this project are welcome! If you encounter any issues or have suggestions for improvement, please feel free to submit a pull request or open an issue in the repository.

We appreciate your interest and hope this robot controller proves to be a valuable tool for your Webots projects. Happy coding!

Our Team

