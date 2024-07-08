Maze Solving Robot
===================

This is the source code for my shortest path maze-solving robotics project. The aim of this project was to develop code to control a Lego Mindstorms EV3 robot using the leJOS framework designed for Lego Mindstorms programmable bricks.

Files
------

javaRobot.java - This file contains the tested and functional code implemented for this project. It has undergone extensive testing and includes the necessary algorithms that enable our robot to navigate a maze using the shortest path. This file also includes various behaviours, such as the buttonPressBehaviour, which terminates the robot when the ENTER button is pressed.

Robot Details
-------------

<p float="left">
  <img src="https://github.com/qas-dev/Maze-Solving-Robot/assets/153782615/0ae2f806-1b0b-41a8-9b48-6090aa265b60" width = "300">
  <img src="https://github.com/qas-dev/Maze-Solving-Robot/assets/153782615/8daefcdc-9848-4489-b8f3-2ec72e658e3a" width = "300">
</p>

Our robot used various hardware components that allowed it to navigate the maze efficiently:

- 2 Light sensors
- 1 Ultrasonic sensor
- 1 Sound sensor
- 2 Motors

During this project, a physical maze made of card, black tape, and red stickers was created. The black tape represents the paths of the maze that the robot follows to reach the end. The red stickers at each vertex indicate places where the robot can stop and change direction if needed.

The software aspect of the maze includes a two-dimensional binary matrix that represents the physical maze. The matrix is composed of the following:

- "1" - The path is valid/on
- "0" - The path is invalid/off
- "E" - Signifies the end coordinate where the path ends

Robot Control Flow
------------------

<p float="left">
  <img src="https://github.com/qas-dev/Maze-Solving-Robot/assets/153782615/0381a924-3de6-4762-9f5e-319370189200" width = "600">
</p>


Demonstration Video
--------------------

To use the robot, it must first be placed on a suitable maze. Then, the maze configuration and start and end coordinates must be entered. After this, the operator should clap near the robot to initiate the shortest pathfinding.

<video src="https://github.com/qas-dev/Maze-Solving-Robot/assets/153782615/e63b6db2-6839-4c73-a001-1f56a6175052"></video>

The robot uses a BFS algorithm to process the maze and find the shortest path. Once this is done, the movement functions will guide the robot appropriately towards the endpoint "E" where the path terminates.

If the robot encounters an obstacle, it stops, sounds an alarm, and waits until the obstacle is removed before continuing.
