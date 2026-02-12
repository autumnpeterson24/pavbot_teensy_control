Project Description: 
This repository was developed for a pathfinding autonomous vehicle senior design project that uses LiDAR, GPS, IMU
And cameras to navigate and obstacle course and avoid obstacles all without the control of a person

Purpose of Repository:
This repository maintains the files that are used for motor control of an autonomous vehicle
It contains the ROS files that normalize and convert motor commands to be sent to a Teensy 4.1

Environment Needed to Run Project:
OS - Ubuntu 22 OR 24
ROS2 - Humble OR Jazzy
*Teensyduino was used for microcontroller side and requires Arduino IDE 2.3.x*

Files Included in Repository:
PROTOCOL.md - description of the communication protocol used to send serial data

param.yaml - list of parametrized ROS variables for each file

demo.txt - instructions on how to run project, both the stub and the actual transport

cmd_vel_to_wheels.cpp - Nav2-friendly mixer that converts /cmd_vel into differential wheel commands 
			              and publishes normalized left/right commands for Teensy transport.

teensy_transport_serial.cpp - C++ code that converts velocity to direction and PWM and sends the data to a Teensy 4.1
					via USB serial

teensy_transport_stub.cpp - simulates transport layer between Jetson and Teensy
					
CMakeLists.txt - make file for all the files in the workspace

package.xml - xml used for the project<img width="1470" height="1112" alt="image" src="https://github.com/user-attachments/assets/aea189af-52da-4bc6-bfb7-6b1da860b03b" />
