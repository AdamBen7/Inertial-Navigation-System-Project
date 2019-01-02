#INS-Project
Adam Benabbou & Nathan Holt
April 27, 2017
Flight Control System - Inertial Navigation System

By the end of the second semester working on this project (Spring 2017), we were able to develop an INS System almost capable of positional and orientational awareness using a Kalman Filter with a low-cost IMU's accelerometer and gyroscope data.
The INS system can be plugged to a Raspberry Pi and output data into a file using serial.
Our INS System contains a highly customizable and easy to use debugging printing system.
Class-based coding architecture (such as Kalman.h object)
Coded in such a way that _dt values are derived from the system instead of explicitly being defined. The algorithm can easily switch between the two ways and work (since Mat[] is updated everytime the State is updated.
We used Eigen30, a branch of the Eigen library popularily used by C++ coders for easy matrix manipulations. This contains different ways to define/setup a matrix. Definitely would be useful to look into for higher efficiency of code. 
We are also very proud of our plotter/listener program use, which we found online to generate real-time live graphs for serial data we obtains. This program has multiple bugs so if a better (maybe industry-standard) program exists, we would definitely urge you to move to that and master it.

Originally,we used the GY-85 IMU for our project but due to its suboptimal build (we believe), applying the Kalman Filter was unable to be effective enough for our purposes. The GY-85 library used which we originally found online can be found in the libraries folder. 

We then moved on to using the MPU9250 IMU which gave us adequate results for a stationary test. However, that was possible since we directly subtracted gravity induced acceleration data from our raw readings using the variable baX, baY, baZ. (which was originally used to account for bias). 

Currently, in the folder, are two versions of our INS one using Euler Angles and the other Quaternions. 
Unfortunately, the resulting data is still not acceptable for an actual flight test.

Future Works:

-Based on our understanding, the following are the possible causes for the bad data which must be considered and fixed:
  1)Sensor alignment might not be ideal (manufacturer's fault... should be correctable mathematically)
  2)Values of the maximum and minimum sensor readings per sensor axis can be unsymetric (can be corrected through a calibration procedure)
  3)Gravitational acceleration readings become an issue when orientation of IMU changes. (mathematical mistake)
  4)Results are minor acceleration readings along with minor orientation drifts resulting in an accumulation of errors.
  5)The Gravitational readings we obtain for creating a gravity vector contains noise not filtered by the Kalman filter.
  6)Timestep, dt, is becoming larger and larger everytime we increase complexity of our algorithms. 
 
-Use correct convention for variables and functions naming.
-Fully Complete and Review Kalman Filter
-Complete 3D Test and demonstrate high accuracy/reliability
-Remove Debugger Statements
-Increase Efficiency of Code
-Rely more on MatLab to test and plan coding as the project expands.

Files Contained Within:
-INS Euler

-INS Quaternion
(change names to INS and Kalman when attempting to use)

Libraries:
-Eigen30
-Plotter
-MPU9250
-GY85 
-Trigd

-Kalman Quaternion
-Kalman Euler

Tools:
-Plotter (listener) (https://github.com/devinaconley/arduino-plotter includes OS specific listener applications)


Hardware Setup:
Teensy 3.2 Microcontroller
MPU9250 IMU
MicroUSB-USB Cable for Serial Communication
Several small wires

Picture of Setup in Folder. Note that GY85(smaller blue chip) is not in use.

