# Unscented Kalman Filter Project Starter Code

Self-Driving Car Engineer Nanodegree Program

In this project I implemented an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy LIDAR and RADAR measurements.

[//]: # (Image References)

[image1]: images/CTRV-diff-eq.tiff "CTRV"
[image2]: images/solution.tiff "solution"
[image3]: images/process_noise.tiff "noise"


## Project Explanation

In this project I implemented an UKF using the CTRV motion model.

As a quick reminder, a Kalman Filter only handles models with linear equations whereas the Extended and Unscented Kalman Filters handle non-linear equations. I have implemented an Extended Kalman Filter in my previous project that you can find [here](https://github.com/Labonneguigue/CarND-Extended-Kalman-Filter-Project).

The CTRV model stands for Constant Turn Rate and Velocity magnitude model. It takes into account a turn rate \dot{varphi} into its state vector and would therefore predict the position of the car more accurately when it turns.

![alt text][image1]

We need to compute the integral of these values over a small time step (delta t) to obtain equations giving us the values at the time t+1.

![alt text][image2]

To model the stochastic part of the process model we add some process noise.

![alt text][image3]

The Extended Kalman Filter uses the Jacobian matrix to linearize non-linear functions. Instead of using this technique, the Unscented Kalman Filter takes the Posterior Gaussian distribution at time k and predicts the Prior Gaussian distribution at time k+1 through a process called Unscented Transformation. The tricky part is that we do not have any linear equations to do so.

The Unscented Transformation chooses Sigma points. These points are supposed to represent accurately the Posterior Gaussian distribution. They are then inserted into the non-linear process function f().





## Installation

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

---

## Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)


## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/c3eb3583-17b2-4d83-abf7-d852ae1b9fff/concepts/f437b8b0-f2d8-43b0-9662-72ac4e4029c1)
for instructions and the project rubric.
