# Unscented Kalman Filter Project

[![Build Status](https://travis-ci.org/Labonneguigue/CarND-Unscented-Kalman-Filter-Project.svg?branch=master)](https://travis-ci.org/Labonneguigue/CarND-Unscented-Kalman-Filter-Project)


Self-Driving Car Engineer Nanodegree Program

In this project I implemented an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy LIDAR and RADAR measurements.

[//]: # (Image References)

[image1]: images/CTRV-diff-eq.tiff "CTRV"
[image2]: images/solution.tiff "solution"
[image3]: images/process_noise.tiff "noise"
[image4]: images/sigma.tiff "sigma"
[image5]: images/augmentation.tiff "augmentation"
[image6]: images/compute.tiff "compute"
[image7]: images/prediction.tiff "prediction"
[image8]: images/meas_pred.tiff "meas_pred"
[image9]: images/meas_pred2.tiff "meas_pred2"
[image10]: images/a_2_yawdd_03.tiff "better"
[image11]: images/a_3_yawdd_08.tiff "worse"
[image12]: images/result.tiff "result"
[image13]: images/sparse.tiff "sparse"
[image14]: images/nis.tiff "nis"

## Project Explanation

In this project I implemented an UKF using the CTRV motion model.

As a quick reminder, a Kalman Filter only handles models with linear equations whereas
the Extended and Unscented Kalman Filters handle non-linear equations. I have implemented
an Extended Kalman Filter in my previous project that you can find [here](https://github.com/Labonneguigue/CarND-Extended-Kalman-Filter-Project).

The CTRV model stands for Constant Turn Rate and Velocity magnitude model. It takes
into account a turn rate phi dot into its state vector and would therefore predict
the position of the car more accurately when it turns.

![alt text][image1]

Such as the Extended KF, the Unscented KF has a prediction step and a measurement step.
I'll break them down:

#### Prediction Step

We need to compute the integral of these values over a small time step (delta t) to
obtain equations giving us the values at the time t+1.

![alt text][image2]

To model the stochastic part of the process model we add some process noise.

![alt text][image3]

The Extended Kalman Filter uses the Jacobian matrix to linearize non-linear functions.
Instead of using this technique, the Unscented Kalman Filter takes the Posterior Gaussian
distribution at time k and predicts the Prior Gaussian distribution at time k+1 through
a process called Unscented Transformation. The tricky part is that we do not have any
linear equations to do so.

The Unscented Transformation chooses Sigma points. These points are supposed to represent
accurately the Posterior Gaussian distribution. Here is a summary of how to generate them:

![alt text][image4]

I generated `2 * n_x + 1` points where `n_x` is the dimension of the state vector plus
the dimension of the process noise. This technique is called Augmentation. It is because
we do have non-linear process noise effects that it must be accounted for when generating
the sigma points.

In this project the dimensionality of my state vector is 5 and the process noise is 2.
That makes it 15 sigma points.
Lambda governs the spreading from the mean value.

![alt text][image5]

They are then inserted into the non-linear process model function f().

![alt text][image6]

From the obtained points, I then extracted a new Gaussian distribution. Here are the
formulas I've used to compute the new mean and covariance at each prediction step.

![alt text][image7]

There are multiple ways to determine the weights omega. The only thing to remember is that
it is linked to lambda, as you want to recover from the spreading inferred to the sigma
points when they were calculated.

#### Measurement Step

The measurement step moves the gaussian distribution predicted by the prediction
step according to the receive actual sensor measurement. The shortcut that is often
used is to generated sigma points from the prediction step.

To be able to use the generated sigma points from the prediction step, I mapped them onto
the measurement space.

![alt text][image8]

The major difference at this step is that we add the measurement covariance noise instead to
applying the non-linear process function as I did in the previous step. This is due to the
fact that the impact of the noise on the measurement is purely additive and doesn't have a
non-linear relationship.

![alt text][image9]

For the laser data, it was even easier since the sigma points are already expressed in the measurement space (px and py).

### Parameter Tuning

#### Process noise

I tweaked the process noise standard deviations (longitudinal and rotation accelerations) to obtain the best ones.
The way to find a first estimate of these values is to think about what they represent in real life. The square root of `std_a` is expressed in meters / second. The question is what is the expected acceleration of a bicycle at any given time. 

I first started with

```cpp
std_a = 3;
std_yawdd = 0.8;
```

and obtained the following RMSE (root mean squared error):

![alt text][image11]

I then observed that by constraining them a little more, the result did improve.

![alt text][image10]

I ended up with these process noises:

```cpp
std_a = 2;
std_yawdd = 0.3;
```

#### Measurement noise

I calculated the NIS (Normalized Innovation Squared) for each measurements to assess whether my measurement noises were set properly.

![alt text][image14]

I obtained the following result at the end of the simulation using the dataset 1:

```bash
82.7309 % radar measurement below 95% limit.
97.992 % laser measurement below 95% limit.
```

Having only 82% of epsilon values under the 95% limit means that I was underestimating the uncertainty (therefore the noise) of the radar measurements.

On the contrary, having 98% of the laser measurements below the 95% limit (taken this time with 2 degrees of freedom instead of 3) means that I was overestimating the uncertainty of the laser measurements.

As a result, I changed the radar measurement noise standard deviation angle from `std_radphi_ = 0.0175;` to `std_radphi_ = 0.03;`.

I also changed reduced the laser measurement noise by `0.01` to:
```c++
std_laspx_ = 0.14;
std_laspy_ = 0.14;
```

I finally obtained NIS values as follow:

```bash
95.9839 % radar measurement below 95% limit.
95.9839 % laser measurement below 95% limit.
```

As a caveat, it seemed to have increased a tiny bit the RMSE of each observed parameters except the speed error in the x direction `vx` which is the highest of them all.
Second caveat, I could not actually change these values to pass the project ...

## Unit Tests

It is the first time I created unit tests for a project of the SDC Nanodegree and I'm very glad I took the time to do it. After implementing every functions I needed to complete each step of the UKF, my project was done ! No debugging time was needed whatsoever.

It will also save me some time if later on I want to merge this code into a larger project because if something goes wrong the individual functions won't be the cause. I'll just have to write integration tests.

To run the tests:

```bash
cmake -Dtest=ON
make test
```
or
```bash
./unit_tests
```

Here is my unit tests output:

```bash
[==========] Running 12 tests from 4 test cases.
[----------] Global test environment set-up.
[----------] 1 test from Normalize
[ RUN      ] Normalize.EdgeCases
[       OK ] Normalize.EdgeCases (0 ms)
[----------] 1 test from Normalize (0 ms total)

[----------] 2 tests from NIS
[ RUN      ] NIS.NIS_ZeroVectors
[       OK ] NIS.NIS_ZeroVectors (0 ms)
[ RUN      ] NIS.NIS_ProperFunctionning
[       OK ] NIS.NIS_ProperFunctionning (0 ms)
[----------] 2 tests from NIS (0 ms total)

[----------] 4 tests from isNISAboveLimit
[ RUN      ] isNISAboveLimit.DF_2_Below
[       OK ] isNISAboveLimit.DF_2_Below (0 ms)
[ RUN      ] isNISAboveLimit.DF_2_Above
[       OK ] isNISAboveLimit.DF_2_Above (0 ms)
[ RUN      ] isNISAboveLimit.DF_3_Below
[       OK ] isNISAboveLimit.DF_3_Below (0 ms)
[ RUN      ] isNISAboveLimit.DF_3_Above
[       OK ] isNISAboveLimit.DF_3_Above (0 ms)
[----------] 4 tests from isNISAboveLimit (0 ms total)

[----------] 5 tests from UKFTest
[ RUN      ] UKFTest.AugmentedSigmaPointGeneration
[       OK ] UKFTest.AugmentedSigmaPointGeneration (1 ms)
[ RUN      ] UKFTest.PredictSigmaPoint
[       OK ] UKFTest.PredictSigmaPoint (0 ms)
[ RUN      ] UKFTest.PredictMeanAndCovariance
[       OK ] UKFTest.PredictMeanAndCovariance (0 ms)
[ RUN      ] UKFTest.PredictRadarMeasurement
[       OK ] UKFTest.PredictRadarMeasurement (1 ms)
[ RUN      ] UKFTest.UpdateState
[       OK ] UKFTest.UpdateState (0 ms)
[----------] 5 tests from UKFTest (2 ms total)

[----------] Global test environment tear-down
[==========] 12 tests from 4 test cases ran. (3 ms total)
[  PASSED  ] 12 tests.
Program ended with exit code: 0
```


## Final result

Here is a screenshot of the final result:

![alt text][image12]

If we zoom closer, it is quite impressive to see that the green marks (the filtered position of the car) seems very accurate and filters out very well the sparse measurement from both sensors.

![alt text][image13]

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
