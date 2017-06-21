# Extended Kalman Filter Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[image1]: ./result_pic/result_dataset1.png
[image2]: ./result_pic/result_dataset2.png

In this project I utilized a extended kalman filter model to estimate the state of a moving object of interest with noisy lidar and radar measurements. This code consists of following processes.
* Read the data from project simulator which provide radar and laser measurement of moving object near by car in the simulator.
* Estimate the position and velocity of the object by kalman filter model.
* Represent the estimation and RMSE value on the simulator.

The source code of this project consists with followings.
* main.cpp: Reads the data and send the processed data to simulator.
* FusionEKF.cpp: Takes the sensor data and initializes variables and updates variables.  
* kalman_filter.cpp: Instance which hold the matrixes and vectors and methods to update and predict.
* tools.cpp: tools to calculate RMSE and Jacobian Matrix. 

The visualization of this project was made with the Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
4. Run it: `./ExtendedKF`  
5. Run the simulator and watch the results.
---

## Results
* Comparing the result implemented by different sensor we can verify that fused sensor bring us better result.
#### RMSE Value by sensor.

|    |Fused sensor | only laser | only radar |
|:--:|:----------:|:----------:|:----------:|
| px |   0.0954   |   0.1473   |   0.2302   |
| py |   0.0853   |   0.1153   |   0.3464   |
| vx |   0.3861   |   0.6383   |   0.5835   |
| vy |   0.4036   |   0.5346   |   0.8040   |

* Here are the visualization of result in the simulator.

#### Dataset1 result in the simulator.
![alt text][image1]
