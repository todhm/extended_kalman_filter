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

I used xcode as main IDE. You can execute project with ide_profiles/xcode/ExtendedKF.xcodeproj file. 

Also you can execute project with following steps.
0. Install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems and [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) for windows.
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
| px |   0.0973   |   0.1473   |   0.2302   |
| py |   0.0855   |   0.1153   |   0.3464   |
| vx |   0.4513   |   0.6383   |   0.5835   |
| vy |   0.4399   |   0.5346   |   0.8040   |

* Here are the visualization of result in the simulator.

#### Dataset1 result in the simulator.
[![alt text][image1]](https://youtu.be/nN3dMHDyro8)


#### Dataset2 result in the simulator.
[![alt text][image2]](https://youtu.be/NDZiqUevtEI)

---
## Discussion  
#### Here are the crucial tuning point that was important to improve the reduce RMSE of estimations.
* In the measurement update step we calculate the difference between our measurement and returned value of measurement function with  predicted state value(expressed as 'y')  we should avoid the case where phi of y exeed the [-pi,pi] range. 
* We should handle the case where px and py is near 0 which lead to Nan value in Matrix variable during the calculation.


