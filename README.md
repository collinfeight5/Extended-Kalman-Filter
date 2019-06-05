[//]: # (Image References)

[image1]: ./Images/EKF-1.png "Simulator"

# Extended-Kalman-Filter
The goals of this project were the following:
* Use Lidar/Radar measurements provided through a Udacity simulator
* Build an Extended Kalman Filter in C++ to accurately predict the vehicles position and speed
* Use Root Mean Square Error(RMSE) to determine how accurately the model was working 

My Project consists of the following files, all of which are in the src folder:
* main.cpp, which communicates with the simulator gathers the data
* FusionEKF.cpp, which revieves sensor data and updates the variables used in the Kalman Filter Equations
* kalman_filter.cpp, which contains the kalman filter equations for prediction and updates
* tools.cpp, which contains the functions for RMSE and Jacobian Matrix
* Header files support these files
* Inside of the source folder, "Eigen" is an external library used for this project

I have attached a screenshot from the simulator running below, and the marks on the screen are as follows:
* Lidar measurements are red circles
* Radar measurements are blue circles with an arrow pointing in the direction of the observed angle
* Estimation markers are green triangles.

![alt text][image1]

