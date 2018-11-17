# Extended Kalman Filter Project 

This project consists of using a Kalman filter to estimate the state of a moving object with noisy Lidar and Radar measurements.

The project consists of the following steps:
* Initializing the Kalman filter variables
* Predicting where the object will be after a time lapse of Δt
* Updating the object position based on the measurements received from the sensors


The code has been developed on an Ubuntu 16.04 Virtual Machine running on Oracle VM VirtualBox.
The Term 2 Simulator has been run on the same virtual machine.

The executable file compiled for this project can be found in the folder **build**, under the name **ExtendedKF**

The picture below shows an example of terminal output during the execution of the simulation. 
* x is the mean state vector, which contains information about the object's position and velocity
* P is the state covariance matrix, which contains information about the uncertainty of the object's position and velocity

![Example of terminal output during the simulation](https://raw.githubusercontent.com/sorix6/CarND-Extended-Kalman-Filter-Project/master/images/terminal_output.JPG) 

The picture below, shows a print-screen of the simulator.
* The red circles represent the Lidar measurements 
* The blue circles represent the radar measurements and the arrow points in the direction of the observed angle
* The green triangles represent the estimation markers 

![Simulator print-screen](https://raw.githubusercontent.com/sorix6/CarND-Extended-Kalman-Filter-Project/master/images/simulator.JPG) 





