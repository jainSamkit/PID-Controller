# CarND-PID-Control-P4
Udacity Self-Driving Car Nanodegree - PID Control project

# Overview

This project implements a [PID controller](https://en.wikipedia.org/wiki/PID_controller) to control a car in Udacity's simulator([it could be downloaded here](https://github.com/udacity/self-driving-car-sim/releases)). The simulator through websocket sends speed,cross-track error and angle (https://en.wikipedia.org/wiki/WebSocket) and the main functional file sends back the normalized steering input and throttle to the simulator through websocket. 

# Prerequisites

The project has the following dependencies (from Udacity's seed project):

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's simulator.

For instructions on how to install these components on different operating systems, please, visit [Udacity's seed project](https://github.com/udacity/CarND-PID-Control-Project). As this particular implementation was done on Mac OS, the rest of this documentation will be focused on Mac OS. I am sorry to be that restrictive.

In order to install the necessary libraries, use the [install-mac.sh](./install-mac.sh).

# Compiling and executing the project

In order to build the project run ./build.sh as indicated below .Otherwise use

```mkdir build
   cd build
   cmake ..
   make
   (Built target pid)
   run ./pid
```
> ./build.sh
-- The C compiler identification is AppleClang 8.0.0.8000042
-- The CXX compiler identification is AppleClang 8.0.0.8000042
-- Check for working C compiler: /Library/Developer/CommandLineTools/usr/bin/cc
-- Check for working C compiler: /Library/Developer/CommandLineTools/usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Detecting C compile features
-- Detecting C compile features - done
-- Check for working CXX compiler: /Library/Developer/CommandLineTools/usr/bin/c++
-- Check for working CXX compiler: /Library/Developer/CommandLineTools/usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Configuring done
-- Generating done
-- Build files have been written to: REPO_ROOT/CarND-PID-Control-P4/build
Scanning dependencies of target pid
[ 33%] Building CXX object CMakeFiles/pid.dir/src/PID.cpp.o
[ 66%] Building CXX object CMakeFiles/pid.dir/src/main.cpp.o
[100%] Linking CXX executable pid
[100%] Built target pid

## Implementation

### The PID procedure follows what was taught in the lessons.

    The PID implementation is done in the file PID.cpp as done in the class.It calculates the present cross-track error at the most recent state as the proportional error,the difference in the cte as the derivative error and the sum of the errors as the integral error.The parameters are tuned as such to reduce these errors to a minimum threshold value aka the global minima.
## Reflection

### Describe the effect each of the P, I, D components had in your implementation.

- The proportional error tries to bring the car back in the central position by solely relying on the cross track error.If the cross track error is large,it may overshoot the center lane as this only relies on the intensity of the cte.

- The integral error tries to remove the bias error if any.

- The differential error tries to prevent the car from overshooting by penalising the very large change in the cross track error ie if the car
suddenly drops to correct itself,due to proportional error.This smooths out the approach towards the center of the lane. 

### Describe how the final hyperparameters were chosen.

The parameters were choosen through the twiddle algorithm until the sum of the change in the parameters were higher than the threshold value i.e tolerance (0.01 in this case).The parameters were initialised with 0 ,both Kp and Kd and they were tweaked until the change in them was negligible.

## Simulation

### The vehicle must successfully drive a lap around the track.
