# SFND_Unscented_Kalman_Filter
Sensor Fusion UKF Highway Project Starter Code

![ukf](media/ukf_highway_tracked.gif)

In this project you will implement an Unscented Kalman Filter to estimate the state of multiple cars on a highway using noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

The main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ukf_highway

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, and src/ukf.h

The program main.cpp has already been filled out, but feel free to modify it.

![ukf2](media/ukf_highway.png)

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. 
The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The ego car is green while the 
other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each of the traffic car's has
it's own UKF object generated for it, and will update each indidual one during every time step. 

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z axis is not taken into account for tracking, so you are only tracking along the X/Y axis.

---

## Other Important Dependencies
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
* PCL 1.2
* googletest [click here for installation instructions](https://github.com/google/googletest)
* gocavr 5.0 [click here for installation instructions](https://gcovr.com/en/stable/installation.html)

or a docker image can be used with nivida support [Click here for installation instructions](https://github.com/m-loay/opencv-pcl-gpu-Docker-image)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ukf_highway`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar modify the code in `highway.h` to alter the cars. Also check out `tools.cpp` to
change how measurements are taken, for instance lidar markers could be the (x,y) center of bounding boxes by scanning the PCD environment
and performing clustering. This is similar to what was done in Sensor Fusion Lidar Obstacle Detection.

## Project Instructions and Rubric

### Accuracy
The output RMSE px, py, vx, vy output coordinates [0.04852,0.062393,0.366935,0.400275].
Also the NIS for both lidar and radar is consistent as shown in the following figures:

For Lidar:

![lidar](media/highway--NIS_laser.png)


For Radar:

![radar](media/highway--NIS_Radar.png)

## Project results:

### UKF results:
For Lidar:

![lidarUKF](media/UKF_sample-laser-radar-measurement-data-1--NIS_laser.png)

For Radar:

![radarUKF](media/UKF_sample-laser-radar-measurement-data-1--NIS-Radar.png)

Output position Estimation VS Measurements VS GT:

![outputUKF](media/UKF_sample-laser-radar-measurement-data-1--Output-estimation.png)

Output velocity Estimation VS GT:

![velUKF](media/UKF_sample-laser-radar-measurement-data-1--Output-V.png)

### EkF results:
For Lidar:

![lidarEKF](media/EKF_sample-laser-radar-measurement-data-1--NIS_laser.png)

For Radar:

![radarEKF](media/EKF_sample-laser-radar-measurement-data-1--NIS-Radar.png)

Output position Estimation VS Measurements VS GT:

![outputEKF](media/EKF_estimation.png)

Output velocity Estimation VS GT:

![velEKF](media/EKF_sample-laser-radar-measurement-data-1--Output-V.png)

## Project Quality Work:

### Test Report Generated from google-test frame work:
![tc](media/test_cases.png)

### Coverage Reprot Report Generated from google-test frame work:
![cr](media/coverage.png)


gcovr -r . --exclude-throw-branches --html --html --html-details -o example-html-details.html

