# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator. You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

# Model Documentation

I have defined multiple classes to organise my code better:
* **PathPlanner** class: which does the main work to define the path.
* **Map** class: to save the map waypoints information.
* **Track** class: which defined the geometry of a track to follow. This can be a lane or a 
track which may be followed during the lane change. All the tacks are defined based on two splines such as (s,x) and 
(s,y). That means to get the `x,y` coordinate of the track given the `s` there is one spline for one coordinate, in total 2 spline.
* **tools** class: including all helper functions and also plotting features. In order to use this you will need the `gnuplot`.(`sudo apt install gnuplot`). Right now all the codes for plotting is commented therefore the current version does not need it.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

# License:
You can do what ever you want with this code.
There is no guarantee for this code to work for anything.
