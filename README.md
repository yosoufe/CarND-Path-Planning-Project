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

The function `PathPlanner::get_path` does the main job. Here there are two states defined:

#### KEEPING_LANE:
which follows the current lane and keep an eye on the front cars not to hit them.
First it looks the cars in front. If there is a car inside the secure distance from the car in the current lanes, it looks 
for the other lanes to find an escape manouver. This is done if there is no car as well in other lane in the seure lanes. 
That means There should not be any car in `secure distance + 20` meters forward and `10` meters backward to change the 
state to the CHANGING_LANE. The `secure distance` is dependent of the speed. Faster the car is driving, the larger the `secure distance` is considered.

Before changing to CHANGING_LANE state it produces the desired track for lane change. This track is designed like the below:
* It chooses few points in current lane from 20 meters of back side of the car until the `s` position of the front car (obstace), then
* chooses few point in the target lane from 40 metere in front of the front car (obstacle) until 80 meter in front of the front car (obstacle), then
* it fits two splines to these points. (s,x) and (s,y).
* Now using these tracks
#### CHANGING_LANE:
It is simply doing the changed lane and followinf the track produced before as descrribed above. It switches again to KEEPING_LANE state when it reaches close to the end of the generated track.

## Following a track:
Whether it is a lane or produced for lane change. It is defined in function `PathPlanner::keep_track`.
* it saves the first 20 unmet points from the previous generated path to have a smooth maneuver.
* The it generates `s` values along the given track based on the given target speed.
* Then it generates the `x` and `y` values given the track and using the spline equations. `(s,x) & (s,y)`.

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
