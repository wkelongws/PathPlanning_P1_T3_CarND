# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data are provided, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.csv
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. cd to this repo.
3. Clean everything: `./clean.sh` (Don't need to do this if never compiled in this repo before)
4. Compile: `./build.sh`
5. Run it: `./run.sh`
6. Open the simulator and watch the car drive itself.

## Description of the solution

There are 5 major components in my solution: 1. the `Vehicle` class; 2. available status of the ego vehicle; 3. generate potential trajectories; 4. select the best trajectory; 5. Construct the actural path.

### 1. the `Vehicle` class

A vehicle class is constructed to hold all the behavior of the vehicles and keep the main function as clean as possible.
The ego vechile as well as all other vechiles on the same side are constructed as the `vehicle` objects. Planned path of the ego vehicle and predicted path of the other vehicle can be computed.

### 2. available status of the ego vehicle

All potential status of the ego vechile include: "keep lane", "lane change left" and "lane change right". Based on the relative location between the ego vehicle and other vehicles at the time of planning, "lane change left" and "lane change right" may be removed from the available status pool.

### 3. generate potential trajectories

For each available status of the ego vehicle, one potential planning trajectory is computed. The planning period is 1.5 second. This is inspired by the 30m used to generated the final path in the "project walkthrough" video. In the "project walkthrough" video, any lane change needs to be completed within 30m length. Driving at the speed limit of about 21m/s, this value yeilds to be 1.5 seconds. So any lane change needs to be completed within 1.5 second. So I compute the potential planning trajectory in a 1.5 seconds period. The computation of end points of the potential trajectories takes the prediction of other vehicles during the 1.5 second period into consideration. The potential trajectories are jerk-minimized using the 'jmt' function.

### 4. select the best trajectory

The trajectory selection is done by using a multi-criteria cost function. The trajectory with lowest cost gets selected.
The cost function consists of 4 components:

a. `Efficiency_cost` rewards trajectories with high average speed. This item drives lane changing behavior to happen when encountering low speed vehicle in front.

b. `In_lane_buffer_cost` rewords trajectories with larger gap from other vehicles in the same lane with the ego vehicle. This item makes the ego vehicle choose a lane with longer longitudinal clearance when changing lanes.

c. `Not_middle_lane_cost` penalizes not shooting for middle lane (d = 6). This item will bring the ego vehicle to the middle lane when everything is clear.

d. `Collision_cost` denies all trajectories that will cause collision.

The overall behavior the ego vehicle is controlled by the weights of all four cost items. A very high weight is given to `Collision_cost` to make safety the highest priority. The weight of `Efficiency_cost` comes as the second largest so that the lane changing behavior can be made in time. `Not_middle_lane_cost` is supplementary and should not affect other items when they suppose to work so the lowest weight is given to `Not_middle_lane_cost`.

### Construct the actural path

After the best trajectory is selected, to make the actual path smoothly connected with the previous path, a spline function is applied to 5 anchor points of the best trajectory.

### A vedio record of the vehicle driving one complete loop and be found [here](https://youtu.be/xTSy6F4BY2k).

--------

# Below are some more information and description about the simulator provided by Udacity. Take a look if you are interested.

## Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Some details about the simulator

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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