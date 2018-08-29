# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

# Project Rubric

![Intro Image](/images/track00020.jpg)

## Compilation

The compilation is described below in the Basic Build Instuctions in more detail.  These are the steps I followed to build the project on Windows 10 machine with Ubuntu Bash.

```
clone the git repo
mkdir build
cd build
cmake ..
make
./path_planner
```

## Valid Trajectories

### The car is able to drive at least 4.32 miles without incident

A [Video Example](https://youtu.be/sXjHurA8eOo) of the car performing the required distance is provided as the performance varies from PC to PC and requires different scaling factors for accelleration and jerk.

### The car drives according to the speed limit 

Desired is set to 49.5 MPH which is 0.5 MPH under the 50 MPH speed limit.  This provides a safety margin with out exceeding.

### Max Acceleration and Jerk are not Exceeded

This is prevented by limiting how much the vehicle can change velocity in a given time step with a safety factor so an acceleration of 10 m/s^2  and a jerk of 10 m/s^3 are not exceeded.

### Car does not have collisions

This is prevented by checking for occupancy in the desired lane within a safty zone.

### The car stays in its lane, except for the time between changing lanes

The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

### The car is able to change lanes

The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic. If it is not meeting the desired speed limit, it will make opportunistic lanes changes if there is an opening to maintain a velocity as close to the traffic speed limit as it can.

## Reflection

Driving logic for this project incorporates a very simple state machine with goal orientated behaviour to ensure that the car is traveling as close to the traffic speed limit as possible.

The algorithm opportunistically changes lanes based on the current speed of the car in front and whether there is an opening left or right of the vehicle. If there is a car in front and the reference velocity is higher than the car velociity it will try to change lanes provided the lanes are open.

```
if (too_close && ref_vel >= target_vel) {
            ref_vel -= 0.224 / 2; // 0.05 m/s
            if (lane > 0 && left_close == false)
              lane--;
            else if (lane < 2 && right_close == false)
              lane++;
          } else if (ref_vel < 49.5) {
            ref_vel += 0.224 / 2;
          }
```

The car avoid collisions by ensuring that no car is beside it or up to 20 meters in front of it. Here is an example of checking for a car too close in the left lane:

```
              check_car_s += ((double)prev_size * 0.02 * check_speed);
              if ((check_car_s > car_s - 5) && ((check_car_s - car_s) < 20)) {
                left_close = true;
              }
```

The trajectory generation is composed of a series of waypoints the car will follow sequentially every .02 seconds.
To increase velocity the path would spread the points further apart and to decrease velocity the points could be closer togehter.

A [spline function]( http://kluge.in-chemnitz.de/opensource/spline) is used to minimize jerk without the need for  generating various polynomial fits and using a jerk minimizing cost function to encure that the limits are not exceeded.  

The spline itself was calculated in Frenet coordinates and mapped back and forth to world using the map road marker locations. A smooth transition between trajectory updates was maintained by adding subsequent points on to the points in the original path that were not covered by the vehicle before the next trajectory update.

If the left over trajectory size is almost empty we use the car as a starting reference, otherwise we use the previous path's last two enpoints to generate a reference point for the new spline trajectory.

```
          if (prev_size < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // use the previous path's end point as a starting reference
          else {

            // redefine reference state as previous path end points
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // use two points that make the path tangent to the previous path's
            // end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
```

The target of the trajectory was set far enough in front of the vehicle (30 waypoints) in the center of the desired lane to ensure that lane and speed transitions and were gradual enough to not exceed the acceleration and jerk limits in both normal and tangential directions.

```
          vector<double> next_wp0 =
              getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s,
                    map_waypoints_x, map_waypoints_y);
```
Two more waypoints are added using the same method at increments of 30.

The trajectory spacing, which determines velocity is based on the velocity required and finding the spacing that fulfills this.

```
            double N = target_dist / (0.02 * ref_vel / 2.24);
            double x_point = x_add_on + target_x / N;
```

The reference yaw used for trajectory generation is not in the Frenet frame and must be rotated back.

```
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;
```

## Outro

The coursework covers a lot of concepts for trajectory generation that were not used in this particular implementation. In fact this repo is a reduction from my original [Forked Repo](https://github.com/SynapticSugar/CarND-Path-Planning-Project) from Udacity that contains a complete path planner with cost minimizing trajectories and a fully realized finite state machine.  Unfortunately it was a bit overambitious to code and debug to get the sucessful submition in time. Sometimes simpler is better!

# Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

# Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

# Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

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

# Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

# Dependencies

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

# Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

# Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
