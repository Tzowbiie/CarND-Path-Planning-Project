# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

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

---

## Reflection

### Evaluation of Sensor Fusion Data

#### Close range evaluation
The first goal is to gather information about the close sourrounding traffic given by the Sensor Fusion Data provided by the simulator.
With this information crashes can be avoided succesfully.
Following car flags are introduced and evaluated in each frame:
```
//car flags
bool too_close = false; // ego car is getting close to car in front
bool emerg_close = false; // ego car is very close to car in front
bool car_l_b = false; //Car left lane, behind ego
bool car_l_f = false; //Car left lane, in front of ego
bool car_r_b = false; //Car right lane, behind ego
bool car_r_f = false; //Car right lane, in front of ego
bool lane_change_active = false; //car is doing lane change or not
```
To do so, each surrounding car's frenet coordinates are evaluated. Each car is assigned to its driving lane. Cars which are in a range of 27m ahead of the ego car are flagged as 'too_close', cars on the left lane, which are in a "unsafe range" in front or behind the ego car are flagged as well. The same happens with the right lane. The "unsafe range" is inversely proportional to the actual velocity of the ego car. The faster the ego car drives, the shorter the unsafe range is. The idea behind this is that when the ego car drives relatively slow, fast cars could approach from behind. To avoid crashes, the "unsafe range" has to be larger then when the ego car drives at 50 mph.

#### Far range evaluation
The second and more sophisticated goal is to evalute the traffic far ahead of the ego car to make smart lane decisions. To do so, I implemented a "Fast Lane Detector".
All cars in a range of 300m to 50m in front of the ego car are evaluated. The slowest car of each lane sets the virtual speed limit of that lane. The lane with the highest virtual speed limit is flagged as 'fast_lane'.

### Behavior Planning

The behavior planner consists of different rules which evaluate the above mentioned flags and create approbiate behaviors for the car.
The lane and speed are adjusted accordingly.

#### Rules for crash avoidance
1. If a car is very close in front and there are no cars on the left side, change lane to the left.
2. If a car is very close in front and there are no cars on the right side, change lane to the right.
3. If a car is very close in front and there are cars on the left or right side, emergency brake!.
4. If a car is getting close in front there are cars on the left or right side, reduce own velocity to velocity of car in front.

#### Rules for German Highway behavior
One rule on german highways is too drive on the right lane as often as possible:

5. If right lane is empty, change lange to the right.

#### Rules to choose Fast Lane 
6. If left lane is empty and left lane is declared as fast lane, change lane to the left.
7. If right lane is empty and right lane is declared as fast lane, change lane to the right.
8. Reducing speed if 'Fast_Lane' is blocked by slow cars on the middle lane. The chance to overtake the slow cars is increased by this maneuver.
9. Accelerate in all other cases.

**At this point, the rest follows the [Udacity Self-driving car Q&A session.](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d) very closely. I will summarize it below:

1. With the lane and speed of the vehicle decided, the car creates a trajectory of 50 points to follow. The points are created by using the previous paths last two points, and then 3 additional points allong the road seperated by 30 m. [lines 339-397]

2. The final waypoints take the remaining points from the last trajectory + new points created from a spline of the 5 aformentioned points. Note, that not necessarily all of the new points will be added. Points will only be added until there is a total of 50 points in the current cycle waypoint list. [lines 398-440]

3. Path smoothing uses the spline library [spline.h](http://kluge.in-chemnitz.de/opensource/spline/).

4. When creating the 5 points for the spline, I changed to fernet coordinates relative to the car's local reference frame for simplicity. I tranform back into a global cartesian reference frame in the end. [lines 388-397, 440-440]

---

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

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

