# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We are provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3. 

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Implementation

We are provided with 'main.cpp' file that contains a number of helper functions such as:
distance - calculates the distance between two points
ClosestWayPoint - Picks a waypoint closest to the car from a map of waypoints
NextWayPoint - Picks a waypoint that is closest to the car and is infront of the car
getFrenet - Tranforms the cartesian coordinates of the car into frenet coordinates that make the path planning easier
getXY - Includes non-linear transformation of frenet coordinates to cartersian coordinates. This function is inverse of getFrenet.

In the main function, highwaymap.csv file is loaded to get the localization data that includes our car's cartesian co-ordinates, Frenet co-ordinates, speed and waypoints. It also contains sensor fusion data that provides position information of other cars. Simulator receives path planning data in the form of a list of 50 points containing 'next_x_vals' and 'next_y_vals' to be followed by the car. 

In order to plan a path, we should first predict what the other cars in our field of view are doing. This can be known using telemetry and sensor fusion data. In each simulator cycle, we iterate over sensor fusion data from each car and determine their lane of travel. Each lane is 4 meters wide and origin of 'd' coordinate is located at the median of the road. On the right side of median line, the road is further divided into 3 lanes, where the left most is assumed to be lane 0, with the right most being lane 2. For example, if the 'd' coordinate of a car from lane 0 lies between 0 and 4, it belongs to lane 0. 

Once the car's lane is known, we are interested to know if the car is in our lane, to the left or to the right. The cars that effect out path planning are the ones that are closer to us. So, we are particularly interested in the cars that are within 30m distance ahead of us in all lanes and within 20m distance in left and right lanes. In the current lane, the position information of the cars that are behind us is not very important. However, the cars that are behind our car in left and right lanes are to be taken into account as they can effect our car's behavior while preparing for a lane change.

The next step is to determine the behavior of our car (change in velocity & lanes) according to the prediction information from sensor fusion data. If there is a car ahead of us in the current lane and is within the range specified above, we could either make a lane change or stay in the current lane and change our speed according to the speed of the car ahead of us. Also it is found from some experimentation that changing lanes at velocities close to 50 is not violating the constraints of jerk and acceleration. So, I have decided to perform lane change first if possible otherwise I would stay in the current lane and change my car speed. Also, making the lane change at near speed limit velocity ensures that we can reach the goal in lesser time as long as the constraints are not violated. If the vehicle ahead of us is moving at a speed lower than our car, we decrease the speed of our car. If not, we will accelerate till our speed matches the speed of the car ahead of us. Also, if the car ahead is very slow, we decelerate at a higher rate. In case, there is no car in front of us, we try to maintain the speed at near 50MPH.

After behavior planning, we have the reference speed which the car should follow and the lane in which the car should stay. This information along with the previous path points data is utilized to generate trajectory. This is done using spline function, which is coded in 'spline.h' header spline. To generate a smooth spline curve, we utilize 5 points, of which 2 are the previous paths end points and other 3 are the points located at distances of 30m, 60m and 90m ahead of the current car location along s-coordinate. In order to get the XY coordinates of those 3 points ahead of us, getXY function along with lane information is used. Equally spaced co-ordinates for a distance of 30m are obtained from the car's reference velocity. Finally, the 50 points that describe the car's trajectory are generated using the left over points from previous trajectory and the co-ordinates from spline curve. Using coordinates from previous points ensures that there is a smooth transition from cycle to cycle. 

The car is able to succesfully travel for 4.32 miles without any collisions and stays within the limits of acceleration and jerk.
