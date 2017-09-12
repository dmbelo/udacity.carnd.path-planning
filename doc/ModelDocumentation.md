# Path Planning Model Documentation

## Introduction

The path planning application consists primarily as a set of classes that are used by the main funcion in the `main.cpp` file. These classes are:

|Class|Description|Relevant Directory|
|---|---|---|
|Planner|Calculates the target lane and velocity for the ego vehicle given the current ego vehicle pose and road traffic pose|src/planner|
|TrajectoryGenerator|Generates a trajectory in the form of x, y coordinates for the simulator to follow based on current vehicle post and target lane and velocity|src/trajectory|

## Planner Class Description

The `Planner` class implements the main functionality responsible for determinine the target lane and velocity give then ego vehicle and road traffic pose information. This is done by implementing a simple state machine consisting on 3 states: Keep Lane (KL), Lane Change Left (LCL) and Lane Change Right (LCR).

The state machine is implemented in the `UpdateState` method and consists of simple rules that determing the transition functions for each state. Of note is the fact that the transition out of the KL state is done by going to vote where in order to transition to LCL or LCR a threshold of votes needs to be crossed in order for the transition to be realized. This helps the stability and robustness of the state transitions ensuring that states are overly sensitiive to noisy features, short transients, etc.

The state machine is in effectively arbitrating what state the planner should be given the given currnet conditions, the current state, and the state requested. The state requested represents the state that minimizes a cost function. The cost function itself is made up of various individual cost functions that are tuned to achieve the the path planning speed, acceleration, jerk and collision avoidance targets. These are all implemented in corresponding class methods, e.g. `GetCollisionCost`, `GetTargetSpeed`, etc.

The `GetMaxAccel` method is also an important one that is responsible for determine the appropriate vehicle longitudinal acceleration input given a state. This is ultimatly the responsible method for controlling the logitudinal behavior of the selected state. Like the rest of the code, it is implemented in a simple fashion whereby the acceleration by a variety of thresholds including hardcode nominal max/min, slower vehicles in the way and lane change limits. The latter lane change limits is implemented as hard coded max/min while a lane change is in progress. This is a simple thresholded longitudinal acceleration intended to limit the resultant acceleration given the fact that we expect an induced lateral acceleration from the lane change.

## Trajectory Generator Description

The `TrajectoryGenerator` generates the trajectory for the simulator to follow. It is formatted specifically for the Udacity Simulator which includes spacing the waypoints of the trajectory deliberately in order to achieve a desired vehicle speed.

The classes uses the `tk::spline` class in order to create smooth splines from which to sample the trajectories. The splines are constructed by carefully picking anchor points from the road map data given current vechicle pose. Care must be taken in order to make sure the starting and ending conditions of the spline ensure a smooth transition to ensure contiunity as new trajectories are generated in sequence. E.g. the spline anchors include one point behing the current position in order to constrain the starting slope of the spline to be aligned with current vehicle orientation.

The main interface `Generate` method appends its generated trajectory the output vectors, `x_trajectory` and `y_trajectory` which in order to keep the implementation simple does not assume these vectors to be empty: if the vectors are empty it creates the trajectory from scatch, if it is not, it appends to the end of the partial trajectory supplied while ensuring continuity and a fixed number of samples in the trajectory.

## Main File

The main files makes use of the `Planner` and `TrajectoryGenerator` class described above. It does so by instantiating the `planner` and `traj` objects, accordingly. These objects are instantiated before the web socket to the Simulator is instatiated and passed by reference to the web socket `onMessage` method.

At each communcation step, the path planning code can be effectively distilled to a few calls to the `planner` and `traj` objects. 

```
vector<double> ego_vec = {0, d_car, s_car, v_car, 0.0};
planner.road.Populate(ego_vec, sensor_fusion);
planner.UpdateState();
lane_target = planner.lane_target;
v_car_target = v_car + planner.road.ego.g;

traj.SetInitialPose(x_car, y_car, a_yaw_car);
traj.SetTargetSpeed(v_car_target);
traj.SetTargetLane(lane_target);
traj.Generate(x_trajectory, y_trajectory);					
```

The first block of code consists of first informing the `planner` object with the latest ego vhehicle and road traffic pose information received from the Simulator and Sensor Fustion module. Next, the `UpdateState` method is called to operate on the latest information. Finally, the resulting target lane and velocity is retrieved.

The second block consists of taking the output from the `planner` and generating the appropriate trajectory for the Simulator. We start by informing the `traj` object about the current pose, and targets. Next we call the `Generate` method to populate the `x_trajectory` and `y_trajectory` vectors which contain the trajectory to be sent to the Simulator. Note that these trajectorys may not be empty, and indeed, for the most part they are not as we carry over the incomplete trajectory from the previous step when the Simulator is enable to process all trajectory nodes that were sent in the previous cycle. 


