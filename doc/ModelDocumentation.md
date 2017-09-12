# Path Planning Model Documentation

## Introduction

The path planning application consists primarily as a set of classes that are used by the main funcion in the `main.cpp` file. These classes are:

|Class|Description|Relevant Directory|
|---|---|---|
|Planner|Calculates the target lane and velocity for the ego vehicle given the current ego vehicle pose and road traffic pose|src/planner|
|TrajectoryGenerator|Generates a trajectory in the form of x, y coordinates for the simulator to follow based on current vehicle post and target lane and velocity|src/trajectory|

## Planner Class Description

The planner class implements the main functionality responsible for determinine the target lane and velocity give then ego vehicle and road traffic pose information. This is done by implementing a simple state machine consisting on 3 states: Keep Lane (KL), Lane Change Left (LCL) and Lane Change Right (LCR).

The state machine is implemented in the `UpdateState` method and consists of simple rules that determing the transition functions for each state. Of note is the fact that the transition out of the KL state is done by going to vote where in order to transition to LCL or LCR a threshold of votes needs to be crossed in order for the transition to be realized. This helps the stability and robustness of the state transitions ensuring that states are overly sensitiive to noisy features, short transients, etc.

The state machine is in effectively arbitrating what state the planner should be given the given currnet conditions, the current state, and the state requested. The state requested represents the state that minimizes a cost function. The cost function itself is made up of various individual cost functions that are tuned to achieve the the path planning speed, acceleration, jerk and collision avoidance targets. These are all implemented in corresponding class methods, e.g. `GetCollisionCost`, `GetTargetSpeed`, etc.

The `GetMaxAccel` method is also an important one that is responsible for determine the appropriate vehicle longitudinal acceleration input given a state. This is ultimatly the responsible method for controlling the logitudinal behavior of the selected state. Like the rest of the code, it is implemented in a simple fashion whereby the acceleration by a variety of thresholds including hardcode nominal max/min, slower vehicles in the way and lane change limits. The latter lane change limits is implemented as hard coded max/min while a lane change is in progress. This is a simple thresholded longitudinal acceleration intended to limit the resultant acceleration given the fact that we expect an induced lateral acceleration from the lane change.

## Trajectory Generator Description

## Main File

