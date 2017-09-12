#ifndef PLANNER_H
#define PLANNER_H

#include "road.h"
#include "vehicle.h"
#include <iomanip>

using namespace std;

class Planner
{

public:

    Road road;

    string state;

    vector<int> votes;

    double v_target = 48 * 1.6 / 3.6; // m/s

    int lane_target = 2;

    int bIsLaneChange = false; // Is lane change in progress?
    
    double g_max = 1.8; // Max nominal acceleration (+ve/-ve)

    double s_buffer = 30; // Distance to keep to following vehicle

    double s_collision = 15; // Distance b/w vehicle origins to trigger collision

    int n_votes_threshold = 40; // Number of votes before transition to LCL/LCR from KL

    Planner();

    virtual ~Planner();

    void UpdateState();

    void ResetVotes();

    void RealizeState(string state);

    void RealizeKeepLane();

    void RealizeLaneChange(string direction);
    
    double GetMaxAccel();

    double GetCollisionCost();

    double GetTargetSpeedCost();

    double GetChangeStateCost(string test_state);

    double GetRoadBoundaryCost();

private:

    vector<string> states = {"KL", "LCL", "LCR"};

    int n_states = 3;

};      

#endif