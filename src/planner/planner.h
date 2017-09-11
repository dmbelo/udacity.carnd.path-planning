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

    double v_target = 48 * 1.6 / 3.6;

    int lane_target = 2;

    int bIsLaneChange = false;
    
    double g_max = 2;

    double s_buffer = 20;

    double s_collision = 5; // Distance b/w vehicle origins to trigger collision

    int n_votes_threshold = 25;

    Planner();

    virtual ~Planner();

    void UpdateState();

    void ResetVotes();

    void RealizeState(string state);

    // void RealizeConstantSpeed();

    void RealizeKeepLane();

    void RealizeLaneChange(string direction);

    // void RealizePrepLaneChange(string flag);
    
    double GetMaxAccel();

    double GetCollisionCost();

    double GetTargetSpeedCost();

    double GetChangeStateCost(string test_state);

    double GetRoadBoundaryCost();

private:

    // vector<string> states = {"KL", "LCL", "LCR", "PLCL", "PLCR"};
    vector<string> states = {"KL", "LCL", "LCR"};

    int n_states = 3;

};      

#endif