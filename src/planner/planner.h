#ifndef PLANNER_H
#define PLANNER_H

#include "road.h"
#include "vehicle.h"

using namespace std;

class Planner
{

public:

    Road road;

    string state;

    vector<int> votes;

    double v_target = 49 * 1.6 / 3.6;
    
    double g_max = 2;

    double s_buffer = 20;

    int n_votes_threshold = 10;

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

private:

    // vector<string> states = {"KL", "LCL", "LCR", "PLCL", "PLCR"};
    vector<string> states = {"KL", "LCL", "LCR"};

    int n_states = 3;

};      

#endif