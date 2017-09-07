#ifndef ROAD_H
#define ROAD_H

#include <iostream>
#include <vector>
#include <cmath>
#include "vehicle.h"

using namespace std;

class Road
{

public:

    Vehicle ego;
    vector<Vehicle> traffic;

    Road(vector<double> ego_state, vector<vector<double>> traffic_state);

    virtual ~Road();

    void Initialize();

    void Reset();

    void Simulate(double t);

    void Print();

private:

    vector<double> ego_vec;
    vector<vector<double>> traffic_vec;

};      

#endif