#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>

using namespace std;

class Vehicle
{

public:

    // Vehicle states
    int id;
    int l;
    double s;
    double v;
    double g;

    Vehicle();
    Vehicle(int id, int lane, double s, double v, double g);

    virtual ~Vehicle();

    void Simulate(double t);

    void Print();

};      

#endif