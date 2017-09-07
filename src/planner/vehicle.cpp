#include "vehicle.h"

Vehicle::Vehicle()
{
    this->id = 0;
    this->l = 0;
    this->s = 0;
    this->v = 0;
    this->g = 0;
}

Vehicle::Vehicle(int id, int lane, double s, double v, double g)
{
    this->id = id;
    this->l = lane;
    this->s = s;
    this->v = v;
    this->g = g;
}

Vehicle::~Vehicle(){};

void Vehicle::Simulate(double t)
{
    this->s += this->v * t + this->g * t * t / 2;
    this->v += this->g * t;
}

void Vehicle::Print()
{
    cout << "Vehicle ID\t" << this->id << endl;
    cout << "-----------" << endl;
    cout << "Lane\t\t" << this->l << endl;
    cout << "Station\t\t" << this->s << endl;
    cout << "Velocity\t" << this->v << endl;
    cout << "Acceleration\t" << this->g << endl;
}