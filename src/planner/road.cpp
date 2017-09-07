#include "road.h"

Road::Road(vector<double> ego_vec, vector<vector<double>> traffic_vec)
{
    this->ego_vec = ego_vec;
    this->traffic_vec = traffic_vec;
    Initialize();
}

Road::~Road(){}

void Road::Initialize()
{

    int id = (int) this->ego_vec[0];
    int lane = (int) this->ego_vec[1]; // TODO GetLane
    double s = this->ego_vec[2];
    double v = this->ego_vec[3];
    double g = this->ego_vec[4];

    this->ego = Vehicle(id, lane, s, v, g);

    for (auto &vehicle_vec : this->traffic_vec)
    {
        id = (int) vehicle_vec[0];
        lane = (int) vehicle_vec[6]; // TODO GetLane
        s = vehicle_vec[5];
        double vx = vehicle_vec[3];
        double vy = vehicle_vec[4];
        v = sqrt(vx * vx + vy * vy);
        this->traffic.push_back(Vehicle({id, lane, s, v, 0.0}));
    }

}

void Road::Reset()
{
    // Remove ego and traffic
    Vehicle empty_vehicle;
    this->ego = empty_vehicle;
    vector<Vehicle> empty_traffic;
    this->traffic = empty_traffic;
    Initialize();
}

void Road::Simulate(double t)
{
    // Ego vehicle
    this->ego.Simulate(t);

    // All other vehicles
    for (auto &vehicle : this->traffic)
    {
        vehicle.Simulate(t);
    }
}

void Road::Print()
{
    // Ego vehicle
    this->ego.Print();
    cout << endl;

    // All other vehicles
    for (auto &vehicle : this->traffic)
    {
        vehicle.Print();
        cout << endl;
    }
}