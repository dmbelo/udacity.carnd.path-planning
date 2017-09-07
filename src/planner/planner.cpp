#include "planner.h"

Planner::Planner(){
    this->state = "KL";
    ResetVotes();
}

Planner::~Planner(){}

void Planner::UpdateState()
{

    vector<double> cost_vec;

    for (string test_state : this->states)
    {
        this->road.Reset();
        // this->RealizeState(state)
        this->road.Simulate(10);

        // cost_vec.push_back(cost);
    }

    double min_cost = 1e10;
    int idx_min_cost = 0;
    for (int i = 0; i < this->n_states; i++)
    {
        if (cost_vec[i] < min_cost)
        {
            min_cost = cost_vec[i];
            idx_min_cost = i;
        }
    }

    // Increment the vote count
    this->votes[idx_min_cost] += 1;

    // Check weather we've exceeded the threshold change state if so
    if (this->votes[idx_min_cost] > n_votes_threshold)
    {
        this->state = states[idx_min_cost];
        ResetVotes();
    }

}

void Planner::ResetVotes()
{
    this->votes = {0, 0};
}

void Planner::RealizeState(string state)
{
    // Given a state, realize it by adjusting acceleration and lane.
    // Note - lane changes happen instantaneously.

    RealizeKeepLane();
    // if(state.compare("CS") == 0)
    // {
    // 	RealizeConstantSpeed();
    // }
    // else if(state.compare("KL") == 0)
    // {
    // 	RealizeKeepLane();
    // }
    // else if(state.compare("LCL") == 0)
    // {
    // 	RealizeLaneChange("L");
    // }
    // else if(state.compare("LCR") == 0)
    // {
    // 	RealizeLaneChange("R");
    // }
    // else if(state.compare("PLCL") == 0)
    // {
    // 	RealizePrepLaneChange("L");
    // }
    // else if(state.compare("PLCR") == 0)
    // {
    // 	RealizePrepLaneChange("R");
    // }

}

// void RealizeConstantSpeed(){
//     this->road.ego.a = 0;
// }

void Planner::RealizeKeepLane()
{
    this->road.ego.g = GetMaxAccel();
}

double Planner::GetMaxAccel()
{
    double dt = 1; // TODO 
    double v_delta = this->v_target - this->road.ego.v;
    double g_max = min(this->g_max, v_delta); // assuming 1 sec integration time

    double s_ego = this->road.ego.s;
    int l_ego = this->road.ego.l;

    // Filter just the traffic in lane and in front of ego
    vector<Vehicle> traffic_in_front;
    for (auto &vehicle : this->road.traffic)
    {
        if ((vehicle.l == l_ego) && (vehicle.s > s_ego))
        {
            traffic_in_front.push_back(vehicle);
        }
    }

    // Find vehicle to follow
    Vehicle vehicle_lead;
    if (traffic_in_front.size() > 0)
    {
        double s_min = 1e10;
        for (auto &vehicle : traffic_in_front)
        {
            if ((vehicle.s - s_ego) < s_min)
            {
                s_min = vehicle.s - s_ego;
                vehicle_lead = vehicle;
            }
        }
    }

    double s_lead_next = vehicle_lead.s + vehicle_lead.v;
    double s_ego_next = s_ego + this->road.ego.v;

    double s_delta_next = s_lead_next - s_ego_next;
    double s_available = s_delta_next - s_buffer;

    g_max = min(g_max, s_available); // assuming 1 sec integration time

    return g_max;
}


