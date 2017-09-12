#include "planner.h"

Planner::Planner(){
    this->state = "KL";
    ResetVotes();
}

Planner::~Planner(){}

void Planner::UpdateState()
{

    vector<double> cost_vec;

    int lane_current = this->road.ego.l;

    cout << "TestState\tCollision\tTargetSpeed\tChangeState\tRoadBoundary\tTotal" << endl;

    for (string test_state : this->states)
    {
        this->bIsLaneChange = false;
        RealizeState(test_state);
        this->road.Simulate(5);
        double cost_collision = GetCollisionCost();
        double cost_target_speed = GetTargetSpeedCost();
        double cost_change_state = GetChangeStateCost(test_state);
        double cost_road_boundary = GetRoadBoundaryCost();
        double cost = cost_collision + cost_target_speed + cost_change_state + cost_road_boundary;
        cost_vec.push_back(cost);
        this->road.Reset();

        cout << test_state << "\t\t"
             << setprecision(4) << cost_collision << "\t\t" << setprecision(4) << cost_target_speed << "\t\t" 
             << setprecision(4) << cost_change_state << "\t\t" << setprecision(4) << cost_road_boundary << "\t\t" 
             << setprecision(4) << cost << endl;

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

    string state_requested = states[idx_min_cost];
    string state_arbitrated;

    // State Machine

    if (this->state.compare("KL") == 0) // State KL
    {
        state_arbitrated = "KL";
        this->votes[idx_min_cost] += 1;
        cout << "Votes: " << this->votes[0] << ", " << this->votes[1] << ", " << this->votes[2] << endl;
        if (this->votes[idx_min_cost] > this->n_votes_threshold)
        {
            ResetVotes();
            state_arbitrated = state_requested;
        }
    }
    else  // State LCL/LCR
    {
        state_arbitrated = this->state;
        if ((state_requested.compare(this->state) == 0) | (state_requested.compare("KL") == 0))
        {   
            state_arbitrated = state_requested;
        }
    }

    this->state = state_arbitrated;
    RealizeState(state_arbitrated);

}

void Planner::ResetVotes()
{
    this->votes = {0, 0, 0};
}

void Planner::RealizeState(string state)
{
    // Given a state, realize it by adjusting acceleration and lane.
    // Note - lane changes happen instantaneously.

    if(state.compare("KL") == 0)
    {
    	RealizeKeepLane();
    }
    else if(state.compare("LCL") == 0)
    {
    	RealizeLaneChange("L");
    }
    else if(state.compare("LCR") == 0)
    {
    	RealizeLaneChange("R");
    }

}

void Planner::RealizeKeepLane()
{
    this->road.ego.g = GetMaxAccel();
}

void Planner::RealizeLaneChange(string direction)
{
    if (!this->bIsLaneChange)
    {
        this->bIsLaneChange = true;
        if (direction.compare("L") == 0)
        {
            this->lane_target -= 1;
            this->road.ego.l -= 1;  
        }
        else
        {
            this->lane_target += 1;
            this->road.ego.l += 1;
        }
    }
    else if (this->lane_target != this->road.ego.l)
    {
        this->bIsLaneChange = false;
    }
    
    this->road.ego.g = GetMaxAccel();
}

double Planner::GetMaxAccel()
{
    double v_delta = this->v_target - this->road.ego.v;
    double g_max = v_delta; // assuming 1 sec integration time

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

    // Limit accel if approaching vehicle in front
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

        double s_lead_next = vehicle_lead.s + vehicle_lead.v;
        double s_ego_next = s_ego + this->road.ego.v;

        double s_delta_next = s_lead_next - s_ego_next;
        double s_available = s_delta_next - this->s_buffer;

        g_max = min(g_max, s_available); // assuming 1 sec integration time

    }

    // Limit accel if changing lane
    if (this->bIsLaneChange)
    {
        g_max = max(-this->g_max * 0.2, g_max);
        g_max = min(this->g_max * 0.2, g_max);
    }
    else
    {
        g_max = max(-this->g_max, g_max);
        g_max = min(this->g_max, g_max);
    }

    return g_max;

}

double Planner::GetCollisionCost()
{

    // TODO Do a better job making sure there is room behind as well as in front
    // TODO Do a better job of comparing if there will be a collision immeniantly or just in the future

    double cost = 0;
    double s_ego = this->road.ego.s;
    int l_ego = this->road.ego.l;

    // Detect collision
    for (const auto &vehicle : this->road.traffic)
    {
        if ((vehicle.l == l_ego) && (abs(vehicle.s - s_ego) <= this->s_collision))
        {
            cost = 1000;
        }
    }

    return cost;

}

double Planner::GetTargetSpeedCost()
{
    return abs(this->v_target - this->road.ego.v);
}

double Planner::GetChangeStateCost(string test_state)
{
    double cost = 0;
    if (test_state.compare(this->state) != 0)
    {
        if (abs(this->road.ego.g) > 0.85 * this->g_max)
        {
            cout << "g = " << this->road.ego.g << endl;
            cost = 100;
        }
        else
        {
            cost = 0.5;
        }
        
    }
    return cost;
}

double Planner::GetRoadBoundaryCost()
{
    double cost = 0;
    if ((this->road.ego.l < 1) || (this->road.ego.l > 3))
    {
        cost = 1e4;
    }
    return cost;
}