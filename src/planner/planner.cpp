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

    cout << "TestState\tCollision\tTargetSpeet\tChangeState\tRoadBoundary\tTotal" << endl;

    for (string test_state : this->states)
    {
        RealizeState(test_state);
        this->road.Simulate(2);
        double cost_collision = GetCollisionCost();
        double cost_target_speed = GetTargetSpeedCost();
        double cost_change_state = GetChangeStateCost(test_state);
        double cost_road_boundary = GetRoadBoundaryCost();
        double cost = cost_collision + cost_target_speed + cost_change_state + cost_road_boundary;
        cost_vec.push_back(cost);
        this->road.Reset();
        this->bIsLaneChange = false;

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
        this->votes[idx_min_cost] += 1;
        if (this->votes[idx_min_cost] > this->n_votes_threshold)
        {
            state_arbitrated = state_requested;
            // if (state_arbitrated.compare("LCL") == 0)
            // {
            //     // this->bIsLaneChange = true;
            //     // this->lane_target -= -1;
            // }
            // else if (state_arbitrated.compare("LCR") == 0)
            // {
            //     // this->lane_target += 1;
            // }
            // ResetVotes();
            this->votes[idx_min_cost] = 0;
        }
        else 
        {
            state_arbitrated = "KL";
        }
    }
    else  // State LCL/LCR
    {
        // Check weather lane change is done
        if (!this->bIsLaneChange)
        {
            state_arbitrated = "KL";
        }
        else 
        {
            state_arbitrated = this->state;
        }
    }

    this->state = state_arbitrated;
    RealizeState(state_arbitrated);


    // cout << "Votes: " << this->votes[0] << ", " << this->votes[1] << ", " << this->votes[2] << endl;

    // if (this->state.compare("KL") == 0) // If currently in KL
    // {
    //     // Go to vote
    //     if (this->votes[idx_min_cost] > this->n_votes_threshold)
    //     {
    //         state_arbitrated = state_requested;
    //         ResetVotes();
    //     }
    //     else
    //     {
    //         state_arbitrated = "KL";
    //     }
         
    // }
    // else
    // {
    //     cout << "Current, Desired Lane: " << lane_current << ", " << lane_current << endl; 
       
    //     if (this->bIsLaneChange)
    //     {
    //         // Wait for LC if not done
    //         state_arbitrated = state_requested; 
    //     }
    //     else
    //     {
    //         // Revert to KL if done with LC
    //         state_arbitrated = "KL"; // Revert LCL/LCR to KL 
    //     }

    // }

    // this->state = state_arbitrated;
    // RealizeState(this->state); 

    // string requested_state = states[idx_min_cost];
    // string new_state;

    // this->n_counter += 1;

    // if (this->state.compare("KL") == 0) // If we're in KL go to vote
    // {
    //     // Increment the vote count
    //     this->votes[idx_min_cost] += 1;

    //     // Check weather we've exceeded the threshold change state if so
    //     if (this->votes[idx_min_cost] > n_votes_threshold)
    //     {
    //         new_state = requested_state;
    //     }
    // }
    // else
    // {
    //     new_state = "KL"; // If we're currently not in KL we go to KL
    // }



    // else if (this->n_counter = this->n_lane_change_counter_threshold)
    // // Make sure we don't switch lanes right after another lane change
    // {
    //     new_state = requested_state;
    // }

    // if (requested_state.compare(this->state) != 0) // If there is a state change request
    // {
    //     if ((new_state.compare("LCL") == 0) | (new_state.compare("LCR") == 0))
    //     {
    //         ResetCounter();
    //     }
    //     ResetVotes();
    // }

    

}

void Planner::ResetVotes()
{
    this->votes = {0, 0, 0};
}

void Planner::RealizeState(string state)
{
    // Given a state, realize it by adjusting acceleration and lane.
    // Note - lane changes happen instantaneously.

    // if(state.compare("CS") == 0)
    // {
    // 	RealizeConstantSpeed();
    // }
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
    // else if(state.compare("PLCL") == 0)
    // {
    // 	RealizePrepLaneChange("L");
    // }
    // else if(state.compare("PLCR") == 0)
    // {
    // 	RealizePrepLaneChange("R");
    // }

}

void Planner::RealizeKeepLane()
{
    this->road.ego.g = GetMaxAccel();
}

void Planner::RealizeLaneChange(string direction)
{
    if (!this->bIsLaneChange)
    {
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
    
    // if (!this->bIsLaneChange)
    // {
    //     this->bIsLaneChange = true;
    //     if (this->lane_target != this->road.ego.l)
    //     {
    //         if (direction.compare("L") == 0)
    //         {
    //             this->road.ego.l -= 1;  
    //         }
    //         else
    //         {
    //             this->road.ego.l += 1;
    //         }
    //     }
    //     else
    //     {
    //         this->bIsLaneChange = false;
    //     }
    // }
    
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

    g_max = max(-this->g_max, g_max);
    g_max = min(this->g_max, g_max);

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
        cost = 0.5;
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