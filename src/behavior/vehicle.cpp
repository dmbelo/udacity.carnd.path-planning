#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <iomanip>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    state = "CS";
    max_acceleration = -1;

}

Vehicle::~Vehicle() {}

// TODO - Implement this method.
void Vehicle::update_state(map<int,vector < vector<int> > > predictions) {
	/*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */

    // state = "KL"; // this is an example of how you change state.

    double target_lane = this->goal_lane;
    double target_s = this->goal_s;

    vector<string> states = {"KL", "LCL", "LCR", "PLCL", "PLCR"};
    vector<double> costs;
    double cost;
    double cost_collision;
    double cost_target_speed;
    double cost_target_lane;
    double cost_target_end_lane;
    double cost_lane_boundaries;

    cout << "#################################" << endl;
    cout << "Cost Functions" << endl;
    cout << "#################################" << endl;
    cout << "State\tCollision\tSpeed\tLane\tEndLane\tLaneBoundaries\tTotal" << endl; 

    for (string test_state : states)
    {

        cost_collision = 0;
        cost_target_speed = 0;
        cost_target_lane = 0;
        cost_target_end_lane = 0;
        cost_lane_boundaries = 0;

        // Create copy of vehicle
        Vehicle test_v = Vehicle(this->lane, this->s, this->v, this->a);
        test_v.target_speed = this->target_speed;        
        test_v.state = test_state;
        test_v.realize_state(predictions);

        // Predict one step into future, for selected state
        vector<int> test_v_state = test_v.state_at(1);
        int pred_lane = test_v_state[0];
        int pred_s = test_v_state[1];
        int pred_v = test_v_state[2];
        int pred_a = test_v_state[3];

        // cout << "pred_lane, pred_s, pred_v, pred_a: " << pred_lane << ", " 
            //  << pred_s << ", " << pred_v << ", " << pred_a << endl;

        // Check for collisions
        map<int, vector<vector<int>>> :: iterator it = predictions.begin();
        vector<vector<vector<int>>> in_front;
        for (const auto &pred : predictions)
        {
            int index = pred.first;
            vector<vector<int>> v = pred.second;
            // Check predictions one step in future
            if ((v[1][0] == (double) pred_lane) && (abs(v[1][1] - (double) pred_s) <= L) && index != -1)
            {
                // cout << "Collision w/ car:" << index << ", "
                    //  << v[1][0] << " " << pred_lane << ", " 
                    //  << v[1][1] << " " << pred_s << endl;
                cost_collision += 1000;
            }
        }

        // cout << "this->target_speed: " << this->target_speed << endl;
        // cout << "pred_v: " << pred_v << endl;
        cost_target_speed = 1 * abs(this->target_speed - (double) pred_v);
        cost_target_lane = 1 * pow(target_lane - (double) pred_lane, 2);
        cost_target_end_lane = 10 * (1 - exp(-abs((double) pred_lane - target_lane)/(target_s - (double) pred_s)));
        
        if (pred_lane < 0 || pred_lane > 3) {
            cost_lane_boundaries = 1000;
        }

        double cost = cost_collision + cost_target_speed + cost_target_lane + 
        cost_target_end_lane + cost_lane_boundaries;

        costs.push_back(cost);

        cout << test_state << "\t" 
             << setprecision(4) << cost_collision << "\t\t"
             << setprecision(4) << cost_target_speed << "\t"
             << setprecision(4) << cost_target_lane << "\t"
             << setprecision(4) << cost_target_end_lane <<"\t"
             << setprecision(4) << cost_lane_boundaries << "\t\t"
             << setprecision(4) << cost << endl;

    }

    double min_cost = 1e6;
    int min_cost_index = 0;

    for (int i = 0; i < costs.size(); i++)
    {
        if (costs[i] < min_cost)
        {
            min_cost = costs[i];
            min_cost_index = i;
        }
    }

    this->state = states[min_cost_index];
    // cout << "State: " << this->state << endl;

}

void Vehicle::configure(vector<int> road_data) {
	/*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    this->target_speed     = road_data[0];
    this->lanes_available  = road_data[1];
    this->goal_s           = road_data[2];
    this->goal_lane        = road_data[3];
    this->max_acceleration = road_data[4];
}

string Vehicle::display() {

	ostringstream oss;
	
	oss << "s:    " << this->s << "\n";
    oss << "lane: " << this->lane << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";
    
    return oss.str();
}

void Vehicle::increment(int dt = 1) {

	this->s += this->v * dt;
    this->v += this->a * dt;
}

vector<int> Vehicle::state_at(int t) {

	/*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    int s = this->s + this->v * t + this->a * t * t / 2;
    int v = this->v + this->a * t;
    return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

	/*
    Simple collision detection.
    */
    vector<int> check1 = state_at(at_time);
    vector<int> check2 = other.state_at(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1; 

	for (int t = 0; t < timesteps+1; t++)
	{
      	if( collides_with(other, t) )
      	{
			collider_temp.collision = true;
			collider_temp.time = t; 
        	return collider_temp;
    	}
	}

	return collider_temp;
}

void Vehicle::realize_state(map<int,vector < vector<int> > > predictions) {
   
	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if(state.compare("CS") == 0)
    {
    	realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R");
    }
    else if(state.compare("PLCL") == 0)
    {
    	realize_prep_lane_change(predictions, "L");
    }
    else if(state.compare("PLCR") == 0)
    {
    	realize_prep_lane_change(predictions, "R");
    }

}

void Vehicle::realize_constant_speed() {
	a = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s) {

	int delta_v_til_target = target_speed - v;
    int max_acc = min(max_acceleration, delta_v_til_target);

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > in_front;
    while(it != predictions.end())
    {
       
    	int v_id = it->first;
    	
        vector<vector<int> > v = it->second;
        
        if((v[0][0] == lane) && (v[0][1] > s))
        {
        	in_front.push_back(v);

        }
        it++;
    }
    
    if(in_front.size() > 0)
    {
    	int min_s = 1000;
    	vector<vector<int>> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][0][1]-s) < min_s)
    		{
    			min_s = (in_front[i][0][1]-s);
    			leading = in_front[i];
    		}
    	}
    	
    	int next_pos = leading[1][1];
    	int my_next = s + this->v;
    	int separation_next = next_pos - my_next;
    	int available_room = separation_next - preferred_buffer;
    	max_acc = min(max_acc, available_room);
    }
    
    return max_acc;

}

void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions) {
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int,vector<vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    int lane = this->lane + delta;

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > at_behind;
    while(it != predictions.end())
    {
    	int v_id = it->first;
        vector<vector<int> > v = it->second;

        if((v[0][0] == lane) && (v[0][1] <= this->s))
        {
        	at_behind.push_back(v);

        }
        it++;
    }
    if(at_behind.size() > 0)
    {

    	int max_s = -1000;
    	vector<vector<int> > nearest_behind = {};
    	for(int i = 0; i < at_behind.size(); i++)
    	{
    		if((at_behind[i][0][1]) > max_s)
    		{
    			max_s = at_behind[i][0][1];
    			nearest_behind = at_behind[i];
    		}
    	}
    	int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    	int delta_v = this->v - target_vel;
    	int delta_s = this->s - nearest_behind[0][1];
    	if(delta_v != 0)
    	{

    		int time = -2 * delta_s/delta_v;
    		int a;
    		if (time == 0)
    		{
    			a = this->a;
    		}
    		else
    		{
    			a = delta_v/time;
    		}
    		if(a > this->max_acceleration)
    		{
    			a = this->max_acceleration;
    		}
    		if(a < -this->max_acceleration)
    		{
    			a = -this->max_acceleration;
    		}
    		this->a = a;
    	}
    	else
    	{
    		int my_min_acc = max(-this->max_acceleration,-delta_s);
    		this->a = my_min_acc;
    	}

    }

}

vector<vector<int> > Vehicle::generate_predictions(int horizon = 10) {

	vector<vector<int> > predictions;
    for( int i = 0; i < horizon; i++)
    {
      vector<int> check1 = state_at(i);
      vector<int> lane_s = {check1[0], check1[1]};
      predictions.push_back(lane_s);
  	}
    return predictions;

}

// void Vehicle::GenerateTrajectoryForState(map<int, vector<vector<int>>> predictions, int horizon)
// {

//     Vehicle test_v(this->lane, this->s, this->v, this->a);

//     for (int i = 0; i < horizon; i++)
//     {
//         test_v.realize_state(predictions);
//         test_v.increment();
//     }

//     vector<int> test_v_state = test_v.state_at(1);
//     int pred_lane = test_v_state[0];
//     int pred_s = test_v_state[1];
//     int pred_v = test_v_state[2];
//     int pred_a = test_v_state[3];
    
// }