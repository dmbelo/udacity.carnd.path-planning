#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "trajectory/trajectory_generator.h"
#include "planner/planner.h"
#include <iomanip>

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos)
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

int main()
{
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line))
	{
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	TrajectoryGenerator traj(map_waypoints_x, map_waypoints_y, map_waypoints_s);
	traj.SetHorizonDistance(30);

	Planner planner;

	int counter = 0;

	h.onMessage([&traj, &planner, &counter](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
																											 uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		// auto sdata = string(data).substr(0, length);
		// cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{

			auto s = hasData(data);

			if (s != "")
			{
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry")
				{
					// j[1] is the data JSON object

					// Main car's localization Data
					double x_car = j[1]["x"];
					double y_car = j[1]["y"];
					double s_car = j[1]["s"];
					double d_car = j[1]["d"];
					double a_yaw_car = j[1]["yaw"]; // rad?
					double v_car = j[1]["speed"];   // mph

					// Unit convert to SI
					v_car = v_car * 1.6 / 3.6; // m/s
					a_yaw_car = a_yaw_car * M_PI / 180.0; // rad

					// Previous path data given to the Planner
					auto x_trajectory_incomplete = j[1]["previous_path_x"];
					auto y_trajectory_incomplete = j[1]["previous_path_y"];
					
					// Previous path's end s and d values
					double s_end_trajectory_incomplete = j[1]["end_path_s"];
					double d_end_trajectory_incomplete = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					/*
					 * Behavior Planning
					 */

					// Update the planner object with the latest pose and sensor fusion data
					// and run the planner 
					vector<double> ego_vec = {0, d_car, s_car, v_car, 0.0};
					planner.road.Populate(ego_vec, sensor_fusion);
					planner.UpdateState();

					// retrieve target lane and speed for trajectory generator
					int lane_target = planner.lane_target;
					double v_car_target = v_car + planner.road.ego.g;
					
					/*
					 * Trajectory Generation
					 */

					// Instantiate trajectory vector to be generated
					vector<double> x_trajectory;
					vector<double> y_trajectory;

					// Limit the number of samples to recycle from the incomplete trajectory 
					// of the previous cycle
					int n_trajectory_unused_max = 15; 
					int n_trajectory_incomplete = x_trajectory_incomplete.size();
					int n_max = min(n_trajectory_incomplete, n_trajectory_unused_max);
					
					if (n_trajectory_incomplete > 2) 
					{
						// Copy over unused trajectory to new generated one
						for (int i = 0; i < n_max; i++)
						{
							x_trajectory.push_back(x_trajectory_incomplete[i]);
							y_trajectory.push_back(y_trajectory_incomplete[i]);
						}
					}

					// Update the vehicle pose and targets to inform the trajectory generation
					traj.SetInitialPose(x_car, y_car, a_yaw_car);
					traj.SetTargetSpeed(v_car_target);
					traj.SetTargetLane(lane_target);
					traj.Generate(x_trajectory, y_trajectory);

					/*
					 * Message
					 */ 

					json msgJson;

					msgJson["next_x"] = x_trajectory;
					msgJson["next_y"] = y_trajectory;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else
			{
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
					   size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
						   char *message, size_t length) {
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
