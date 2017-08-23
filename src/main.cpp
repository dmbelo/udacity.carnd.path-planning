#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = abs(theta - heading);

	if (angle > pi() / 4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
		prev_wp = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x, y};
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

	h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
																											 uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
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

					// Unit convert
					v_car = v_car * 1.6 / 3.6; // m/s

					double v_car_nominal = 50 * 1.6 / 3.6;					
					double v_car_target = v_car_nominal;

					// Previous path data given to the Planner
					auto x_trajectory_unused = j[1]["previous_path_x"];
					auto y_trajectory_unused = j[1]["previous_path_y"];
					// Previous path's end s and d values
					double s_end_trajectory_unused = j[1]["end_path_s"];
					double d_end_trajectory_unused = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					double s_follow_min = 30;
					double s_follow = 1e6;

					int n_vehicles = sensor_fusion.size();

					for (int i = 0; i < n_vehicles; i++) {
						
						double id  = sensor_fusion[i][0]; // car's unique ID
						double xi  = sensor_fusion[i][1]; // car's x position in map coordinates
						double yi  = sensor_fusion[i][2]; // car's y position in map coordinates
						double vxi = sensor_fusion[i][3]; // car's x velocity in m/s
						double vyi = sensor_fusion[i][4]; // car's y velocity in m/s
						double si  = sensor_fusion[i][5]; // car's s position in frenet coordinates,
						double di  = sensor_fusion[i][6]; // car's d position in frenet coordinates.
						double vi  = sqrt(vxi*vxi + vyi*vyi);
					

						// TODO 
						// Implement the following with a nice hypertangent function or 
						// something similiar to have a smooth control actuation
						// Find closest vehicle in front of us

						// TODO Implement the ability to limit the trajectory based on accelation and jerk

						// TODO Start the path from scratch if you have an abrupt change in speed (
						// like a vehicle crossing infront of you)
						 
						// Nominal vs. target speed and nominal vs. target lane? 


						if ((si > s_car) & (di > 4) & (di < 8)) { // There's a car in front of us
							cout << "There's a car " << (si - s_car) << " in front of us" << endl;
							if ((si - s_car) < s_follow) {
								s_follow  = si - s_car;

								if ((s_follow < s_follow_min) & (vi < v_car_target)) {
									cout << "s_follow, vi, vxi , vyi: " << s_follow << ", " << vi << ", " << vxi << ", " << vyi << endl;
									v_car_target = vi * 0.95;
								} else {
									v_car_target = v_car_nominal;
								}
							}
						}


				

						
					}

					json msgJson;

					// Spline object for interpolation
					tk::spline s;

					// trajectory vector to be generated
					vector<double> x_trajectory;
					vector<double> y_trajectory;

					// spline nodes
					vector<double> x_spline;
					vector<double> y_spline;

					// initial spline vehicle position & orientation
					double x_, y_, x0, y0, a0, s0;

					int n_trajectory_unused = x_trajectory_unused.size();

					if (n_trajectory_unused > 0)
					{

						x_ = x_trajectory_unused[n_trajectory_unused - 2];
						y_ = y_trajectory_unused[n_trajectory_unused - 2];
						x0 = x_trajectory_unused[n_trajectory_unused - 1];
						y0 = y_trajectory_unused[n_trajectory_unused - 1];
						s0 = s_end_trajectory_unused;
						a0 = atan2(y0 - y_, x0 - x_);
					}
					else
					{

						x_ = x_car - cos(a_yaw_car);
						y_ = y_car - sin(a_yaw_car);
						x0 = x_car;
						y0 = y_car;
						s0 = s_car;
						a0 = a_yaw_car;
					}

					// Add two points as the beginning of the spline in order to set
					// a smooth boundary slope for the spline
					x_spline.push_back(x_);
					y_spline.push_back(y_);
					x_spline.push_back(x0);
					y_spline.push_back(y0);

					vector<double> xy_car_30 = getXY(s0 + 30, 6.0, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> xy_car_60 = getXY(s0 + 60, 6.0, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> xy_car_90 = getXY(s0 + 90, 6.0, map_waypoints_s, map_waypoints_x, map_waypoints_y);

					x_spline.push_back(xy_car_30[0]);
					y_spline.push_back(xy_car_30[1]);
					x_spline.push_back(xy_car_60[0]);
					y_spline.push_back(xy_car_60[1]);
					x_spline.push_back(xy_car_90[0]);
					y_spline.push_back(xy_car_90[1]);

					// Convert x,y_spline from world to vehicle reference frame
					for (int i = 0; i < x_spline.size(); i++)
					{

						double dx = x_spline[i] - x0;
						double dy = y_spline[i] - y0;
						x_spline[i] = dx * cos(a0) + dy * sin(a0);
						y_spline[i] = -dx * sin(a0) + dy * cos(a0);
					}

					s.set_points(x_spline, y_spline);

					// Copy over unused trajectory to new generated one
					for (int i = 0; i < n_trajectory_unused; i++)
					{

						x_trajectory.push_back(x_trajectory_unused[i]);
						y_trajectory.push_back(y_trajectory_unused[i]);
					}

					double xi = 0;
					double yi = 0;
					double dt = 0.02;

					double ds = v_car_target * dt;

					for (int i = 0; i < 51 - n_trajectory_unused; i++)
					{

						xi += ds;
						yi = s(xi);

						// Convert from vehicle to world reference frame and append
						x_trajectory.push_back(xi * cos(a0) - yi * sin(a0) + x0);
						y_trajectory.push_back(xi * sin(a0) + yi * cos(a0) + y0);
					}

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
		ws.close();
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
