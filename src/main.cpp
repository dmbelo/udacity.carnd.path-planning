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
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
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
  while (getline(in_map_, line)) {
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
		  // j[1] is the data JSON object
		  
			cout << "****************************************" << endl;
          
        	// Main car's localization Data
          	double x_car     = j[1]["x"];
          	double y_car     = j[1]["y"];
          	double s_car     = j[1]["s"];
          	double d_car     = j[1]["d"];
          	double a_yaw_car = j[1]["yaw"]; // rad?
			double v_car     = j[1]["speed"]; // mph
		
			// Unit convert
			v_car = v_car * 1.6 / 3.6; // m/s

          	// Previous path data given to the Planner
          	auto x_trajectory_unused = j[1]["previous_path_x"];
          	auto y_trajectory_unused = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double s_end_trajectory_unused = j[1]["end_path_s"];
          	double d_end_trajectory_unused = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

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

			if (n_trajectory_unused > 0) {

				x_ = x_trajectory_unused[n_trajectory_unused-2];
				y_ = y_trajectory_unused[n_trajectory_unused-2];
				x0 = x_trajectory_unused[n_trajectory_unused-1];
				y0 = y_trajectory_unused[n_trajectory_unused-1];
				s0 = s_end_trajectory_unused;
				a0 = atan2(y0 - y_, x0 - x_);

			} else {
		
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

			vector<double> xy_car_30 = getXY(s0 + 30, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> xy_car_60 = getXY(s0 + 60, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> xy_car_90 = getXY(s0 + 90, 6, map_waypoints_s, map_waypoints_x, map_waypoints_y);

			x_spline.push_back(xy_car_30[0]);
			y_spline.push_back(xy_car_30[1]);
			x_spline.push_back(xy_car_60[0]);
			y_spline.push_back(xy_car_60[1]);
			x_spline.push_back(xy_car_90[0]);
			y_spline.push_back(xy_car_90[1]);

			// Convert x,y_spline from world to vehicle reference frame
			for (int i = 0; i < x_spline.size(); i++) {

				double dx = x_spline[i] - x0;
				double dy = y_spline[i] - y0;
				x_spline[i] = dx * cos(a0) + dy * sin(a0);
				y_spline[i] = -dx * sin(a0) + dy * cos(a0);

			}

			s.set_points(x_spline, y_spline);

			// Copy over unused trajectory to new generated one
			for (int i = 0; i < n_trajectory_unused; i++) {

				x_trajectory[i] = x_trajectory_unused[i];
				y_trajectory[i] = y_trajectory_unused[i];

			}

			double xi = 0;
			double yi = 0;
			double ds = 0.02;

			for (int i = 0; i < 51 - n_trajectory_unused; i++) {

				xi += ds;
				yi = s(xi);
				
				// Convert from vehicle to world reference frame and append
				x_trajectory.push_back(xi * cos(a0) - yi * sin(a0) + x0);
				y_trajectory.push_back(xi * sin(a0) + yi * cos(a0) + y0);

			}
			
			// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

			// double max_accel = 9;
			// double max_jerk = 50;
			// double dt = 0.02;
			// double car_speed_target = 50 * 1.6 / 3.6; // m/s
			// double car_speed_derated = car_speed;
			// double ds;
			// double new_dcar = 6;

			// cout << "Current vehicle position" << endl;
			// cout << "s, d, x, y" << endl;
			// cout << car_s << ", " << car_d << ", " << car_x << ", " << car_y << endl;

			// cout << "Beginning path generation" << endl;

			// unsigned int N = 50;
			// unsigned int N_0 = previous_path_x.size();

			// if (N_0 == 0) {
			// 	cout << "No unused trajector nodes found" << endl;
			// 	end_path_s = car_s;
			// } else {
			// 	// Copy unused tranjectory nodes from previous iteration into this one
			// 	cout << N_0 << " unused trajector nodes found. Copying over..." << endl;
			// 	for (unsigned int i = 0; i < N_0; i++) {
			// 		cout << previous_path_x[i] << ", " << previous_path_y[i] << endl;
			// 		next_x_vals.push_back(previous_path_x[i]);
			// 		next_y_vals.push_back(previous_path_y[i]);
			// 	}
			// 	cout << "Done" << endl;
			// 	// cout << "Ending s: " << end_path_s << endl;
			// 	// cout << "Pred x, y at ending s: " << s_x(end_path_s) << ", " << s_y(end_path_s) << endl;
			// }

			// vector<double> new_xycar_0  = getXY(end_path_s,      end_path_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			// vector<double> new_xycar_25 = getXY(end_path_s + 25, (new_dcar-end_path_d)/2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			// vector<double> new_xycar_50 = getXY(end_path_s + 50, new_dcar, map_waypoints_s, map_waypoints_x, map_waypoints_y);

			// vector<double> s_knots = {end_path_s, end_path_s + 25.0, end_path_s + 50.0};
			// vector<double> x_knots = {new_xycar_0[0], new_xycar_25[0], new_xycar_50[0]};
			// vector<double> y_knots = {new_xycar_0[1], new_xycar_25[1], new_xycar_50[1]};

			// tk::spline s_x;
			// tk::spline s_y;
			// // s_x.set_boundary(s_x.first_deriv, 0.0, s_x.first_deriv, 0.0);
			// // s_y.set_boundary(s_y.first_deriv, 0.0, s_y.first_deriv, 0.0);
			// s_x.set_points(s_knots, x_knots); // currently it is required that X is already sorted
			// s_y.set_points(s_knots, y_knots);

			// car_speed_derated = car_speed_target;
			// ds = car_speed_derated * dt;

			// for(unsigned int i = 0; i < N - N_0; i++)
			// {

			// 	// Limit speed based on accel and jerk 
			// 	// if ((car_speed_target - car_speed_derated) / dt > max_accel) 
			// 	// {
			// 	// 	car_speed_derated = car_speed + max_accel * dt;
			// 	// } else {
			// 	// 	car_speed_derated = car_speed_target;
			// 	// }
				
			// 	double new_scar = end_path_s + ds * (i + 1);

			// 	// double new_xcar = car_x + (ds * i) * cos(deg2rad(car_yaw));
			// 	// double new_ycar = car_y + (ds * i) * sin(deg2rad(car_yaw));

			// 	// vector<double> new_xycar = getXY(new_scar, new_dcar, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			// 	// double new_xcar = new_xycar[0];
			// 	// double new_ycar = new_xycar[1];

			// 	// vector<double> new_xycar  = getXY(new_scar, 0, map_waypoints_s, map_waypoints_x, map_waypoints_y);

			// 	double new_xcar = s_x(new_scar);
			// 	double new_ycar = s_y(new_scar);

			// 	next_x_vals.push_back(new_xcar);
			// 	next_y_vals.push_back(new_ycar);

			// 	cout << i << ", " << new_scar << ", " << new_xcar << ", " << new_ycar << ", " << endl;

			// }

			// cout << "End of path generation" << endl;


			// vector<double> next_x_vals;
			// vector<double> next_y_vals;
  
			// double pos_x;
			// double pos_y;
			// double angle;
			// int path_size = previous_path_x.size();
  
			// for(int i = 0; i < path_size; i++)
			// {
			// 	next_x_vals.push_back(previous_path_x[i]);
			// 	next_y_vals.push_back(previous_path_y[i]);
			// }
  
			// if(path_size == 0)
			// {
			// 	pos_x = car_x;
			// 	pos_y = car_y;
			// 	angle = deg2rad(car_yaw);
			// }
			// else
			// {
			// 	pos_x = previous_path_x[path_size-1];
			// 	pos_y = previous_path_y[path_size-1];
  
			// 	double pos_x2 = previous_path_x[path_size-2];
			// 	double pos_y2 = previous_path_y[path_size-2];
			// 	angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
			// }
  
			// double dist_inc = 0.5;
			// for(int i = 0; i < 50-path_size; i++)
			// {    
			// 	next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
			// 	next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
			// 	pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
			// 	pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
			// }

          	msgJson["next_x"] = x_trajectory;
          	msgJson["next_y"] = y_trajectory;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
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
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































