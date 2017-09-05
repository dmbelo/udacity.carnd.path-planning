#include "trajectory_generator.h"

TrajectoryGenerator::TrajectoryGenerator(vector<double> map_x, vector<double> map_y, vector<double> map_s)
{
    this->map_x = map_x;
    this->map_y = map_y;
    this->map_s = map_s;
}

TrajectoryGenerator::~TrajectoryGenerator(){}

void TrajectoryGenerator::SetHorizonDistance(double x_horizon)
{
    this->x_horizon = x_horizon;
}

void TrajectoryGenerator::SetInitialPose(double x, double y, double theta)
{
    this->x0 = x;
    this->y0 = y;
    this->theta0 = theta;
}

void TrajectoryGenerator::SetTargetSpeed(double target_speed)
{
    this->target_speed = target_speed;
}

void TrajectoryGenerator::SetTargetLane(int target_lane)
{
    this->target_lane = target_lane;
}

void TrajectoryGenerator::Print()
{
    cout << "Target Speed: " << target_speed << endl;
    cout << "Target Lan: " << target_lane << endl;
}

void TrajectoryGenerator::Generate(vector<double> &x_trajectory, vector<double> &y_trajectory)
{

    vector<double> x_spline;
    vector<double> y_spline;

    int n_trajectory = x_trajectory.size();

    // Find the coordinates for the start of the spline
    double x_start, y_start, theta_start;
    if (n_trajectory > 0)
    {
        double x_   = x_trajectory[n_trajectory - 2];
        double y_   = y_trajectory[n_trajectory - 2];
        x_start     = x_trajectory[n_trajectory - 1];
        y_start     = y_trajectory[n_trajectory - 1];
        theta_start = atan2(y_start - y_, x_start - x_); 
    }
    else
    {
        x_start     = this->x0;
        y_start     = this->y0;
        theta_start = this->theta0;
    }

    vector<double> sd_start = getFrenet(this->x0, this->y0, this->theta0, this->map_x, this->map_y);

    // Calculate the spline knots from the desired start of the spline
    vector<double> xy_car_1 = {x_start - 0.01 * cos(theta_start), y_start - 0.01 * sin(theta_start)};
    vector<double> xy_car_2 = {x_start, y_start};
    vector<double> xy_car_3 = getXY(sd_start[0] + 50,  4 * (target_lane - 1) + 2, map_s, map_x, map_y);
    vector<double> xy_car_4 = getXY(sd_start[0] + 75,  4 * (target_lane - 1) + 2, map_s, map_x, map_y);
    vector<double> xy_car_5 = getXY(sd_start[0] + 100, 4 * (target_lane - 1) + 2, map_s, map_x, map_y);

    x_spline.push_back(xy_car_1[0]);
    y_spline.push_back(xy_car_1[1]);
    x_spline.push_back(xy_car_2[0]);
    y_spline.push_back(xy_car_2[1]);
    x_spline.push_back(xy_car_3[0]);
    y_spline.push_back(xy_car_3[1]);
    x_spline.push_back(xy_car_4[0]);
    y_spline.push_back(xy_car_4[1]);
    x_spline.push_back(xy_car_5[0]);
    y_spline.push_back(xy_car_5[1]);

    // Convert x,y_spline from world to vehicle reference frame
    for (int i = 0; i < x_spline.size(); i++)
    {
        double dx = x_spline[i] - x_start;
        double dy = y_spline[i] - y_start;
        x_spline[i] = dx * cos(theta_start) + dy * sin(theta_start);
        y_spline[i] = -dx * sin(theta_start) + dy * cos(theta_start);
    }

    tk::spline s; // Spline object for interpolation
    s.set_points(x_spline, y_spline);

    double xi = 0;
    double yi = 0;
    double dt = 0.02;
    double dx = target_speed * dt;

    // Calculate how much of the trajectory horizon we've covered by the incomplete trajectory
    // By finding the distance between the current position of the vehicle and the start of the
    // trajectory and projecting that distance into the x-axis of the vehicle's csys
    double theta1 = atan2(y_start - this->y0, x_start - this->x0) - this->theta0;
    double x_horizon_start = distance(x_start, y_start, this->x0, this->y0) * cos(theta1);

    int n_knots = (this->x_horizon - x_horizon_start) / dx;
    for (int i = 0; i < n_knots; i++)
    {
        // Calculate new x position in car csys
        xi += dx;
        yi = s(xi);
        // Convert from vehicle to world reference frame and append
        x_trajectory.push_back(xi * cos(theta_start) - yi * sin(theta_start) + x_start);
        y_trajectory.push_back(xi * sin(theta_start) + yi * cos(theta_start) + y_start);
    }

}