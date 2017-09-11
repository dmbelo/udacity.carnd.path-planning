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

    int n_trajectory = x_trajectory.size();

    // Find the coordinates for the start of the spline
    vector<double> xy_car_1;
    vector<double> xy_car_2;
    double x_start, y_start, theta_start;
    if (n_trajectory > 2)
    {
        double x_   = x_trajectory[n_trajectory - 2];
        double y_   = y_trajectory[n_trajectory - 2];
        x_start     = x_trajectory[n_trajectory - 1];
        y_start     = y_trajectory[n_trajectory - 1];
        theta_start = atan2(y_start - y_, x_start - x_); 
        xy_car_1 = {x_, y_};
        xy_car_2 = {x_start, y_start};
    }
    else
    {
        x_start     = this->x0;
        y_start     = this->y0;
        theta_start = this->theta0;
        xy_car_1 = {x_start - cos(theta_start), y_start - sin(theta_start)};
        xy_car_2 = {x_start, y_start};
    }

    vector<double> sd0 = getFrenet(this->x0, this->y0, this->theta0, this->map_x, this->map_y);

    // Calculate the spline knots from the desired start of the spline
    vector<double> xy_car_3 = getXY(sd0[0] + 60, 4 * (target_lane - 1) + 2, map_s, map_x, map_y);
    vector<double> xy_car_4 = getXY(sd0[0] + 75, 4 * (target_lane - 1) + 2, map_s, map_x, map_y);
    vector<double> xy_car_5 = getXY(sd0[0] + 90, 4 * (target_lane - 1) + 2, map_s, map_x, map_y);

    vector<double> x_spline;
    vector<double> y_spline;
    
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
        double new_x =  dx * cos(theta_start) + dy * sin(theta_start);
        double new_y = -dx * sin(theta_start) + dy * cos(theta_start);
        x_spline[i] = new_x;
        y_spline[i] = new_y;
    }

    tk::spline s; // Spline object for interpolation
    s.set_points(x_spline, y_spline);

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;

    double N = target_dist / (0.02 * target_speed);
    for (int i = 1; i < 50 - n_trajectory; i++)
    {
        double x_point = x_add_on + target_x/N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = x_ref * cos(theta_start) - y_ref * sin(theta_start);
        y_point = x_ref * sin(theta_start) + y_ref * cos(theta_start);
        
        x_point += x_start;
        y_point += y_start;

        x_trajectory.push_back(x_point);
        y_trajectory.push_back(y_point);

    }

}