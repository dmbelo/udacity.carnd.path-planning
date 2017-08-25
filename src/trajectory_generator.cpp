#include "trajectory_generator.h"

TrajectoryGenerator::TrajectoryGenerator(vector<double> map_x, vector<double> map_y, vector<double> map_s)
{
    this->map_x = map_x;
    this->map_y = map_y;
    this->map_s = map_s;
}

TrajectoryGenerator::~TrajectoryGenerator(){}

void TrajectoryGenerator::SetInitialPose(double x, double y, double theta0)
{
    this->x0 = x;
    this->y0 = y;
    this->theta0 = theta0;
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

void TrajectoryGenerator::Generate(double distance, vector<double> &x_trajectory, vector<double> &y_trajectory)
{

    vector<double> x_spline;
    vector<double> y_spline;

    double s_horizon = 30;
    double x1 = x0 + s_horizon * cos(theta0);
    double y1 = y0 + s_horizon * sin(theta0);
    vector<double> sd_car = getFrenet(x1, y1, theta0, map_x, map_y);

   

    // vector<double> xy_car_3 = getXY(sd_car[0],     4 * (target_lane - 1) + 2, map_s, map_x, map_y);
    // vector<double> xy_car_4 = getXY(sd_car[0]+ 10, 4 * (target_lane - 1) + 2, map_s, map_x, map_y);
    // vector<double> xy_car_5 = getXY(sd_car[0]+ 20, 4 * (target_lane - 1) + 2, map_s, map_x, map_y);

    vector<double> xy_car_3 = getXY(sd_car[0],     0, map_s, map_x, map_y);
    vector<double> xy_car_4 = getXY(sd_car[0]+ 15, 0, map_s, map_x, map_y);
    vector<double> xy_car_5 = getXY(sd_car[0]+ 30, 0, map_s, map_x, map_y);

    cout << xy_car_3[0] << ", " << xy_car_3[1] << endl;
    cout << xy_car_4[0] << ", " << xy_car_4[1] << endl;
    cout << xy_car_5[0] << ", " << xy_car_5[1] << endl;

    x_spline.push_back(x0 - 0.01 * cos(theta0));
    y_spline.push_back(y0 - 0.01 * sin(theta0));
    x_spline.push_back(x0);
    y_spline.push_back(y0);
    x_spline.push_back(xy_car_3[0]);
    y_spline.push_back(xy_car_3[1]);
    x_spline.push_back(xy_car_4[0]);
    y_spline.push_back(xy_car_4[1]);
    x_spline.push_back(xy_car_5[0]);
    y_spline.push_back(xy_car_5[1]);

    // Convert x,y_spline from world to vehicle reference frame
    for (int i = 0; i < x_spline.size(); i++)
    {
        double dx = x_spline[i] - x0;
        double dy = y_spline[i] - y0;
        x_spline[i] = dx * cos(theta0) + dy * sin(theta0);
        y_spline[i] = -dx * sin(theta0) + dy * cos(theta0);
    }

    tk::spline s; // Spline object for interpolation
    s.set_points(x_spline, y_spline);

    double xi = 0;
    double yi = 0;
    double dt = 0.02;
    double ds = target_speed * dt;

    for (int i = 0; i < 101; i++)
    {
        xi += ds;
        yi = s(xi);

        // Convert from vehicle to world reference frame and append
        x_trajectory.push_back(xi * cos(theta0) - yi * sin(theta0) + x0);
        y_trajectory.push_back(xi * sin(theta0) + yi * cos(theta0) + y0);
    }

}