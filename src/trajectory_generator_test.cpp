#include <fstream>
#include <sstream>
#include <string>
#include "trajectory_generator.h"

using namespace std;

int main()
{

    vector<double> map_x;
	vector<double> map_y;
    vector<double> map_s;


    ifstream in_map_("../data/highway_map.csv");

    // Read map in 
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
		map_x.push_back(x);
		map_y.push_back(y);
		map_s.push_back(s);
    }

    TrajectoryGenerator traj(map_x, map_y, map_s);

    traj.SetHorizonDistance(30);
    traj.SetTargetLane(1);
    traj.SetTargetSpeed(30);
    traj.SetInitialPose(1059.0, 1169.0, 0.55);
    
    vector<double> x_trajectory;
    vector<double> y_trajectory;
    
    traj.Generate(x_trajectory, y_trajectory);

    ofstream output_file("output.csv");

    for (int i = 0; i < x_trajectory.size(); i++)
    {
        output_file << x_trajectory[i] << ", " << y_trajectory[i] << endl;
    }

    output_file.close();

    return 0;

}