#include <vector>
#include "utils.h"
#include "spline.h"

using namespace std;


class TrajectoryGenerator
{

    public:

        TrajectoryGenerator(vector<double> map_x,
                            vector<double> map_y, 
                            vector<double> map_s);

        ~TrajectoryGenerator();

        void SetInitialPose(double x, double y, double theta0);
        void SetTargetSpeed(double target_speed);
        void SetTargetLane(int target_lane);
        void Generate(double distance, vector<double> &x_trajectory, vector<double> &y_trajectory);
        void Print();

    private:

        vector<double> map_x;
        vector<double> map_y;
        vector<double> map_s;
        double target_speed;
        int target_lane;
        double x0;
        double y0;
        double theta0;

};