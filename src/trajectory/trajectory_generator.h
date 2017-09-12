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

        void SetHorizonDistance(double x_horizon);
        void SetInitialPose(double x, double y, double theta);
        void SetTargetSpeed(double target_speed);
        void SetTargetLane(int target_lane);
        void Generate(vector<double> &x_trajectory, vector<double> &y_trajectory);
        void Print();

    private:

        vector<double> map_x;
        vector<double> map_y;
        vector<double> map_s;
        double target_speed;   // m/s
        int target_lane;       
        double x_horizon;      // Trajectory horizon distance in vehicle csys
        double x0;             // Current vehicle x in world csys (m)
        double y0;             // Current vehicle y in world csys (m)
        double theta0;         // Current vehicle theta (rad)

};