#include <iostream>
#include "road.h"
#include "vehicle.h"
#include <vector>

using namespace std;

int main(){

    // Vehicle vehicle_1(1,1,0,0,0);
    // Vehicle vehicle_2(2,1,2,0,8);
    // Vehicle vehicle_3(3,0,5,3,0);
    // Vehicle vehicle_4(4,1,7,0,0);
    // Vehicle vehicle_5(5,3,0,0,2);
                    //    
    vector<double> ego_vector = 
    {
        0, // id
        2, // lane
        3, // s
        4, // v
        6  // g
    };

    vector<vector<double>> traffic_vector;

    traffic_vector.push_back(
        {
            1, // id
            -1, // x
            -1, // y
            sqrt(2), // vx
            sqrt(2), // vy
            3, // s
            2, // d
        }
    );

    traffic_vector.push_back(
        {
            2, // id
            -1, // x
            -1, // y
            sqrt(5*5/2.0), // vx
            sqrt(5*5/2.0), // vy
            4, // s
            2, // d
        }
    );

    traffic_vector.push_back(
        {
            3, // id
            -1, // x
            -1, // y
            sqrt(4*4/2.0), // vx
            sqrt(4*4/2.0), // vy
            6, // s
            2, // d
        }
    );

    Road road;
    road.Populate(ego_vector, traffic_vector);

    cout << "Initial Values" << endl;
    road.Print();

    cout << "Simulation" << endl;
    road.Simulate(4.0);
    road.Print();

    cout << "Reset" << endl;
    road.Reset();
    road.Print();

    return 0;

}