#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "tools.hpp"

using namespace std;

class PathPlanner {
public:
	PathPlanner();

	~PathPlanner();

	void main_loop(double car_x,
						double car_y,
						double car_s,
						double car_d,
						double car_yaw,
						double car_speed,
//						auto previous_path_x,
//						auto previous_path_y,
						double end_path_s,
						double end_path_d,
//						auto sensor_fusion,
						vector<double> &next_x_vals,
						vector<double> &next_y_vals);

private:
	tools m_tools;

};


#endif //PATHPLANNER_H
