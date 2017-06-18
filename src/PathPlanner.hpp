#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "tools.hpp"
#include "Other_car.h"

using namespace std;

class PathPlanner {
public:
	PathPlanner();

	~PathPlanner();

	void update(double car_x,
						double car_y,
						double car_s,
						double car_d,
						double car_yaw,
						double car_speed,
						vector<double> previous_path_x,
						vector<double> previous_path_y,
						double end_path_s,
						double end_path_d,
						vector<Other_car> sensor_fusion);

	void get_path(vector<double> &next_x_vals,
								vector<double> &next_y_vals);

private:
	tools m_tools;

	double m_car_x;
	double m_car_y;
	double m_car_s;
	double m_car_d;
	double m_car_yaw;
	double m_car_speed;
	vector<double> m_previous_path_x;
	vector<double> m_previous_path_y;
	double m_end_path_s;
	double m_end_path_d;
	vector<Other_car> m_sensor_fusion;

	void keep_lane(void);
};


#endif //PATHPLANNER_H
