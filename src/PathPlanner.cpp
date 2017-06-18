#include "PathPlanner.hpp"

PathPlanner::PathPlanner(){
	m_tools = tools();
}

PathPlanner::~PathPlanner() {}

void PathPlanner::main_loop(double car_x,
											 double car_y,
											 double car_s,
											 double car_d,
											 double car_yaw,
											 double car_speed,
//											 auto previous_path_x,
//											 auto previous_path_y,
											 double end_path_s,
											 double end_path_d,
//											 auto sensor_fusion,
											 vector<double> &next_x_vals,
											 vector<double> &next_y_vals){
//	double dist_inc = 0.5;
//	for(int i = 0; i < 50; i++)
//	{
//				next_x_vals.push_back(car_x+(dist_inc*i)*cos(m_tools.deg2rad(car_yaw)));
//				next_y_vals.push_back(car_y+(dist_inc*i)*sin(m_tools.deg2rad(car_yaw)));
//	}
}
