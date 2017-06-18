#include "PathPlanner.hpp"

PathPlanner::PathPlanner(){
	m_tools = tools();
}

PathPlanner::~PathPlanner() {}

void PathPlanner::update(double car_x,
											 double car_y,
											 double car_s,
											 double car_d,
											 double car_yaw,
											 double car_speed,
											 vector<double> previous_path_x,
											 vector<double> previous_path_y,
											 double end_path_s,
											 double end_path_d,
											 vector<Other_car> sensor_fusion){
	m_car_x = car_x;
	m_car_y = car_y;
	m_car_s = car_s;
	m_car_d = car_d;
	m_car_yaw = car_yaw;
	m_car_speed = car_speed;
	m_previous_path_x = previous_path_x;
	m_previous_path_y = previous_path_y;
	m_end_path_s = end_path_s;
	m_end_path_d = end_path_d;
	m_sensor_fusion = sensor_fusion;

}

void PathPlanner::get_path(vector<double> &next_x_vals,
													 vector<double> &next_y_vals){

}

void PathPlanner::keep_lane(){

}


/*
	double dist_inc = 0.5;
	for(int i = 0; i < 50; i++)
	{
				next_x_vals.push_back(m_car_x+(dist_inc*i)*cos(m_tools.deg2rad(m_car_yaw)));
				next_y_vals.push_back(m_car_y+(dist_inc*i)*sin(m_tools.deg2rad(m_car_yaw)));
	}

	/////////////////////////////////
	double pos_x;
	double pos_y;
	double angle;
	int path_size = previous_path_x.size();

	for(int i = 0; i < path_size; i++)
	{
			next_x_vals.push_back(previous_path_x[i]);
			next_y_vals.push_back(previous_path_y[i]);
	}

	if(path_size == 0)
	{
			pos_x = car_x;
			pos_y = car_y;
			angle = m_tools.deg2rad(car_yaw);
	}
	else
	{
			pos_x = previous_path_x[path_size-1];
			pos_y = previous_path_y[path_size-1];

			double pos_x2 = previous_path_x[path_size-2];
			double pos_y2 = previous_path_y[path_size-2];
			angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
	}

	double dist_inc = 0.1;
	for(int i = 0; i < 50-path_size; i++)
	{
			next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(m_tools.pi()/100)));
			next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(m_tools.pi()/100)));
			pos_x += (dist_inc)*cos(angle+(i+1)*(m_tools.pi()/100));
			pos_y += (dist_inc)*sin(angle+(i+1)*(m_tools.pi()/100));
	}



	double dist_inc = 0.5;
	for(int i = 0; i < 50; i++)
	{
				next_x_vals.push_back(car_x+(dist_inc*i)*cos(m_tools.deg2rad(car_yaw)));
				next_y_vals.push_back(car_y+(dist_inc*i)*sin(m_tools.deg2rad(car_yaw)));
	}
*/
