#include "PathPlanner.hpp"

PathPlanner::PathPlanner(){
	m_tools = tools();
}

PathPlanner::~PathPlanner() {}

void PathPlanner::set_map(Map &map){
	m_map = map;

	m_lane2_x.set_points(m_map.m_map_waypoints_s, m_tools.operation(m_map.m_map_waypoints_x,m_map.m_map_waypoints_dx,6));
	m_lane2_y.set_points(m_map.m_map_waypoints_s, m_tools.operation(m_map.m_map_waypoints_y,m_map.m_map_waypoints_dy,6));
}

void PathPlanner::update(double car_x,
											 double car_y,
											 double car_s,
											 double car_d,
											 double car_yaw,
											 double car_speed,
											 vector<double> &previous_path_x,
											 vector<double> &previous_path_y,
											 double end_path_s,
											 double end_path_d,
											 vector<Other_car> &sensor_fusion){
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

void PathPlanner::keep_lane(vector<double> &next_x_vals,
														vector<double> &next_y_vals){
	int not_met_path_size = m_previous_path_x.size(); //fmin( m_previous_path_x.size() , 20);
	int path_length = 50;
	int n_set_points_from_pre_path = min(10 , not_met_path_size);
	int idx_first_unmet_set_point_from_pre_path = path_length - not_met_path_size;
	vector<double> planned_s;
	static vector<double> pre_planned_s;

	if (not_met_path_size==0){
		planned_s.push_back( m_car_s );

		double dist_inc=MPH2inc(m_car_speed) ;
		for(int i = 0; i < path_length-1 ; i++){
			dist_inc = set_speed (48, inc2MPH(dist_inc));//MPH
			planned_s.push_back(planned_s.back() + dist_inc);
		}
	}
	else
	{
		for(int i = 0; i < n_set_points_from_pre_path; i++){
			planned_s.push_back(pre_planned_s[idx_first_unmet_set_point_from_pre_path + i]);
		}

		int len = planned_s.size();
		double dist_inc =planned_s[len-1]-planned_s[len-2];
		for(int i = n_set_points_from_pre_path; i < path_length ; i++){
			dist_inc = set_speed (48,inc2MPH(dist_inc));//MPH
			planned_s.push_back(planned_s.back() + dist_inc);
		}
	}

	for(auto it = planned_s.begin() ; it < planned_s.end(); it++){
		double s_p = *it;
		next_x_vals.push_back(m_lane2_x(s_p));
		next_y_vals.push_back(m_lane2_y(s_p));
	}
	pre_planned_s = planned_s;
//	m_tools.twoPlot(m_map.m_map_waypoints_x,m_map.m_map_waypoints_y,"blue",next_x_vals,next_y_vals,"red");
}





void PathPlanner::get_path(vector<double> &next_x_vals,
													 vector<double> &next_y_vals){
	keep_lane(next_x_vals,next_y_vals);

}

float PathPlanner::set_speed(float desired, float pre_speed){
	float step = 0.4;
	if (desired - pre_speed < 1.){
		step *= -1;
	} else if(fabs(desired - pre_speed) < 1.){
		step = 0;
	}

	float inc = MPH2inc(pre_speed+step);
	return inc;
}

float PathPlanner::inc2MPH(float inc){
	float mph =inc / TIME_INTERVAL / 0.44704;
	return mph;
}

float PathPlanner::MPH2inc(float MPH){
	float inc =MPH * TIME_INTERVAL *0.44704;
	return inc;
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
	int path_size = m_previous_path_x.size();

	for(int i = 0; i < path_size; i++)
	{
			next_x_vals.push_back(m_previous_path_x[i]);
			next_y_vals.push_back(m_previous_path_y[i]);
	}

	if(path_size == 0)
	{
			pos_x = m_car_x;
			pos_y = m_car_y;
			angle = m_tools.deg2rad(m_car_yaw);
	}
	else
	{
			pos_x = m_previous_path_x[path_size-1];
			pos_y = m_previous_path_y[path_size-1];

			double pos_x2 = m_previous_path_x[path_size-2];
			double pos_y2 = m_previous_path_y[path_size-2];
			angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
	}

	double dist_inc = 0.5;
	for(int i = 0; i < 50-path_size; i++)
	{
			next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(M_PI/100)));
			next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(M_PI/100)));
			pos_x += (dist_inc)*cos(angle+(i+1)*(M_PI/100));
			pos_y += (dist_inc)*sin(angle+(i+1)*(M_PI/100));
	}
*/
