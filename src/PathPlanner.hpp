#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "tools.hpp"
#include "Other_car.hpp"
#include "map.hpp"
#include "spline.h"
#include "track.hpp"

#define TIME_INTERVAL 0.02
#define MPH2MS				0.44704

using namespace std;

class PathPlanner {
public:
	PathPlanner();

	~PathPlanner();

	enum lane_changing_status{
		CHANGING_LANE,
		KEEPING_LANE
	}m_change_status;

	void update(double car_x,
						double car_y,
						double car_s,
						double car_d,
						double car_yaw,
						double car_speed,
						vector<double> &previous_path_x,
						vector<double> &previous_path_y,
						double end_path_s,
						double end_path_d,
						vector<Other_car> &sensor_fusion);

	void get_path(vector<double> &next_x_vals,
								vector<double> &next_y_vals);

	void set_map(Map &map);

private:
	tools m_tools;

	Map m_map;

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
	vector<Other_car> m_front_car;

	Track m_left,m_center,m_right;
	Track m_change;

	void keep_track(vector<double> &next_x_vals,
								 vector<double> &next_y_vals,
								 float speed,
								 int path_length,
								 Track &lane);
	double set_speed(double desired, double pre_speed);
	double inc2MPH(double inc);
	double MPH2inc(double MPH);
	Other_car get_front_car(double d,double planned_s, double secure_dist);
	vector<Other_car> find_cars_on_this_side(double d);
	vector<Other_car> find_cars_on_this_lane(double d,vector<Other_car> &among_these_cars);
	Other_car find_closest_car(double s,vector<Other_car> &among_these_cars);

	void setup_lane_changing(Track target, Track curr ,float s_obstacle, float speed);
	void change_lane_to(Track lane);

	Track find_best_escape_lane(Track &current_lane);
};


#endif //PATHPLANNER_H
