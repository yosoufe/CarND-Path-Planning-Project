#include "PathPlanner.hpp"

PathPlanner::PathPlanner(){
	m_tools = tools();
	m_change_status = PathPlanner::KEEPING_LANE;
	Other_car empty_car;
	for (int i = 0; i <3; i ++)
		m_front_car.push_back(empty_car);
}

PathPlanner::~PathPlanner() {}

void PathPlanner::set_map(Map &map){
	m_map = map;
	Track left = Track(m_map.m_map_waypoints_s,
								 m_tools.operation(m_map.m_map_waypoints_x,m_map.m_map_waypoints_dx,2),
								 m_tools.operation(m_map.m_map_waypoints_y,m_map.m_map_waypoints_dy,2),
								 1 , 2.0);
	Track center = Track(m_map.m_map_waypoints_s,
								 m_tools.operation(m_map.m_map_waypoints_x,m_map.m_map_waypoints_dx,6),
								 m_tools.operation(m_map.m_map_waypoints_y,m_map.m_map_waypoints_dy,6),
								 2, 6.0);
	Track right = Track(m_map.m_map_waypoints_s,
								 m_tools.operation(m_map.m_map_waypoints_x,m_map.m_map_waypoints_dx,10),
								 m_tools.operation(m_map.m_map_waypoints_y,m_map.m_map_waypoints_dy,10),
								 3,10.0);

	left.priority_to_change_to.push_back(center);
	left.priority_to_change_to.push_back(left);
	center.priority_to_change_to.push_back(left);
	center.priority_to_change_to.push_back(right);
	center.priority_to_change_to.push_back(center);
	right.priority_to_change_to.push_back(center);
	right.priority_to_change_to.push_back(right);
	m_lanes.push_back(left);
	m_lanes.push_back(center);
	m_lanes.push_back(right);
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

void PathPlanner::keep_track(vector<double> &next_x_vals,
														vector<double> &next_y_vals,
														float speed,
														int path_length,
														Track &lane){
	int not_met_path_size = m_previous_path_x.size(); //fmin( m_previous_path_x.size() , 20);
	int n_set_points_from_pre_path = min(20 , not_met_path_size);
	int idx_first_unmet_set_point_from_pre_path = path_length - not_met_path_size;
	vector<double> planned_s;
	static vector<double> pre_planned_s;

	if (not_met_path_size==0){
		planned_s.push_back( m_car_s + 0.5 );

		double dist_inc=MPH2inc(m_car_speed) ;
		for(int i = 0; i < path_length-1 ; i++){
			dist_inc = set_speed (speed, inc2MPH(dist_inc));//MPH
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
			dist_inc = set_speed (speed,inc2MPH(dist_inc));//MPH
			planned_s.push_back(planned_s.back() + dist_inc);
		}
	}

	for(auto it = planned_s.begin() ; it < planned_s.end(); it++){
		double s_p = *it;
		next_x_vals.push_back(lane.m_s_x(s_p));
		next_y_vals.push_back(lane.m_s_y(s_p));
	}
	pre_planned_s = planned_s;
	//m_tools.twoPlot(m_map.m_map_waypoints_x,m_map.m_map_waypoints_y,"blue",next_x_vals,next_y_vals,"red");
}

void PathPlanner::get_path(vector<double> &next_x_vals,
													 vector<double> &next_y_vals){
	float speed = 48.5;//MPH
	int path_length = 50;
	float secure_dist = (speed+20) * MPH2MS * path_length * TIME_INTERVAL;

	Track lane;
	// which lane I am now?
	for (auto it = m_lanes.begin(); it < m_lanes.end() ; it++){
		lane = *it;
		if (fabs(m_car_d-lane.m_d)<1.5){ // I Am in this lane
			break;
		}
	}

	if(lane.m_id == 3)
		speed = 46;

	// detect the front cars and a bit behind on other lines
	for (int idx = 0 ; idx < m_front_car.size(); idx++){
		m_front_car[idx].isEmpty = true; //reset it
		if (idx == (lane.m_id - 1))
			get_front_car(m_front_car[idx],m_lanes[idx].m_d,m_car_s,secure_dist,0);
		else
			get_front_car(m_front_car[idx],m_lanes[idx].m_d,m_car_s,secure_dist + 20.0, -10.0);
	}

	switch (m_change_status){
		case CHANGING_LANE:
			speed = 40;
			if (m_car_s + secure_dist < m_change.m_s_end){
				keep_track(next_x_vals,next_y_vals,speed,path_length,m_change);
			} else{
				keep_track(next_x_vals,next_y_vals,speed,path_length,m_change);
				m_change_status = KEEPING_LANE;
			}
			break;
		case KEEPING_LANE:	
			Other_car car = m_front_car[lane.m_id-1];
			if ( !(car.isEmpty) ){ //If there is a car in front in this line
				Track target;
				find_best_escape_lane(lane,target, secure_dist);
				float s_diff = car.s-m_car_s;
				speed = inc2MPH(s_diff/path_length);
				if (target.m_id != lane.m_id){ // if the target lane is not the same as current lane
					setup_lane_changing(target,lane,car.s);
					keep_track(next_x_vals,next_y_vals,speed,path_length,m_change);
					break;
				} else { // if the target lane is the same as current lane and avoid crashing front car
					speed = sqrt(car.vx*car.vx + car.vy * car.vy);//inc2MPH(fmin( (car.s - m_car_s - 2.0) / path_length , 48.5));
					keep_track(next_x_vals,next_y_vals,speed,path_length,lane);
					break;
				}
			} else { // no car in front
				keep_track(next_x_vals,next_y_vals,speed,path_length,lane);
				break; // come out from the for loop
			}
			break;
	}
}

double PathPlanner::set_speed(double desired, double pre_speed){
	double error = desired - pre_speed;
	double step = 0.005 * error;
	if(fabs(error) < 0.01){
		step = 0;
	}

	double inc = MPH2inc(pre_speed+step); //pre_speed+step
	return inc;
}

double PathPlanner::inc2MPH(double inc){
	double mph =inc / TIME_INTERVAL / MPH2MS;
	return mph;
}

double PathPlanner::MPH2inc(double MPH){
	double inc =MPH * TIME_INTERVAL * MPH2MS;
	return inc;
}

void PathPlanner::get_front_car(Other_car &result1,double d,double planned_s, double secure_dist , float secure_dist_neg){
	Other_car result;
	vector<Other_car> CarsInMySide;
	CarsInMySide = find_cars_on_this_side(d);
	vector<Other_car> CarsInMyLane;
	CarsInMyLane = find_cars_on_this_lane(d,CarsInMySide);
	Other_car front_car = find_closest_car(planned_s,CarsInMyLane , secure_dist_neg);
	if (front_car.s - planned_s < secure_dist && front_car.s - planned_s> secure_dist_neg && CarsInMyLane.size()!=0)
		result = front_car;
	result1 = result;
}

vector<Other_car> PathPlanner::find_cars_on_this_side(double d){
	vector<Other_car> result;
	for(auto it = m_sensor_fusion.begin() ; it < m_sensor_fusion.end() ; it++){
		Other_car car = *it;
		if(car.d * d > 0){
			result.push_back(car);
		}
	}
	return result;
}

vector<Other_car> PathPlanner::find_cars_on_this_lane(double d,vector<Other_car> &among_these_cars){
	vector<Other_car> result;
	if (among_these_cars.size()!=0){
		for(auto it = among_these_cars.begin() ; it < among_these_cars.end() ; it++){
			Other_car car = *it;
			if(fabs(car.d - d) < 2.0){
				result.push_back(car);
			}
		}
	}
	return result;
}

Other_car PathPlanner::find_closest_car(double s,vector<Other_car> &among_these_cars,float secure_dist_neg){
	Other_car result;
	if (among_these_cars.size()!=0){
		double shortest_dist = INFINITY;
		for(auto it = among_these_cars.begin() ; it < among_these_cars.end() ; it++){
			Other_car car = *it;
			if(car.s - s > secure_dist_neg && fabs(car.s-s) < shortest_dist ){
				shortest_dist = car.s-s;
				result = car;
			}
		}
	}
	return result;
}

void PathPlanner::setup_lane_changing(Track &target, Track &curr ,float s_obstacle){
	vector<double> x,y,s_;
	float s = m_car_s-20.0;
	float inc =1.0;
	while(s < s_obstacle){
		s += inc;
		double temp_x = curr.m_s_x(s);
		double temp_y = curr.m_s_y(s);
		s_.push_back(s);
		x.push_back(temp_x);
		y.push_back(temp_y);
	}

	s += 40;
	float d = s + 40;

	while(s < d){
		s += inc;
		double temp_x = target.m_s_x(s);
		double temp_y = target.m_s_y(s);
		s_.push_back(s);
		x.push_back(temp_x);
		y.push_back(temp_y);
	}

	m_change = Track(s_,x,y,0,0);
	m_change.m_s_end = s;
	m_change.m_s_x.set_points(s_,x);
	m_change.m_s_y.set_points(s_,y);
	m_change_status = CHANGING_LANE;
	// m_tools.twoPlot(m_map.m_map_waypoints_x,m_map.m_map_waypoints_y,"blue",x,y,"red");
}

void PathPlanner::find_best_escape_lane(Track &current_lane, Track &target, float secure_dist){
	Track result;
	float dist = INFINITY;
	for (int i=0; i<current_lane.priority_to_change_to.size();i++){
		int id = current_lane.priority_to_change_to[i].m_id - 1;
		float s_diff = m_front_car[id].s - m_car_s;
		if (m_front_car[id].isEmpty){
			result = current_lane.priority_to_change_to[i];
			break;
		} else	if (s_diff < dist && s_diff > -secure_dist){
			dist = m_front_car[id].s - m_car_s;
			result = current_lane.priority_to_change_to[i];

			if (dist < secure_dist)
				result = current_lane;
		}
	}
	target = result;
}
