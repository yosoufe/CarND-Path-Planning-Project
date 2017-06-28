#ifndef TRACK_H
#define TRACK_H


#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"

class Track{
private:
public:
		Track(){}
		Track(vector<double> s,vector<double> x,vector<double> y,int id){
			m_s_x.set_points(s,x);
			m_s_y.set_points(s,y);
			m_id = id;
		}
		~Track(){}

		tk::spline m_s_x,m_s_y;
		int m_id;
		vector<Track> priority_to_change_to;
		float m_s_end;
};

#endif // TRACK_H
