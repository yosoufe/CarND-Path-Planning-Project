#ifndef TRACK_H
#define TRACK_H


#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"

using namespace std;

class Track{
private:
public:
    Track(){m_id = 0;m_d = 0;}
    Track(vector<double> s,vector<double> x,vector<double> y,int id, double d){
        // add the first element to the end and
        // last element to the first to have a smoother map
        double temp_end = s.back();
        double delta_s = sqrt(pow(x.front()-x.back() , 2) + pow(y.front()-y.back() , 2));
        s.push_back(s.back() + delta_s);
        s.insert(s.begin(),s.front() - delta_s);

        temp_end = x.back();
        x.push_back(x.front());
        x.insert(x.begin(),temp_end);

        temp_end = y.back();
        y.push_back(y.front());
        y.insert(y.begin(),temp_end);

        m_s_x.set_points(s,x);
        m_s_y.set_points(s,y);
        m_id = id;
        m_s_end = 0;
        m_d = d;
    }
    ~Track(){}

    tk::spline m_s_x,m_s_y;
    int m_id;				// if the track is a lane, should have a none zero id;
    vector<Track> priority_to_change_to;
    double m_s_end;	// if the track is not lane, it should have the s value of its end
    double m_d;			// if the track is a lane, it should have d_value
};

#endif // TRACK_H
