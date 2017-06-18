#ifndef TOOLS_H
#define TOOLS_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <iostream>

class Tools{
public:
	Tools();
	virtual ~Tools();
	vector<double> getFrenet(double x,
													 double y,
													 double theta,
													 vector<double> maps_x,
													 vector<double> maps_y);

	double distance(double x1, double y1,
									double x2, double y2);

	int ClosestWaypoint(double x,double y,
											vector<double> maps_x,
											vector<double> maps_y);

	int NextWaypoint(double x,
									 double y,
									 double theta,
									 vector<double> maps_x,
									 vector<double> maps_y);

	vector<double> getXY(double s,
											 double d,
											 vector<double> maps_s,
											 vector<double> maps_x,
											 vector<double> maps_y);
private:

};

#endif // TOOLS_H
