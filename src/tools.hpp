#ifndef TOOLS_H
#define TOOLS_H

#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <iostream>

using namespace std;

class tools{
public:
    tools();
    ~tools();
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

    // For converting back and forth between radians and degrees.
    double pi() { return M_PI; }
    double deg2rad(double x) { return x * M_PI / 180.0; }
    double rad2deg(double x) { return x * 180.0 / M_PI; }

    vector<double> operation(vector<double> &sorc1,vector<double> &sorc2, double times_by);
    void updateGraphFromFile(double value);
    void plot(vector<vector<double>> &data);
    void plot(vector<double> &x , vector<double> &y, char const* color);
    void twoPlot(vector<double> &x , vector<double> &y, char const* color, vector<double> &x1 , vector<double> &y1, char const* color1);

private:
    FILE *myfile;
    FILE *gp_;
};

#endif // TOOLS_H
