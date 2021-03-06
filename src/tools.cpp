#include "tools.hpp"

tools::tools(){
    myfile = std::fopen("graph1","w");
    std::fclose(myfile);

    gp_ = popen("gnuplot -persist" , "w");
}
tools::~tools(){}

void tools::updateGraphFromFile(double value){
    myfile = std::fopen("NIS_lidar","a");
    if(myfile != NULL){
        std::string toWrite = std::to_string(value) + '\n';
        std::fwrite(toWrite.c_str(),1,toWrite.length(),myfile);
        std::fclose(myfile);
    }

    if (gp_!=NULL){
        fprintf(gp_ , "call '../plotScript.gp'\n");
        std::fflush(gp_);
    }
}

void tools::plot(vector<vector<double>> &data){
    if (gp_!=NULL && data.size()!=0){
        fprintf(gp_ , "plot '-' using 1:2 with lines\n");
        for (int i=0; i< data.size(); i++){
            fprintf(gp_, "%f.5 %f.5\n",data[i][0],data[i][1]);
        }
        fprintf(gp_,"e\n");
        fflush(gp_);
    }
}

void tools::plot(vector<double> &x , vector<double> &y, char const* color){
    if (gp_!=NULL && x.size()!=0 && x.size()==y.size()){
        fprintf(gp_ , "plot '-' using 1:2 with linespoint lc rgb '%s'\n",color);
        for (int i=0; i< x.size(); i++){
            fprintf(gp_, "%f %f\n",x[i],y[i]);
        }
        fprintf(gp_,"e\n");
        fflush(gp_);
    }
}

void tools::twoPlot(vector<double> &x , vector<double> &y, char const* color, vector<double> &x1 , vector<double> &y1, char const* color1){
    if (gp_!=NULL && x.size()!=0 && x.size()==y.size() && x1.size()!=0 && x1.size()==y1.size()){
        fprintf(gp_ , "reset\n");
        fprintf(gp_ , "set size ratio -1\n");
        //fprintf(gp_ , "set size 1,1\n");
        fprintf(gp_ , "set multiplot\n");
        //fprintf(gp_ , "set size square\n");
        fprintf(gp_ , "set size 0.5,1\n");
        fprintf(gp_ , "set origin 0,0\n");
        fprintf(gp_ , "plot '-' using 1:2 with linespoint lc rgb '%s', '-' using 1:2 with linespoint lc rgb '%s'\n",color, color1);
        for (int i=0; i< x.size(); i++){
            fprintf(gp_, "%f %f\n",x[i],y[i]);
        }
        fprintf(gp_,"e\n");
        fflush(gp_);
        for (int i=0; i< x1.size(); i++){
            fprintf(gp_, "%f %f\n",x1[i],y1[i]);
        }
        fprintf(gp_,"e\n");
        fflush(gp_);

        fprintf(gp_ , "set origin 0.5,0\n");
        //fprintf(gp_ , "set size square\n");
        fprintf(gp_ , "plot '-' using 1:2 with linespoint lc rgb '%s'\n", color1);
        for (int i=0; i< x1.size(); i++){
            fprintf(gp_, "%f %f\n",x1[i],y1[i]);
        }
        fprintf(gp_,"e\n");
        fflush(gp_);
    }
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> tools::getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x, maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

double tools::distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int tools::ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

int tools::NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2( (map_y-y),(map_x-x) );

    double angle = abs(theta-heading);

    if(angle > pi()/4)
    {
        closestWaypoint++;
    }

    return closestWaypoint;

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> tools::getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};

}

vector<double> tools::operation(vector<double> &sorc1,vector<double> &sorc2, double times_by){
    vector<double> result;
    for (int i = 0; i< sorc1.size(); i++){
        result.push_back(sorc1[i] + sorc2[i]*times_by);
    }
    return result;
}
