#ifndef MAP_H
#define MAP_H

class Map{
public:
    vector<double> m_map_waypoints_x;
    vector<double> m_map_waypoints_y;
    vector<double> m_map_waypoints_s;
    vector<double> m_map_waypoints_dx;
    vector<double> m_map_waypoints_dy;

    Map(){}
    Map(vector<double> &map_waypoints_x,
        vector<double> &map_waypoints_y,
        vector<double> &map_waypoints_s,
        vector<double> &map_waypoints_dx,
        vector<double> &map_waypoints_dy):
            m_map_waypoints_x(map_waypoints_x),
            m_map_waypoints_y(map_waypoints_y),
            m_map_waypoints_s(map_waypoints_s),
            m_map_waypoints_dx(map_waypoints_dx),
            m_map_waypoints_dy(map_waypoints_dy){}

};

#endif // MAP_H
