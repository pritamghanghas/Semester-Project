#ifndef WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROLLER_H

#include <Eigen/Eigen>
#include <stdlib.h>
#include <vector>

struct WaypointControllerParameters{
    std::vector<Eigen::Vector3d> input_waypoints_;
};

class WaypointController
{
public:
    WaypointController();
    ~WaypointController();


private:
    std::vector<Eigen::Vector3d> waypoints_;

};

#endif // WAYPOINT_CONTROLLER_H
