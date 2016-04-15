#ifndef WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROLLER_H

#include <Eigen/Eigen>
#include <stdlib.h>
#include <vector>

struct WaypointControllerParameters{
    std::vector<Eigen::Vector3d> waypoints_;
};

class WaypointController
{
public:
    WaypointController();
};

#endif // WAYPOINT_CONTROLLER_H
