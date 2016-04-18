#ifndef WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROLLER_H

#include <Eigen/Eigen>
#include <stdlib.h>
#include <vector>
#include <iostream>

struct WaypointControllerParameters{
    std::vector<Eigen::Vector3d> input_waypoints;
    double input_goal_change_threshold;
};

class WaypointController
{
public:
    WaypointController();
    ~WaypointController();

    void InitParameters(WaypointControllerParameters some_param);
    void ComputeGoal(const Eigen::Vector3d &current_position,
                     Eigen::Vector3d *new_goal);
private:
    double goal_change_threshold_;

    std::vector<Eigen::Vector3d> waypoints_;
    WaypointControllerParameters controller_parameters_;

};

#endif // WAYPOINT_CONTROLLER_H
