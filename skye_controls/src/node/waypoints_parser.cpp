#include "waypoints_parser.h"

WaypointsParser::~WaypointsParser(){}


WaypointsParser::WaypointsParser(ros::NodeHandle nh,
                                 std::vector<Eigen::Vector3d> *waypoints,
                                 std::vector<Eigen::Quaterniond> *orientations){

    // Sum a list of doubles from the parameter server
    std::vector<double> waypoints_x, waypoints_y, waypoints_z,
                        quaternions_x, quaternions_y, quaternions_z, quaternions_w;
    bool read_all_parameters = nh.getParam("waypoints_x", waypoints_x) &&
                               nh.getParam("waypoints_y", waypoints_y) &&
                               nh.getParam("waypoints_z", waypoints_z) &&
                               nh.getParam("quaternions_x", quaternions_x) &&
                               nh.getParam("quaternions_y", quaternions_y) &&
                               nh.getParam("quaternions_z", quaternions_z) &&
                               nh.getParam("quaternions_w", quaternions_w);

    // Check if Skye's parameters where imported
    if (! read_all_parameters){
        ROS_ERROR("Waypoints not imported");
        return;
    }

    // Check that waypoints were set up correctly
    if (waypoints_x.size() != waypoints_y.size() ||
            waypoints_y.size() != waypoints_z.size() ||
            waypoints_z.size() != quaternions_x.size() ||
            quaternions_x.size() != quaternions_y.size() ||
            quaternions_y.size() != quaternions_z.size() ||
            quaternions_z.size() != quaternions_w.size() ) {
        ROS_ERROR("Waypoints not correctly set up, please set them up correctly");
        return;
    }

    // Pack every waypoint in position and orientation
    for (int i = 0; i < waypoints_x.size(); ++i) {
        Eigen::Vector3d single_waypoint;
        single_waypoint << waypoints_x.at(i),
                            waypoints_y.at(i),
                            waypoints_z.at(i);
        waypoints->push_back(single_waypoint);

        Eigen::Quaterniond single_quaternion;
        single_quaternion.x() = quaternions_x.at(i);
        single_quaternion.y() = quaternions_y.at(i);
        single_quaternion.z() = quaternions_z.at(i);
        single_quaternion.w() = quaternions_w.at(i);
        orientations->push_back(single_quaternion);
    }
}
