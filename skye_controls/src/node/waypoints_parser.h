#ifndef WAYPOINTS_PARSER_H
#define WAYPOINTS_PARSER_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>

/**
 * @brief The WaypointsParser class is used to provide a thin interface for
 * parsing parameters. It returns a vector of Eigen::Vector3d representing the 3D
 * inertial frame points to visit and a vector of Eigen::Quaterniond representing
 * Skye's orientations expressed in quaternions
 */
class WaypointsParser
{
public:
    /**
     * @brief WaypointsParser : The constructor gets parameters from ROS and packs the vectors
     * @param nh : ROS node where the parser is called from
     * @param waypoints : output vector containing inertial frame coordinates
     * @param orientations : output vector containing inertial frame orientations
     */
    WaypointsParser(ros::NodeHandle nh,
                    std::vector<Eigen::Vector3d> *waypoints,
                    std::vector<Eigen::Quaterniond> *orientations);

    ~WaypointsParser();
};

#endif // WAYPOINTS_PARSER_H
