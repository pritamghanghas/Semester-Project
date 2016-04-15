
/*
 * Author: Marco Zorzi
 * Description: ROS node for Skye pose control
 */

#ifndef POSE_CONTROL_NODE_H
#define POSE_CONTROL_NODE_H

#include <stdio.h>
#include <iostream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <skye_ros/ApplyWrenchCogBf.h>
#include <skye_controls/skye_geometric_controller.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/LinkState.h>
#include <skye_controls/skye_paramsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <skye_controls/waypoint_controller.h>


class PoseControllerNode {
public:
    PoseControllerNode(ros::NodeHandle nh);
    ~PoseControllerNode ();

    void ConfigCallback(skye_controls::skye_paramsConfig& config, uint32_t level);
    void PositionCallback(const gazebo_msgs::LinkState::ConstPtr& msg);
    void AngularVelocityCallback(const sensor_msgs::Imu::ConstPtr& msg);
    bool CallService();

private:
    //functions
    bool ParseParameters(ros::NodeHandle nh);

    //variables
    double k_x, k_v, k_R, k_omega, k_if, k_im;
    std::string wrench_service_name_;
    std::string points_file_path_;

    //ros stuff
    geometry_msgs::Wrench temporaryWrench_;
    ros::ServiceClient wrench_service_;
    skye_ros::ApplyWrenchCogBf srv_;

    //dynamic reconfigure server for dynamic parameters
    dynamic_reconfigure::Server<skye_controls::skye_paramsConfig> dr_srv_;
    dynamic_reconfigure::Server<skye_controls::skye_paramsConfig>::CallbackType cb;

    //Waypoint Controllers
    WaypointController waypoint_controller_;
    WaypointControllerParameters waypoint_parameters_;

    //Skye node
    SkyeGeometricController geometric_controller_;
    SkyeParameters skye_parameters_;

    //Eigen variables
    Eigen::Vector3d position_, velocity_, angular_velocity_;
    Eigen::Quaterniond orientation_;
    Eigen::Vector3d control_force_, control_acceleration_, control_momentum_;
    Eigen::Matrix3d R_des_, inertia_;

};


#endif // POSE_CONTROL_NODE_H
