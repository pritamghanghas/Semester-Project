
/*
 * Author: Marco Zorzi
 * Description: ROS node for Skye pose control
 */

#ifndef POSE_CONTROL_NODE_H
#define POSE_CONTROL_NODE_H

#include <stdio.h>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <skye_ros/ApplyWrenchCogBf.h>
#include <skye_controls/skye_geometric_controller.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/LinkState.h>
#include <skye_controls/skye_paramsConfig.h>
// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>


class PoseControllerNode {
public:
    PoseControllerNode(ros::NodeHandle nh);
    ~PoseControllerNode ();

    void ConfigCallback(skye_controls::skye_paramsConfig& config, uint32_t level);
    void PositionCallback(const gazebo_msgs::LinkState::ConstPtr& msg);

    bool CallService();
private:

    geometry_msgs::Wrench temporaryWrench_;
    ros::ServiceClient wrench_service_;
    skye_ros::ApplyWrenchCogBf srv_;

    Eigen::Vector3d position_, velocity_, angular_velocity_;
    Eigen::Vector3d control_force_, control_momentum_;
    SkyeGeometricController geometric_controller;
    Eigen::Quaterniond orientation_;

    Eigen::Matrix3d R_test;

    dynamic_reconfigure::Server<skye_controls::skye_paramsConfig> dr_srv_;
    dynamic_reconfigure::Server<skye_controls::skye_paramsConfig>::CallbackType cb;

    Eigen::Matrix3d inertia_;
    double k_x, k_v, k_R, k_omega;



};


#endif // POSE_CONTROL_NODE_H
