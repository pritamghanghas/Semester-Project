#ifndef SKYE_H
#define SKYE_H
#include <stdio.h>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <skye_ros/ApplyWrenchCogNed.h>
#include <sensor_msgs/Imu.h>

class Skye
{
public:
    Skye();

    bool init(ros::ServiceClient a_wrench_service);
    bool apply_force(Eigen::Vector3d force);
    bool apply_torque(Eigen::Vector3d torque);
    bool apply_wrench(Eigen::Vector3d force, Eigen::Vector3d torque);
    void acceleration_callback(const sensor_msgs::Imu::ConstPtr& msg);

private:
    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;
    Eigen::Vector3d acceleration_;
    Eigen::Vector3d attitude_;
    Eigen::Vector3d angular_velocity_;
    Eigen::Vector3d zero;

    long double timestamp_;
    int counter_;
    bool start_counting_;


    ros::ServiceClient wrench_service;
    skye_ros::ApplyWrenchCogNed srv;

};


#endif // SKYE_H
