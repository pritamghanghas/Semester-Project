#ifndef SKYE_H
#define SKYE_H
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <skye_ros/ApplyWrenchCogNed.h>

class Skye
{
public:
    Skye();

    bool init(ros::ServiceClient a_wrench_service);
    bool apply_force(Eigen::Vector3d force);
    bool apply_torque(Eigen::Vector3d torque);
    bool apply_wrench(Eigen::Vector3d force, Eigen::Vector3d torque);

private:
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d attitude;
    Eigen::Vector3d angular_velocity;

    ros::ServiceClient wrench_service;
    skye_ros::ApplyWrenchCogNed srv;

};


#endif // SKYE_H
