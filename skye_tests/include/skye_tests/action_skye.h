#ifndef ACTION_SKYE_H
#define ACTION_SKYE_H

#include <skye_tests/skye.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Wrench.h>

class Actions_skye
{
public:
    Actions_skye(int a_time_in_ms_, Skye *a_skye); //ros::ServiceClient wrench_service,

    bool poke(Eigen::Vector3d force, int duration);
    bool twist(Eigen::Vector3d torque, int duration);
    bool move(Eigen::Vector3d force, Eigen::Vector3d torque, int duration);
    void poke_callback(const geometry_msgs::Wrench::ConstPtr& msg );
    void twist_callback(const geometry_msgs::Wrench::ConstPtr& msg );
    void move_callback(const geometry_msgs::Wrench::ConstPtr& msg );
    int duration_in_ms();

private:
    Skye skye;
    Eigen::Vector3d zero;
    int duration_in_ms_;

};

#endif // ACTION_SKYE_H
