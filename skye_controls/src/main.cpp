/* Author: Marco Zorzi
 * Description: File to run the controller with ros.
 *              This creates a controller object and
 *              applyes a force to skye given the controller
 *
 * TODO: - init ros
 *          - start a service with skye
 *          - get the position
 *          - compute the error
 *          - feed it to the controller and apply the resulting forces
 *
 * NOTES: check the timing, this should run every time the position is
 *         calculated
 */

#include <stdio.h>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <skye_ros/ApplyWrenchCogBf.h>
#include <skye_controls/skye_position_controller.h>
#include <skye_controls/skye_geometric_controller.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/LinkState.h>

ros::ServiceClient wrench_service;
skye_ros::ApplyWrenchCogBf srv;
geometry_msgs::Wrench temporaryWrench;

Eigen::Vector3d position_, velocity_, angular_velocity_;
Eigen::Vector3d control_force_, control_momentum_;
PositionController controller;
SkyeGeometricController geometric_controller;
Eigen::Quaterniond orientation_;

Eigen::Matrix3d R_test;
void callback(const gazebo_msgs::LinkState::ConstPtr& msg){

    position_ <<msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z;

    velocity_ << msg->twist.linear.x,
            msg->twist.linear.y,
            msg->twist.linear.z;

    orientation_.x() = msg->pose.orientation.x;
    orientation_.y() = msg->pose.orientation.y;
    orientation_.z() = msg->pose.orientation.z;
    orientation_.w() = msg->pose.orientation.w;

    angular_velocity_ << msg->twist.angular.x,
            msg->twist.angular.y,
            msg->twist.angular.z;


    geometric_controller.updateParameters(position_, velocity_, orientation_, angular_velocity_);
    geometric_controller.computeForce(control_force_);
    //    geometric_controller.computeMomentum(control_momentum_);


    /******************** DEBUG *************************/
    std::cout << "force: " << control_force_(0) <<
                 " | y: " << control_force_(1) <<
                 " | z: " << control_force_(2) <<
                 std::endl;

}

int main (int argc, char** argv) {

    ros::init(argc, argv, "skye_position_controller_node");
    ros::NodeHandle nh;

    //Declare parameters to be imported
    std::string wrench_service_name, imu_topic;
    double k_x, k_v, k_omega, k_R, mass;
    double desired_position_x, desired_position_y, desired_position_z;
    Eigen::Vector3d desired_position_,desired_velocity_,
            desired_angular_velocity_,desired_angular_acceleration_,
            desired_acceleration_;
    Eigen::Matrix3d inertia_;

    //Get the parameters
    bool read_all_parameters = nh.getParam("wrench_service_name", wrench_service_name) &&
            nh.getParam("imu_topic", imu_topic) &&
            nh.getParam("k_x", k_x) &&
            nh.getParam("k_v", k_v) &&
            nh.getParam("k_omega", k_omega) &&
            nh.getParam("k_R", k_R) &&
            nh.getParam("mass", mass) &&
            nh.getParam("desired_position_x", desired_position_x) &&
            nh.getParam("desired_position_y", desired_position_y) &&
            nh.getParam("desired_position_z", desired_position_z);

    desired_position_<< desired_position_x,
            desired_position_y,
            desired_position_z;

    if (! read_all_parameters) ROS_ERROR("Parameters not imported");

    geometric_controller.initializeParams(k_x, k_v, k_omega, k_R, mass,
                                          desired_position_,desired_velocity_,
                                          desired_angular_velocity_,desired_angular_acceleration_,
                                          desired_acceleration_, inertia_);

    ros::service::waitForService(wrench_service_name);

    //Setup service
    ros::ServiceClient wrench_service;
    wrench_service = nh.serviceClient<skye_ros::ApplyWrenchCogBf>(wrench_service_name, true);
    ros::Subscriber pos_sub = nh.subscribe (imu_topic, 1, &callback);

    ros::Rate r(50); // 50 hz
    while (1)
    {
        srv.request.start_time.nsec = 0;
        srv.request.duration.sec =  -1;

        temporaryWrench.force.x =  control_force_(0);
        temporaryWrench.force.y =  control_force_(1);
        temporaryWrench.force.z =  control_force_(2);
        temporaryWrench.torque.x = control_momentum_(0);
        temporaryWrench.torque.y = control_momentum_(1);
        temporaryWrench.torque.z = control_momentum_(2);

        srv.request.wrench = temporaryWrench;

        if (!wrench_service.call(srv)){
            ROS_ERROR("Failed to contact service. Could not pass wrench");
        }
        ros::spinOnce();
        r.sleep();
    }

}
