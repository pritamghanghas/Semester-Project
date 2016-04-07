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

Eigen::Vector3d position_, velocity_, angular_velocity_;
ros::ServiceClient wrench_service;
skye_ros::ApplyWrenchCogBf srv;
geometry_msgs::Wrench temporaryWrench;

Eigen::Vector3d control_force_, control_momentum_;
PositionController controller;
SkyeGeometricController geometric_controller;
Eigen::Matrix3d R_;
Eigen::Quaterniond orientation_;
Eigen::Matrix3d R_90_z;


double timestamp_ = 0.01;

void callback(const gazebo_msgs::LinkState::ConstPtr& msg){

    position_ << msg->pose.position.x,
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

    //    R_ = orientation_.toRotationMatrix();
    //    std::cout << "R_: " << R_ << std::endl;
    //    R_ = R_90_z;

    //    std::cout << "--------------------------------------------------" << std::endl <<
    //                 "position_: " << position_(0) <<
    //                 " | y: " << position_(1) <<
    //                 " | z: " << position_(2) <<
    //                 std::endl;

    //    error << reference_pos_(0) - position_(0),
    //            reference_pos_(1)- position_(1),
    //            reference_pos_(2) - position_(2);
    //    control_input_ = controller.computeForce(error);


    //    position_error_ << geometric_controller.desired_position()(0) - position_(0),
    //            geometric_controller.desired_position()(1) - position_(1),
    //            geometric_controller.desired_position()(2) - position_(2);

    //    velocity_error_ << geometric_controller.desired_velocity()(0) - velocity_(0),
    //            geometric_controller.desired_velocity()(1) - velocity_(1),
    //            geometric_controller.desired_velocity()(2) - velocity_(2);


    geometric_controller.updateParameters(position_, velocity_, orientation_, angular_velocity_);
    geometric_controller.computeForce(control_force_);
    geometric_controller.computeMomentum(control_momentum_);

    std::cout << "control_input_: " << control_force_(0) <<
                 " | y: " << control_force_(1) <<
                 " | z: " << control_force_(2) <<
                 std::endl;
}

int main (int argc, char** argv) {
    R_90_z << 0, 1, 0,
            -1, 0,  0,
            0, 0, 1;


    //    usleep(20000000);
    ros::init(argc, argv, "skye_position_controller_node");
    ros::NodeHandle nh;

    ROS_INFO("ros started");
    //Declare parameters to be imported
    std::string wrench_service_name, imu_topic;

    //Get the parameters
    bool read_all_parameters = nh.getParam("wrench_service_name", wrench_service_name) &&
                               nh.getParam("imu_topic", imu_topic) ;

    if (read_all_parameters) {
        ROS_INFO("Parameters parsed");
    }
    else
        ROS_ERROR("Parameters not imported");

    ros::service::waitForService(wrench_service_name);

    ROS_INFO("I got the service");
    //Setup service
    ros::ServiceClient wrench_service;
    wrench_service = nh.serviceClient<skye_ros::ApplyWrenchCogBf>(wrench_service_name, true);

    ROS_INFO("About to subscribe");

    ros::Subscriber pos_sub = nh.subscribe (imu_topic, 1, &callback);

    ros::Rate r(10); // 10 hz
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


    ROS_INFO("Starting to spin");

    ros::spin();
}
