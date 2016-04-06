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
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/LinkState.h>

Eigen::Vector3d position_, acceleration_, velocity_, angular_velocity_, reference_pos_;
ros::ServiceClient wrench_service;
skye_ros::ApplyWrenchCogBf srv;
geometry_msgs::Wrench temporaryWrench;
Eigen::Vector3d error, control_input_;
PositionController controller;


double timestamp_ = 0.01;

void callback(const gazebo_msgs::LinkState::ConstPtr& msg){

    position_ << msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z;

    std::cout << //"counter:" << counter_ <<
                 "position_: " << position_(0) <<
                 " | y: " << position_(1) <<
                 " | z: " << position_(2) <<
                 std::endl;

    error << reference_pos_(0) - position_(0),
            reference_pos_(1) - position_(1),
            reference_pos_(2) - position_(2);
    control_input_ = controller.computeForce(error, control_input_);

    std::cout << //"counter:" << counter_ <<
                 "control_input_: " << control_input_(0) <<
                 " | y: " << control_input_(1) <<
                 " | z: " << control_input_(2) <<
                 std::endl;



}


int main (int argc, char** argv) {
    position_ << 0,0,0;
    acceleration_ << 0,0,0;
    velocity_ << 0,0,0;
    angular_velocity_ << 0,0,0;
    reference_pos_ << 0,0,-3;



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

        temporaryWrench.force.x =  control_input_(0);
        temporaryWrench.force.y =  control_input_(1);
        temporaryWrench.force.z =  control_input_(2);
        temporaryWrench.torque.x = 0;
        temporaryWrench.torque.y = 0;
        temporaryWrench.torque.z = 0;

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
