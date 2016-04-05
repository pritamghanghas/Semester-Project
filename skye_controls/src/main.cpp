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

Eigen::Vector3d position_, acceleration_, velocity_, angular_velocity_, reference_pos_;
ros::ServiceClient wrench_service;
double timestamp_ = 0.01;

void callback(const sensor_msgs::Imu::ConstPtr& msg){


    angular_velocity_ << msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z;

    Eigen::Vector3d new_acceleration_;
    new_acceleration_<< msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z;

    Eigen::Vector3d new_velocity_, control_input_;


    new_velocity_(0) = velocity_(0) + timestamp_*(new_acceleration_(0) - acceleration_(0)/2);
    new_velocity_(1) = velocity_(1) + timestamp_*(new_acceleration_(1) - acceleration_(1)/2);
    new_velocity_(2) = velocity_(2) + timestamp_*(new_acceleration_(2) - acceleration_(2)/2);

    position_(0) = position_(0) + timestamp_*(new_velocity_(0) - velocity_(0)/2);
    position_(1) = position_(1) + timestamp_*(new_velocity_(1) - velocity_(1)/2);
    position_(2) = position_(2) + timestamp_*(new_velocity_(2) - velocity_(2)/2);

    velocity_ = new_velocity_;
    acceleration_ = new_acceleration_;


    control_input_ << 0,0,0;
    PositionController controller;

    Eigen::Vector3d error;
    error << reference_pos_(0) - position_(0),
            reference_pos_(1) - position_(1),
            reference_pos_(2) - position_(2);
    control_input_ = controller.computeForce(error, control_input_);


    std::cout << //"counter:" << counter_ <<
                 "control_input_: " << control_input_(0) <<
                 " | y: " << control_input_(1) <<
                 " | z: " << control_input_(2) <<
                 std::endl;



    skye_ros::ApplyWrenchCogBf srv;
    srv.request.start_time.nsec = 0;
    srv.request.duration.sec =  -1;

    geometry_msgs::Wrench temporaryWrench;
    temporaryWrench.force.x = 1; // control_input_(0);
    temporaryWrench.force.y = 1; // control_input_(1);
    temporaryWrench.force.z = 1; // control_input_(2);
    temporaryWrench.torque.x = 0;
    temporaryWrench.torque.y = 0;
    temporaryWrench.torque.z = 0;

    srv.request.wrench = temporaryWrench;

    if (wrench_service.call(srv)){
        ROS_INFO("Success: %x", srv.response.success);
    }
    else {
        ROS_ERROR("Failed to contact service. Could not pass wrench");
    }


}


int main (int argc, char** argv) {
    position_ << 0,0,0;
    acceleration_ << 0,0,0;
    velocity_ << 0,0,0;
    angular_velocity_ << 0,0,0;
    reference_pos_ << 0,0,0;

    usleep(20000000);
    ros::init(argc, argv, "skye_position_controller_node");
    ros::NodeHandle nh;

    ros::service::waitForService("skye_ros/apply_wrench_cog_bf");

    //Setup service
    ros::ServiceClient wrench_service;
    wrench_service = nh.serviceClient<skye_ros::ApplyWrenchCogBf>("skye_ros/apply_wrench_cog_bf", true);

    ros::Subscriber pos_sub = nh.subscribe ("/skye_ros/sensor_msgs/imu_bf", 1, &callback);

    ros::spin();
}
