/*
 * Author: Marco Zorzi
 * Description: Main cpp file to start a ROS node that
 * performs actions on the SKye Gazebo model.
 *
 * TODO: improve action input through a ROS topic
 * so the test can be run more easily and repeatedly.
 * */

#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <skye_ros/ApplyWrenchCogNed.h>
#include <skye_tests/action_skye.h>
#include <geometry_msgs/Wrench.h>


int main (int argc, char** argv) {

    std::printf("-----------------------------\n");
    std::printf("------ TESTS LAUNCHED -------\n");
    std::printf("-----------------------------\n");

    ros::init (argc, argv, "skye_tests_node");
    ros::NodeHandle nh;


    //Declare parameters to be imported
    std::string wrench_service_name;
    int sleeping_time, is_poke_requested,
        is_twist_requested, is_move_requested,
        poke_duration_in_sec, twist_duration_in_sec,
        move_duration_in_sec;
    double force_x, force_y, force_z, torque_x, torque_y, torque_z;

    //Get the parameters
    bool read_all_parameters = nh.getParam("wrench_service_name", wrench_service_name) &&
            nh.getParam("sleeping_time", sleeping_time) &&
            nh.getParam("is_poke_requested", is_poke_requested) &&
            nh.getParam("is_twist_requested", is_twist_requested) &&
            nh.getParam("is_move_requested", is_move_requested) &&
            nh.getParam("poke_duration_in_sec", poke_duration_in_sec) &&
            nh.getParam("twist_duration_in_sec", twist_duration_in_sec) &&
            nh.getParam("move_duration_in_sec", move_duration_in_sec) &&
            nh.getParam("force_x", force_x) &&
            nh.getParam("force_y", force_y) &&
            nh.getParam("force_z", force_z) &&
            nh.getParam("torque_x", torque_x) &&
            nh.getParam("torque_y", torque_y) &&
            nh.getParam("torque_z", torque_z);
    //Check if the parameters were correctly imported
    if (read_all_parameters) {
        ROS_INFO("Parameters imported" );
    }
    else{
        ROS_ERROR("Parameters import failed");
    }

    //Setup service
    ros::ServiceClient wrench_service;
    wrench_service = nh.serviceClient<skye_ros::ApplyWrenchCogNed>("skye_ros/apply_wrench_cog_ned", true);

    //Setup F and M vectors
    Eigen::Vector3d force;
    force << force_x, force_y, force_z;
    Eigen::Vector3d torque;
    torque << torque_x, torque_y, torque_z;
    ROS_INFO("Vectors packed");

    //Wait until everything else is setup
    //TODO: Fix this with some better code.
    //TODO: Talk to Marco T to better understand gazebo boot proc.
    std::cout << "going to sleep.." << std::endl;
    usleep(sleeping_time);
    std::cout << "Just woke up" << std::endl;

    ROS_INFO("started waiting for service");
    ros::service::waitForService(wrench_service_name);

    Skye skye;
    skye.init(wrench_service);

    //Create skye actions
    Actions_skye action(poke_duration_in_sec*1000000, &skye);

    if(is_poke_requested) {
        ROS_INFO("Poking");
        action.poke(force, poke_duration_in_sec*1000000);
    }
    else if(is_twist_requested){
        ROS_INFO("Twisting");
        action.twist(torque, twist_duration_in_sec*1000000);
    }
    else if(is_move_requested){
        ROS_INFO("Moving Skye");
        action.move(force, torque, move_duration_in_sec*1000000);
    }

    ros::Publisher poke_pub = nh.advertise<geometry_msgs::Wrench>("/skye_tests/poke", 1, true);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Wrench>("/skye_tests/twist", 1, true);
    ros::Publisher move_pub = nh.advertise<geometry_msgs::Wrench>("/skye_tests/move", 1, true);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber poke_sub = nh.subscribe ("/skye_tests/poke", 1, &Actions_skye::poke_callback, &action);
    ros::Subscriber twist_sub = nh.subscribe ("/skye_tests/twist", 1, &Actions_skye::twist_callback, &action);
    ros::Subscriber move_sub = nh.subscribe ("/skye_tests/move", 1, &Actions_skye::move_callback, &action);

    ros::Subscriber pos_sub = nh.subscribe ("/skye_ros/sensor_msgs/imu_ned", 1, &Skye::acceleration_callback, &skye);


    ros::spin();

}
