/*
 * Author: Marco Zorzi
 * Description: Skye interface to interact with gazebo
 *
 * TODO: add callbacks to get acceleration, position and velocity of Skye.
 *
 * */

#include <skye_tests/skye.h>

Skye::Skye() {
}

bool Skye::init(ros::ServiceClient a_wrench_service){

    wrench_service = a_wrench_service;
    srv.request.start_time.nsec = 0;
    srv.request.duration.sec =  -1;
}

bool Skye::apply_force(Eigen::Vector3d force){

    Eigen::Vector3d torque;
    torque << 0,0,0;
    return apply_wrench(force, torque);
}

bool Skye::apply_torque(Eigen::Vector3d torque){

    Eigen::Vector3d force;
    force << 0,0,0;
    return apply_wrench(force, torque);
}

bool Skye::apply_wrench(Eigen::Vector3d force, Eigen::Vector3d torque){

    geometry_msgs::Wrench temporaryWrench;
    temporaryWrench.force.x = force(0);
    temporaryWrench.force.y = force(1);
    temporaryWrench.force.z = force(2);
    temporaryWrench.torque.x = torque(0);
    temporaryWrench.torque.y = torque(1);
    temporaryWrench.torque.z = torque(2);

    srv.request.wrench = temporaryWrench;

    if (wrench_service.call(srv)){
        ROS_INFO("Success: %x", srv.response.success);
    }
    else {
        ROS_ERROR("Failed to contact service. Could not pass wrench");
    }

    return srv.response.success;
}

