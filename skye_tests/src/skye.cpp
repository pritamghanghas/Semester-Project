/*
 * Author: Marco Zorzi
 * Description: Skye interface to interact with gazebo
 *
 * TODO: add callbacks to get acceleration, position and velocity of Skye.
 *
 * */

#include <skye_tests/skye.h>

Skye::Skye() {
    srv.request.start_time.nsec = 0;
    srv.request.duration.sec =  -1;
    timestamp_ = 0.01;
    zero << 0,0,0;
}

bool Skye::init(ros::ServiceClient a_wrench_service){

    wrench_service = a_wrench_service;

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

void Skye::acceleration_callback(const sensor_msgs::Imu::ConstPtr& msg){

    angular_velocity_ << msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z;

    Eigen::Vector3d new_acceleration_;
    new_acceleration_<< msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z - 9.81;

    Eigen::Vector3d new_velocity_;


    new_velocity_(0) = velocity_(0) + timestamp_*(new_acceleration_(0) - acceleration_(0)/2);
    new_velocity_(1) = velocity_(1) + timestamp_*(new_acceleration_(1) - acceleration_(1)/2);
    new_velocity_(2) = velocity_(2) + timestamp_*(new_acceleration_(2) - acceleration_(2)/2);

    position_(0) = position_(0) + timestamp_*(new_velocity_(0) - velocity_(0)/2);
    position_(1) = position_(1) + timestamp_*(new_velocity_(1) - velocity_(1)/2);
    position_(2) = position_(2) + timestamp_*(new_velocity_(2) - velocity_(2)/2);

    std::cout << //"counter:" << counter_ <<
                 "x: " << position_(0) <<
                 " | y: " << position_(1) <<
                 " | z: " << position_(2) <<
                 std::endl;

//    std::cout << //"counter:" << counter_ <<
//                 "vel(0): " << velocity_(0) <<
//                 " | time_: " << timestamp_ <<
//                 " | new_acc: " << new_acceleration_(0) <<
//                 " | acc: " << acceleration_(0) <<
//                 " | diff/2: " << new_acceleration_(0) - acceleration_(0)/2 <<
//                 " | new_vel(0): " << new_velocity_(0) <<
//                 std::endl;

    velocity_ = new_velocity_;
    acceleration_ = new_acceleration_;

    if (start_counting_) {
       counter_++;
       std::cout << "counter:" << counter_ << std::endl;

    }

    // CHECK THIS AND IMPROVE IT!!!!
    if(counter_>200){
        start_counting_ = false;
        counter_ = 0;
        this->apply_wrench(zero,zero);
    }

}
