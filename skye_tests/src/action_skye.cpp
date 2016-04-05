#include <skye_tests/action_skye.h>

Actions_skye::Actions_skye(int a_time_in_ms_, Skye *a_skye) //ros::ServiceClient wrench_service,
{
//    ROS_INFO("About to subscribe");
//    ROS_INFO("subscribed!!!");
    zero << 0,0,0;
    skye = *a_skye;
    duration_in_ms_ = a_time_in_ms_;


//    ros::Subscriber pos_sub = nh.subscribe ("/skye_ros/sensor_msgs/imu_ned", 1, &Skye::acceleration_callback, &skye);


}

bool Actions_skye::poke(Eigen::Vector3d force, int duration){
  bool result = true;
  result = result && skye.apply_force(force);
//  usleep(duration);
//  result = result && skye.apply_force(zero);
  return result;
}

bool Actions_skye::twist(Eigen::Vector3d torque, int duration){
    bool result = true;
    result = result && skye.apply_torque(torque);
//    usleep(duration);
//    result = result && skye.apply_torque(zero);
    return result;
}

bool Actions_skye::move(Eigen::Vector3d force, Eigen::Vector3d torque, int duration){
    bool result = true;
    result = result && skye.apply_wrench(force, torque);
//    usleep(duration);
//    result = result && skye.apply_wrench(zero, zero);
    return result;

}

int Actions_skye::duration_in_ms(){
    return duration_in_ms_;
}


void Actions_skye::poke_callback(const geometry_msgs::Wrench::ConstPtr& msg ){
    Eigen::Vector3d force;
    force << msg->force.x, msg->force.y, msg->force.z;
    this->poke(force, duration_in_ms_);
}

void Actions_skye::twist_callback(const geometry_msgs::Wrench::ConstPtr& msg ){
    Eigen::Vector3d torque;
    torque << msg->torque.x, msg->torque.y, msg->torque.z;
    this->twist(torque, duration_in_ms_);
}
void Actions_skye::move_callback(const geometry_msgs::Wrench::ConstPtr& msg ){
    Eigen::Vector3d force, torque;
    force << msg->force.x, msg->force.y, msg->force.z;
    torque << msg->torque.x, msg->torque.y, msg->torque.z;
    this->move(force, torque, duration_in_ms_);

}

