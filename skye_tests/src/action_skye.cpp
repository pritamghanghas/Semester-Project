#include <skye_tests/action_skye.h>

Actions_skye::Actions_skye(ros::ServiceClient wrench_service)
{
    zero << 0,0,0;
    skye.init(wrench_service);
}

bool Actions_skye::poke(Eigen::Vector3d force, int duration){
  bool result = true;
  result = result && skye.apply_force(force);
  usleep(duration);
  result = result && skye.apply_force(zero);
  return result;
}

bool Actions_skye::twist(Eigen::Vector3d torque, int duration){
    bool result = true;
    result = result && skye.apply_torque(torque);
    usleep(duration);
    result = result && skye.apply_torque(zero);
    return result;
}

bool Actions_skye::move(Eigen::Vector3d force, Eigen::Vector3d torque, int duration){
    bool result = true;
    result = result && skye.apply_wrench(force, torque);
    usleep(duration);
    result = result && skye.apply_wrench(zero, zero);
    return result;

}
