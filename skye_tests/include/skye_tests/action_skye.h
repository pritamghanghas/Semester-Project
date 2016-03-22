#ifndef ACTION_SKYE_H
#define ACTION_SKYE_H

#include <skye_tests/skye.h>

class Actions_skye
{
public:
    Actions_skye(ros::ServiceClient wrench_service);

    bool poke(Eigen::Vector3d force, int duration);
    bool twist(Eigen::Vector3d torque, int duration);
    bool move(Eigen::Vector3d force, Eigen::Vector3d torque, int duration);


private:
    Skye skye;
    Eigen::Vector3d zero;
};

#endif // ACTION_SKYE_H
