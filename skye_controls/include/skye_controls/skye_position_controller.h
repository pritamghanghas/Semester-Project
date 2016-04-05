#ifndef SKYE_POSITION_CONTROLLER_H
#define SKYE_POSITION_CONTROLLER_H

#include <Eigen/Eigen>

class PositionController
{
public:
    PositionController();

    Eigen::Vector3d computeForce(Eigen::Vector3d error, Eigen::Vector3d output_force);

};

#endif // SKYE_POSITION_CONTROLLER_H
