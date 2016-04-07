#ifndef SKYE_POSITION_CONTROLLER_H
#define SKYE_POSITION_CONTROLLER_H

#include <Eigen/Eigen>
#include <stdlib.h>
#include <iostream>

class PositionController
{
public:
    PositionController();

    Eigen::Vector3d computeForce(Eigen::Vector3d error);

};

#endif // SKYE_POSITION_CONTROLLER_H
