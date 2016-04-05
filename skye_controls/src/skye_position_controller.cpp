#include <skye_controls/skye_position_controller.h>

PositionController::PositionController(){
}

Eigen::Vector3d PositionController::computeForce(Eigen::Vector3d error, Eigen::Vector3d output_force){
    
    double kp = 2;
    output_force(0) = kp*error(0);
    output_force(1) = kp*error(1);
    output_force(2) = kp*error(2);
    return output_force;
}
