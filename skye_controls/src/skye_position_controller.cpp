#include <skye_controls/skye_position_controller.h>


PositionController::PositionController(){
}

Eigen::Vector3d PositionController::computeForce(Eigen::Vector3d error){
    Eigen::Vector3d output_force;
    double kp = 0.5;
    std::cout << "error: " << error(0) <<
                 " | y: " << error(1) <<
                 " | z: " << error(2) <<
                 std::endl;

    std::cout << "abs(error): " << std::abs(error(0)) <<
                 " | y: " << std::abs(error(1)) <<
                 " | z: " << std::abs(error(2)) <<
                 std::endl;

    if (std::abs(error(0))<0.001) {
        output_force(0) = 0;
    }
    else{
        output_force(0) = kp*error(0);
    }
    if (std::abs(error(1))<0.001) {
        output_force(1) = 0;
    }
    else{
        output_force(1) = kp*error(1);
    }
    if (std::abs(error(2))<0.001) {
        output_force(2) = 0;
    }
    else{
        output_force(2) = kp*error(2);
    }

    return output_force;
}
