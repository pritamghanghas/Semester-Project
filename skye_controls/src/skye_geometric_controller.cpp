/* Author: Marco Zorzi
 * Description: Controller class that computes force and momentum
 *              for Skye.
 * */

#include <skye_controls/skye_geometric_controller.h>

SkyeGeometricController::SkyeGeometricController(){
    InitializeParams();
}

Eigen::Vector3d SkyeGeometricController::desired_position(){
    return desired_position_;
}

Eigen::Vector3d SkyeGeometricController::desired_velocity(){
    return desired_velocity_;
}


void SkyeGeometricController::InitializeParams() {
    mass_ = 4;

    k_x_ << 1, 1, 1;
    k_v_ << 1, 1, 1;
    k_omega_ << 1, 1, 1;
    k_R_ << 1, 1, 1;

    desired_position_ << 0,0,-3;
    desired_velocity_ << 0,0,0;
    desired_angular_velocity_ << 0,0,0;
    desired_angular_acceleration_ << 0,0,0;
    desired_acceleration_ << 0,0,0;

    desired_attitude_ << 1,0,0,
            0,1,0,
            0,0,1;
    inertia_ << 8.9927, 0.1046, -0.0843,
            0.1046, 10.1374, 0.0243,
            -0.0843, 0.0243, 10.1823;

}

void SkyeGeometricController::computeForce(Eigen::Vector3d & position_error_,
                                           Eigen::Vector3d & velocity_error_,
                                           Eigen::Matrix3d & R_,
                                           Eigen::Vector3d & output_force_){

    output_force_ = R_*(k_x_.cwiseProduct(position_error_) + k_v_.cwiseProduct(velocity_error_) - mass_*desired_acceleration_ ) ;

}

