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

inline void vectorFromSkewMatrix(Eigen::Matrix3d& skew_matrix, Eigen::Vector3d* vector) {
  *vector << skew_matrix(2, 1), skew_matrix(0,2), skew_matrix(1, 0);
}


void SkyeGeometricController::InitializeParams() {
    mass_ = 4;

    k_x_ << 1.5, 1.5, 1.5;
    k_v_ << 0.5, 0.5, 0.5;
    k_omega_ << 0.2, 0.2, 0.2;
    k_R_ << 0.8, 0.8, 0.8;

    desired_position_ << 0,0,-3;
    desired_velocity_ << 0,0,0;
    desired_angular_velocity_ << 0,0,0;
    desired_angular_acceleration_ << 0,0,0;
    desired_acceleration_ << 0,0,0;

    R_des_ << 1,0,0,
            0,1,0,
            0,0,1;
    inertia_ << 8.9927, 0.1046, -0.0843,
            0.1046, 10.1374, 0.0243,
            -0.0843, 0.0243, 10.1823;

}

void SkyeGeometricController::updateParameters(Eigen::Vector3d & position_,
                                               Eigen::Vector3d & velocity_,
                                               Eigen::Quaterniond & orientation_,
                                               Eigen::Vector3d & a_angular_velocity_){
    R_ = orientation_.toRotationMatrix();
    position_error_ << desired_position_(0) - position_(0),
            desired_position_(1) - position_(1),
            desired_position_(2) - position_(2);

    velocity_error_ << desired_velocity_(0) - velocity_(0),
            desired_velocity_(1) - velocity_(1),
            desired_velocity_(2) - velocity_(2);

    angular_velocity_ = a_angular_velocity_;

    /*********************************************************************************/
    Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des_.transpose() * R_ - R_.transpose() * R_des_);
    vectorFromSkewMatrix(angle_error_matrix, &attitude_error_);
    /*********************************************************************************/



    angular_velocity_error_ =  angular_velocity_ - R_.transpose() * R_des_ * desired_angular_velocity_;

}

void SkyeGeometricController::computeForce(Eigen::Vector3d & output_force_){

    output_force_ = R_*(k_x_.cwiseProduct(position_error_) + k_v_.cwiseProduct(velocity_error_) - mass_*desired_acceleration_ ) ;

}

void SkyeGeometricController::computeMomentum(Eigen::Vector3d & output_momentum_){

    output_momentum_ = -k_R_.cwiseProduct(attitude_error_) - k_omega_.cwiseProduct(angular_velocity_error_) + angular_velocity_.cross(inertia_*angular_velocity_)
                       - inertia_*(angular_velocity_.cross(R_.transpose()*(R_des_*desired_velocity())) - R_.transpose()*(R_des_*desired_angular_acceleration_));
}

