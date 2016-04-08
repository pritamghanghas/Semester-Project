/* Author: Marco Zorzi
 * Description: Controller class that computes force and momentum
 *              for Skye.
 * */

#include <skye_controls/skye_geometric_controller.h>

SkyeGeometricController::SkyeGeometricController(){
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

void SkyeGeometricController::initializeParams(double input_k_x_,
                                               double input_k_v_,
                                               double input_k_omega_,
                                               double input_k_R_,
                                               double input_mass_,
                                               Eigen::Vector3d & input_desired_position_,
                                               Eigen::Vector3d & input_desired_velocity_,
                                               Eigen::Vector3d & input_desired_angular_velocity_,
                                               Eigen::Vector3d & input_desired_angular_acceleration_,
                                               Eigen::Vector3d & input_desired_acceleration_,
                                               Eigen::Matrix3d inertia_) {

    k_x_ = input_k_x_;
    k_v_ = input_k_v_;
    k_omega_ = input_k_omega_;
    k_R_ = input_k_R_;
    mass_ = input_mass_;

    desired_position_ = input_desired_position_;
    desired_velocity_ = input_desired_velocity_;
    desired_angular_velocity_ = input_desired_angular_velocity_;
    desired_angular_acceleration_ = input_desired_angular_acceleration_;
    desired_acceleration_ = input_desired_acceleration_;


    // To make the tuning independent of the inertia matrix we divide here.
    normalized_k_R_ << k_R_, k_R_, k_R_;
    normalized_k_R_ = normalized_k_R_.transpose()*inertia_.inverse();

    // To make the tuning independent of the inertia matrix we divide here.
    normalized_k_omega_ << k_omega_,k_omega_,k_omega_;
    normalized_k_omega_ = normalized_k_omega_.transpose()*inertia_.inverse();

    //    R_des_ << 1,0,0,
    //            0,1,0,
    //            0,0,1;

}

void SkyeGeometricController::updateParameters(Eigen::Vector3d & position_,
                                               Eigen::Vector3d & velocity_,
                                               Eigen::Quaterniond & orientation_,
                                               Eigen::Vector3d & a_angular_velocity_){
    R_ = orientation_.inverse().matrix();

    position_error_ << desired_position_(0) - position_(0),
            desired_position_(1) - position_(1),
            desired_position_(2) - position_(2);

    velocity_error_ << desired_velocity_(0) - velocity_(0),
            desired_velocity_(1) - velocity_(1),
            desired_velocity_(2) - velocity_(2) ;

    position_error_ = R_*position_error_;
    velocity_error_ = R_*velocity_error_;

    //    angular_velocity_ = a_angular_velocity_;

    /******************** DEBUG *************************/
    std::cout << "--------------------------------------------------" << std::endl <<
                 "position_: " << position_(0) <<
                 " | y: " << position_(1) <<
                 " | z: " << position_(2) <<
                 std::endl << std::endl;


    std::cout << "position_error_: " << position_error_(0) <<
                 " | y: " << position_error_(1) <<
                 " | z: " << position_error_(2) <<
                 std::endl << std::endl;

    std::cout << "velocity_error_: " << velocity_error_(0) <<
                 " | y: " << velocity_error_(1) <<
                 " | z: " << velocity_error_(2) <<
                 std::endl << std::endl;

    std::cout << "R_:" << R_ << std::endl << std::endl;


    /******************** END DEBUG *************************/


    /*********************************************************************************
    Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des_.transpose() * R_ - R_.transpose() * R_des_);
    vectorFromSkewMatrix(angle_error_matrix, &attitude_error_);
    /*********************************************************************************/

    //    angular_velocity_error_ =  angular_velocity_ - R_.transpose() * R_des_ * desired_angular_velocity_;

}

void SkyeGeometricController::computeForce(Eigen::Vector3d & output_force_){

    output_force_ = (k_x_*(position_error_) + k_v_*(velocity_error_) - mass_*desired_acceleration_ );
    std::cout << "errors:" << std::endl << output_force_ << std::endl;
}

void SkyeGeometricController::computeMomentum(Eigen::Vector3d & output_momentum_){

    //    output_momentum_ = -k_R_.cwiseProduct(attitude_error_) - k_omega_.cwiseProduct(angular_velocity_error_) + angular_velocity_.cross(inertia_*angular_velocity_)
    //                       - inertia_*(angular_velocity_.cross(R_.transpose()*(R_des_*desired_velocity())) - R_.transpose()*(R_des_*desired_angular_acceleration_));
}

