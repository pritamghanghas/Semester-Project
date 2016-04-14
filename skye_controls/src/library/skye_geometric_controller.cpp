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

inline void SkyeGeometricController::ComputeNormalizedGains(){
    // To make the tuning independent of the inertia matrix we divide here.
    normalized_k_R_ << k_R_, k_R_, k_R_;
    normalized_k_R_ = normalized_k_R_.transpose()*inertia_.inverse();

    // To make the tuning independent of the inertia matrix we divide here.
    normalized_k_omega_ << k_omega_,k_omega_,k_omega_;
    normalized_k_omega_ = normalized_k_omega_.transpose()*inertia_.inverse();

}

void SkyeGeometricController::InitializeParams(double input_k_x_,
                                               double input_k_v_,
                                               double input_k_omega_,
                                               double input_k_R_,
                                               double input_mass_,
                                               Eigen::Vector3d & input_desired_position_,
                                               Eigen::Vector3d & input_desired_velocity_,
                                               Eigen::Vector3d & input_desired_angular_velocity_,
                                               Eigen::Vector3d & input_desired_angular_acceleration_,
                                               Eigen::Vector3d & input_desired_acceleration_,
                                               Eigen::Matrix3d & input_inertia_,
                                               Eigen::Matrix3d & input_R_des_){

    k_x_ = input_k_x_;
    k_v_ = input_k_v_;
    k_omega_ = input_k_omega_;
    k_R_ = input_k_R_;
    mass_ = input_mass_;
    inertia_ = input_inertia_;
    R_des_ << input_R_des_;

    desired_position_ = input_desired_position_;
    desired_velocity_ = input_desired_velocity_;
    desired_angular_velocity_ = input_desired_angular_velocity_;
    desired_angular_acceleration_ = input_desired_angular_acceleration_;
    desired_acceleration_ = input_desired_acceleration_;


    this->ComputeNormalizedGains();

    std::cout << "--------------------------------------------------" << std::endl <<
                 "normalized_k_R_: " << normalized_k_R_(0) <<
                 " | y: " << normalized_k_R_(1) <<
                 " | z: " << normalized_k_R_(2) <<
                 std::endl << std::endl;

    std::cout << "--------------------------------------------------" << std::endl <<
                 "normalized_k_omega_: " << normalized_k_omega_(0) <<
                 " | y: " << normalized_k_omega_(1) <<
                 " | z: " << normalized_k_omega_(2) <<
                 std::endl << std::endl;


}

void SkyeGeometricController::UpdateParameters(Eigen::Vector3d & position_,
                                               Eigen::Vector3d & velocity_,
                                               Eigen::Quaterniond & orientation_,
                                               Eigen::Vector3d & a_angular_velocity_){

    // General calculations
    R_ = orientation_.inverse().matrix();

    //Position control parameters
    position_error_ << desired_position_(0) - position_(0),
            desired_position_(1) - position_(1),
            desired_position_(2) - position_(2);

    velocity_error_ << desired_velocity_(0) - velocity_(0),
            desired_velocity_(1) - velocity_(1),
            desired_velocity_(2) - velocity_(2) ;
    position_error_ = R_*position_error_;
    velocity_error_ = R_*velocity_error_;

    //Attitude control parameters
    angular_velocity_ = a_angular_velocity_;

    Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des_.transpose() * R_ - R_.transpose() * R_des_);
    vectorFromSkewMatrix(angle_error_matrix, &attitude_error_);

    angular_velocity_error_ =  angular_velocity_ - R_.transpose() * R_des_ * desired_angular_velocity_;


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

    std::cout << "attitude_error_: " << attitude_error_(0) <<
                 " | y: " << attitude_error_(1) <<
                 " | z: " << attitude_error_(2) <<
                 std::endl << std::endl;


    std::cout << "angular_velocity_error_: " << angular_velocity_error_(0) <<
                 " | y: " << angular_velocity_error_(1) <<
                 " | z: " << angular_velocity_error_(2) <<
                 std::endl << std::endl;

    //std::cout << "R_:" << R_ << std::endl << std::endl;
    /******************** END DEBUG *************************/

}

void SkyeGeometricController::UpdateGains(double k_x,
                                          double k_v,
                                          double k_R,
                                          double k_omega){
    k_x_ = k_x;
    k_v_ = k_v;
    k_R_ = k_R;
    k_omega_ = k_omega;
    this->ComputeNormalizedGains();

}


void SkyeGeometricController::ComputeForce(Eigen::Vector3d & output_force_){

    output_force_ = (k_x_*(position_error_) + k_v_*(velocity_error_) - mass_*desired_acceleration_ );
    std::cout << "errors:" << std::endl << output_force_ << std::endl;

    if(output_force_(0) > 15) output_force_(0) = 15;
    if(output_force_(1) > 15) output_force_(1) = 15;
    if(output_force_(2) > 15) output_force_(2) = 15;

}

void SkyeGeometricController::ComputeAcceleration(Eigen::Vector3d & output_momentum_){

    output_momentum_ = -normalized_k_R_.cwiseProduct(attitude_error_)
                       - normalized_k_omega_.cwiseProduct(angular_velocity_error_)
                       + angular_velocity_.cross(angular_velocity_);

}

