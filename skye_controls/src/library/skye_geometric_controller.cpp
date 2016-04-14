/* Author: Marco Zorzi
 * Description: Controller class that computes force and momentum
 *              for Skye.
 * */

#include <skye_controls/skye_geometric_controller.h>

SkyeGeometricController::SkyeGeometricController(){
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

//TODO: declare struct to pass parameters
void SkyeGeometricController::InitializeParams(SkyeParameters param){

    //Save the parameters
    number_of_actuators_ = param.input_number_of_actuators_;
    k_x_ = param.input_k_x_;
    k_v_ = param.input_k_v_;
    k_omega_ = param.input_k_omega_;
    k_R_ = param.input_k_R_;
    mass_ = param.input_mass_;
    radius_ = param.input_radius_;
    maximum_force_cog_ = param.input_maximum_force_cog_;
    distance_integrator_treshold_ = param.input_distance_integrator_treshold_;
    attitude_integrator_treshold_ = param.input_attitude_integrator_treshold_;
    inertia_ = param.input_inertia_;
    R_des_ = param.input_R_des_;
    desired_position_ = param.input_desired_position_;
    desired_velocity_ = param.input_desired_velocity_;
    desired_angular_velocity_ = param.input_desired_angular_velocity_;
    desired_angular_acceleration_ = param.input_desired_angular_acceleration_;
    desired_acceleration_ = param.input_desired_acceleration_;
    //Compute normalized gains for the first time
    this->ComputeNormalizedGains();

    //Compute maximum acceleration cog
    maximum_acceleration_cog_ = (maximum_force_cog_/number_of_actuators_) * radius_;

}

void SkyeGeometricController::UpdateParameters(Eigen::Vector3d & position_,
                                               Eigen::Vector3d & velocity_,
                                               Eigen::Quaterniond & orientation_,
                                               Eigen::Vector3d & a_angular_velocity_){

    // Calculate rotation matrix from bf to NED
    R_ = orientation_.inverse().matrix();

    //Calculate errors
    position_error_ << desired_position_(0) - position_(0),
            desired_position_(1) - position_(1),
            desired_position_(2) - position_(2);

    velocity_error_ << desired_velocity_(0) - velocity_(0),
            desired_velocity_(1) - velocity_(1),
            desired_velocity_(2) - velocity_(2) ;
    // Convert errors frame
    position_error_ = R_*position_error_;
    velocity_error_ = R_*velocity_error_;

    // Calculate attitude control errors as per Lee paper.
    Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des_.transpose() * R_ - R_.transpose() * R_des_);
    vectorFromSkewMatrix(angle_error_matrix, &attitude_error_);

    angular_velocity_ = a_angular_velocity_;
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

    std::cout << "pos_err_norm: " << position_error_.norm() <<
                 std::endl << std::endl;

    std::cout << "integrator_force_error_: " << integrator_force_error_(0) <<
                 " | y: " << integrator_force_error_(1) <<
                 " | z: " << integrator_force_error_(2) <<
                 std::endl << std::endl;

            // ATTITUDE

    std::cout << "attitude_error_: " << attitude_error_(0) <<
                 " | y: " << attitude_error_(1) <<
                 " | z: " << attitude_error_(2) <<
                 std::endl << std::endl;


    std::cout << "angular_velocity_error_: " << angular_velocity_error_(0) <<
                 " | y: " << angular_velocity_error_(1) <<
                 " | z: " << angular_velocity_error_(2) <<
                 std::endl << std::endl;

    std::cout << "atti_err_norm: " << attitude_error_.norm() <<
                 std::endl << std::endl;

    std::cout << "integrator_acceleration_error_: " << integrator_acceleration_error_(0) <<
                 " | y: " << integrator_acceleration_error_(1) <<
                 " | z: " << integrator_acceleration_error_(2) <<
                 std::endl << std::endl;


    //std::cout << "R_:" << R_ << std::endl << std::endl;
    /******************** END DEBUG *************************/

}

void SkyeGeometricController::UpdateGains(double k_x,
                                          double k_v,
                                          double k_if,
                                          double k_im,
                                          double k_R,
                                          double k_omega){
    k_x_ = k_x;
    k_v_ = k_v;
    k_R_ = k_R;
    k_omega_ = k_omega;
    k_if_ = k_if;
    k_im_ = k_im;
    this->ComputeNormalizedGains();

}

inline void SkyeGeometricController::SaturateForce(Eigen::Vector3d & output_force_){
    if(std::abs(output_force_(0)) > maximum_force_cog_){
        if(std::signbit(output_force_(0))) output_force_(0) = maximum_force_cog_*(-1);
        else output_force_(0) = maximum_force_cog_;
    }
    if(std::abs(output_force_(1)) > maximum_force_cog_){
        if(std::signbit(output_force_(1))) output_force_(1) = maximum_force_cog_*(-1);
        else output_force_(1) = maximum_force_cog_;
    }
    if(std::abs(output_force_(2)) > maximum_force_cog_){
        if(std::signbit(output_force_(2))) output_force_(2) = maximum_force_cog_*(-1);
        else output_force_(2) = maximum_force_cog_;
    }
}

inline void SkyeGeometricController::SaturateAcceleration(Eigen::Vector3d & output_acceleration_){
    if(std::abs(output_acceleration_(0)) > maximum_acceleration_cog_){
        if(std::signbit(output_acceleration_(0))) output_acceleration_(0) = maximum_acceleration_cog_*(-1);
        else output_acceleration_(0) = maximum_acceleration_cog_;
    }
    if(std::abs(output_acceleration_(1)) > maximum_acceleration_cog_){
        if(std::signbit(output_acceleration_(1))) output_acceleration_(1) = maximum_acceleration_cog_*(-1);
        else output_acceleration_(1) = maximum_acceleration_cog_;
    }
    if(std::abs(output_acceleration_(2)) > maximum_acceleration_cog_){
        if(std::signbit(output_acceleration_(2))) output_acceleration_(2) = maximum_acceleration_cog_*(-1);
        else output_acceleration_(2) = maximum_acceleration_cog_;
    }
}


void SkyeGeometricController::ComputeForce(Eigen::Vector3d & output_force_){

    output_force_ = (k_x_*(position_error_) + k_v_*(velocity_error_) - mass_*desired_acceleration_ );

    // Integrator with antiwindup
    if (position_error_.norm() < distance_integrator_treshold_ ) {
        output_force_ = output_force_ + k_if_ * integrator_force_error_;
        integrator_force_error_ = output_force_;
    }
    else{
        integrator_force_error_ << 0,0,0;
    }

    // Force saturation
    this->SaturateForce(output_force_);
}

void SkyeGeometricController::ComputeAcceleration(Eigen::Vector3d & output_acceleration_){

    output_acceleration_ = -normalized_k_R_.cwiseProduct(attitude_error_)
                       - normalized_k_omega_.cwiseProduct(angular_velocity_error_)
                       + angular_velocity_.cross(angular_velocity_);

    // Integrator with antiwindup
    if (attitude_error_.norm() < attitude_integrator_treshold_ ) {
        output_acceleration_ = output_acceleration_ + k_im_ * integrator_acceleration_error_;
        integrator_acceleration_error_ = output_acceleration_;
    }
    else{
        integrator_acceleration_error_ << 0,0,0;
    }

    // Acceleration saturation
    this->SaturateAcceleration(output_acceleration_);

}

