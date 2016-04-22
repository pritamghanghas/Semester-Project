/* Author: Marco Zorzi
 * Description: Controller class that computes force and momentum
 *              for Skye.
 * */

#include <skye_controls/skye_geometric_controller.h>

SkyeGeometricController::SkyeGeometricController(){
}

inline void SkyeGeometricController::VectorFromSkewMatrix(Eigen::Matrix3d &skew_matrix, Eigen::Vector3d *vector) {
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

void SkyeGeometricController::InitializeParams(const SkyeParameters param){

    //Save the parameters
    number_of_actuators_ = param.input_number_of_actuators;
    k_x_ = param.input_k_x;
    k_v_ = param.input_k_v;
    k_omega_ = param.input_k_omega;
    k_R_ = param.input_k_R;
    mass_ = param.input_mass;
    radius_ = param.input_radius;
    maximum_force_cog_bf_ = param.input_maximum_force_cog;
    maximum_force_integrator_ = param.input_maximum_force_integrator;
    maximum_momentum_integrator_ = param.input_maximum_momentum_integrator;

    distance_integrator_treshold_ = param.input_distance_integrator_treshold;
    attitude_integrator_treshold_ = param.input_attitude_integrator_treshold;
    inertia_ = param.input_inertia;
    R_des_if_ = param.input_R_des_if;
    desired_position_if_ = param.input_desired_position_if;
    desired_velocity_if_ = param.input_desired_velocity_if;
    desired_angular_velocity_bf_ = param.input_desired_angular_velocity_bf;
    desired_angular_acceleration_bf_ = param.input_desired_angular_acceleration_bf;
    desired_acceleration_if_ = param.input_desired_acceleration_if;

    //Compute normalized gains for the first time
    this->ComputeNormalizedGains();

    //Compute maximum acceleration cog
    maximum_acceleration_cog_bf_ = (maximum_force_cog_bf_/number_of_actuators_) * radius_;

}

void SkyeGeometricController::UpdateParameters(const Eigen::Vector3d &position_if,
                                               const Eigen::Vector3d &velocity_if,
                                               const Eigen::Quaterniond &orientation_if,
                                               const Eigen::Vector3d &a_angular_velocity_bf){

    // Calculate rotation matrix from NED(or inertial frame) to body fixed frame
    R_if_ = orientation_if.matrix();

    //Calculate errors
    position_error_if_ << desired_position_if_(0) - position_if(0),
            desired_position_if_(1) - position_if(1),
            desired_position_if_(2) - position_if(2);

    velocity_error_if_ << desired_velocity_if_(0) - velocity_if(0),
            desired_velocity_if_(1) - velocity_if(1),
            desired_velocity_if_(2) - velocity_if(2) ;

    // Convert errors frame
    position_error_bf_ = R_if_.transpose()*position_error_if_;
    velocity_error_bf_ = R_if_.transpose()*velocity_error_if_;

    // Calculate attitude control errors as per Lee paper.
    Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des_if_.transpose() * R_if_ - R_if_.transpose() * R_des_if_);
    VectorFromSkewMatrix(angle_error_matrix, &attitude_error_bf_);

    // Compute angular velocity error as per Lee paper.
    angular_velocity_ = a_angular_velocity_bf;
    angular_velocity_error_bf_ =  angular_velocity_ - R_if_.transpose() * R_des_if_ * desired_angular_velocity_bf_;

    /******************** DEBUG ************************
     * WHEN REMOVING THIS CODE DO NOT FORGET TO REMOVE IOSTREAM INCLUSION
     * IN THE HEADER FILE!!!!
     *
     */
    std::cout << "--------------------------------------------------" << std::endl <<
                 "position_: " << position_if(0) <<
                 " | y: " << position_if(1) <<
                 " | z: " << position_if(2) <<
                 std::endl << std::endl;


    std::cout << "position_error_: " << position_error_bf_(0) <<
                 " | y: " << position_error_bf_(1) <<
                 " | z: " << position_error_bf_(2) <<
                 std::endl << std::endl;

    std::cout << "velocity_error_: " << velocity_error_bf_(0) <<
                 " | y: " << velocity_error_bf_(1) <<
                 " | z: " << velocity_error_bf_(2) <<
                 std::endl << std::endl;

    std::cout << "pos_err_norm: " << position_error_bf_.norm() <<
                 std::endl << std::endl;

    std::cout << "integrator_force_error_: " << integrator_force_(0) <<
                 " | y: " << integrator_force_(1) <<
                 " | z: " << integrator_force_(2) <<
                 std::endl << std::endl;

    // ATTITUDE

    std::cout << "attitude_error_: " << attitude_error_bf_(0) <<
                 " | y: " << attitude_error_bf_(1) <<
                 " | z: " << attitude_error_bf_(2) <<
                 std::endl << std::endl;


    std::cout << "angular_velocity_error_: " << angular_velocity_error_bf_(0) <<
                 " | y: " << angular_velocity_error_bf_(1) <<
                 " | z: " << angular_velocity_error_bf_(2) <<
                 std::endl << std::endl;

    std::cout << "atti_err_norm: " << attitude_error_bf_.norm() <<
                 std::endl << std::endl;

    std::cout << "integrator_acceleration_error_: " << integrator_acceleration_(0) <<
                 " | y: " << integrator_acceleration_(1) <<
                 " | z: " << integrator_acceleration_(2) <<
                 std::endl << std::endl;


        std::cout << "R_if_:" << R_if_ << std::endl << std::endl;

        std::cout << "R_des_if_:" << R_des_if_ << std::endl << std::endl;
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

inline void SkyeGeometricController::SaturateVector(double a_threshold, Eigen::Vector3d *a_vector){
    if (std::abs((*a_vector)(0)) > a_threshold) {
        if(std::signbit((*a_vector)(0))) (*a_vector)(0) = a_threshold*(-1);
        else (*a_vector)(0) = a_threshold;
    }
    if (std::abs((*a_vector)(1)) > a_threshold) {
        if(std::signbit((*a_vector)(1))) (*a_vector)(1) = a_threshold*(-1);
        else (*a_vector)(1) = a_threshold;
    }
    if (std::abs((*a_vector)(2)) > a_threshold) {
        if(std::signbit((*a_vector)(2))) (*a_vector)(2) = a_threshold*(-1);
        else (*a_vector)(2) = a_threshold;
    }
}

void SkyeGeometricController::ComputeForce(Eigen::Vector3d *output_force_bf){

    *output_force_bf = (k_x_*(position_error_bf_) + k_v_*(velocity_error_bf_) - mass_*desired_acceleration_if_ );

    // Integrator with antiwindup
    if (position_error_bf_.norm() < distance_integrator_treshold_ ) {

        *output_force_bf = *output_force_bf + k_if_ * integrator_force_;
        integrator_force_ = *output_force_bf;
        SaturateVector(maximum_force_integrator_, &integrator_force_);
    } else {
        integrator_force_ << 0,0,0;
    }

    // Force saturation
    this->SaturateVector(maximum_force_cog_bf_, output_force_bf);
}

void SkyeGeometricController::ComputeAcceleration(Eigen::Vector3d *output_acceleration_bf){

    *output_acceleration_bf = -normalized_k_R_.cwiseProduct(attitude_error_bf_)
                              - normalized_k_omega_.cwiseProduct(angular_velocity_error_bf_)
                              + angular_velocity_.cross(angular_velocity_);

    // Integrator with antiwindup
    if (attitude_error_bf_.norm() < attitude_integrator_treshold_ ) {
        *output_acceleration_bf = *output_acceleration_bf + k_im_ * integrator_acceleration_;
        integrator_acceleration_ = *output_acceleration_bf;
        SaturateVector(maximum_momentum_integrator_, &integrator_acceleration_);

    } else {
        integrator_acceleration_ << 0,0,0;
    }

    // Acceleration saturation
    this->SaturateVector(maximum_acceleration_cog_bf_, output_acceleration_bf);

}

void SkyeGeometricController::UpdateDesiredPose(const Eigen::Vector3d &desired_position_if,
                                                const Eigen::Quaterniond &desired_orientation_if){
    desired_position_if_ = desired_position_if;
    R_des_if_ = desired_orientation_if.matrix();
}
