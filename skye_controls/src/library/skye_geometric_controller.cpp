/* Author: Marco Zorzi
 * Description: Controller class that computes force and momentum
 *              for Skye.
 * */

#include <skye_controls/skye_geometric_controller.h>

//---------------------------------------------------------------------------------------------------------
SkyeGeometricController::SkyeGeometricController(){
    singularity_detected_ = false;
}

//---------------------------------------------------------------------------------------------------------
inline void SkyeGeometricController::VectorFromSkewMatrix(Eigen::Matrix3d &skew_matrix, Eigen::Vector3d *vector) {
    *vector << skew_matrix(2, 1), skew_matrix(0,2), skew_matrix(1, 0);
}

//---------------------------------------------------------------------------------------------------------
inline void SkyeGeometricController::ComputeNormalizedGains(){
    // To make the tuning independent of the inertia matrix we divide here.
    normalized_k_R_ << k_R_, k_R_, k_R_;
    normalized_k_R_ = normalized_k_R_.transpose()*inertia_.inverse();

    // To make the tuning independent of the inertia matrix we divide here.
    normalized_k_omega_ << k_omega_,k_omega_,k_omega_;
    normalized_k_omega_ = normalized_k_omega_.transpose()*inertia_.inverse();
}

//---------------------------------------------------------------------------------------------------------
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
    maximum_acceleration_integrator_ = param.input_maximum_momentum_integrator;

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

//---------------------------------------------------------------------------------------------------------
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

    //Convert current rotations to axis angles
    Eigen::AngleAxisd current_rotation, desired_rotation, temporary_rotation;
    current_rotation = R_if_;
    desired_rotation = R_des_if_;

    //Calculate the difference of the angles
    float diff = std::abs(current_rotation.angle() - desired_rotation.angle());

    //Check if a critical point rotation has been found
    if (diff >= M_PI_2 && !singularity_detected_) {

        //if there is a singularity keep track of it so to restore it afterwards
        singularity_detected_ = true;
        R_temp_if_ = R_des_if_;

        if (diff <= M_PI) { // in this case I am on the left side
            temporary_rotation.axis() = current_rotation.axis();
            temporary_rotation.angle() = current_rotation.angle() + M_PI_2;
            R_des_if_ = temporary_rotation;
        } else {
            temporary_rotation.axis() = current_rotation.axis();
            temporary_rotation.angle() = current_rotation.angle()  - 3*M_PI_2;
            R_des_if_ = temporary_rotation;
        }
    }

    // If I detected a singularity and I am now close enough to my final attitude
    if (diff <= M_PI_2 && singularity_detected_) {
        //I save that I don't have a singularity anymore
        singularity_detected_ = false;

        //Original goal restored
        R_des_if_ = R_temp_if_;
        R_temp_if_ << 0,0,0,
                0,0,0,
                0,0,0;
        temporary_rotation = R_temp_if_;
    }


    // Calculate attitude control errors as per Lee paper.
    Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des_if_.transpose() * R_if_ - R_if_.transpose() * R_des_if_);
    VectorFromSkewMatrix(angle_error_matrix, &attitude_error_bf_);

    // Compute angular velocity error as per Lee paper.
    angular_velocity_ = a_angular_velocity_bf;
    angular_velocity_error_bf_ =  angular_velocity_ - R_if_.transpose() * R_des_if_ * desired_angular_velocity_bf_;

}

//---------------------------------------------------------------------------------------------------------
void SkyeGeometricController::UpdateGains(double k_x, double k_v, double k_if,
                                          double k_im, double k_R, double k_omega){
    k_x_ = k_x;
    k_v_ = k_v;
    k_R_ = k_R;
    k_omega_ = k_omega;
    k_if_ = k_if;
    k_im_ = k_im;
    this->ComputeNormalizedGains();
}

//---------------------------------------------------------------------------------------------------------
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

//---------------------------------------------------------------------------------------------------------
void SkyeGeometricController::ComputeForce(Eigen::Vector3d *output_force_bf){

    Eigen::Vector3d proportional_term, derivative_term;

    //calcuate proportional term
    proportional_term = k_x_*position_error_bf_;

    //calcuate derivative term + feedforward term
    derivative_term = k_v_*(velocity_error_bf_) + mass_*desired_acceleration_if_ ;

    // Activate the integrator if within the proper area
    if (position_error_bf_.norm() < distance_integrator_treshold_ ) {

        //Perform the integration with the latest windup term
        integrated_position_error_ += k_if_*position_error_bf_ + windup_force_ ;

        //Save the unsaturated and integrated force for the windup
        unbounded_force_integrator_ = integrated_position_error_;

        //Now perform the saturation
        SaturateVector(maximum_force_integrator_, &integrated_position_error_);

        //perform the back calculation for the inner windup loop
        windup_integrator_force_ = integrated_position_error_ - unbounded_force_integrator_;
    } else {
        integrated_position_error_ << 0,0,0;
    }

    //add all the terms of the controller
    resulting_force_ = proportional_term + derivative_term + integrated_position_error_;
    *output_force_bf = resulting_force_ ;

    // Force saturation
    this->SaturateVector(maximum_force_cog_bf_, output_force_bf);

    //Perform the anti-windup back calculation for the outer part, i.e. the force saturation
    windup_force_ = *output_force_bf - resulting_force_ ;

}

//---------------------------------------------------------------------------------------------------------
void SkyeGeometricController::ComputeAcceleration(Eigen::Vector3d *output_acceleration_bf){

    Eigen::Vector3d proportional_term, derivative_term;

    //calcuate proportional term
    proportional_term = -normalized_k_R_.cwiseProduct(attitude_error_bf_);

    //calcuate derivative term
    derivative_term = - normalized_k_omega_.cwiseProduct(angular_velocity_error_bf_)
                      + angular_velocity_.cross(angular_velocity_);

    // Activate the integrator if within the proper area
    if (attitude_error_bf_.norm() < attitude_integrator_treshold_ ) {
        //Perform the integration with the latest windup term
        integrated_orientation_error_ += k_im_*attitude_error_bf_ + windup_acceleration_;

        //Save the unsaturated and integrated acceleration for the windup
        unbounded_acceleration_integrator_ = integrated_orientation_error_;

        //Now perform the saturation
        SaturateVector(maximum_acceleration_integrator_, &integrated_orientation_error_);

        //perform the back calculation for the inner windup loop
        windup_integrator_acceleration_= integrated_orientation_error_ - unbounded_acceleration_integrator_;

    } else {
        integrated_orientation_error_ << 0,0,0;
    }

    //add all the terms of the controller
    resulting_acceleration_ = proportional_term + derivative_term + integrated_orientation_error_;
    *output_acceleration_bf = resulting_acceleration_ ;

    //acceleration saturation
    this->SaturateVector(maximum_acceleration_cog_bf_, output_acceleration_bf);

    //Perform the anti-windup back calculation for the outer part, i.e. the acceleration saturation
    windup_acceleration_ = *output_acceleration_bf - resulting_acceleration_ ;
}

//---------------------------------------------------------------------------------------------------------
void SkyeGeometricController::UpdateDesiredPose(const Eigen::Vector3d &desired_position_if,
                                                const Eigen::Vector3d &desired_velocity_if,
                                                const Eigen::Vector3d &desired_angular_velocity_bf,
                                                const Eigen::Vector3d &desired_acceleration_if,
                                                const Eigen::Quaterniond &desired_orientation_if){
    //save desired acceleration
    desired_acceleration_if_ = desired_acceleration_if;
    desired_position_if_ = desired_position_if;
    desired_velocity_if_ = desired_velocity_if;
    desired_angular_velocity_bf_ = desired_angular_velocity_bf;
    R_des_if_ = desired_orientation_if.matrix();
}
