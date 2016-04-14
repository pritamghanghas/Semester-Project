#ifndef SKYE_GEOMETRIC_CONTROLLER_H
#define SKYE_GEOMETRIC_CONTROLLER_H

#include <Eigen/Eigen>
#include <iostream>

struct SkyeParameters{
    int input_number_of_actuators_;
    double input_k_x_;
    double input_k_v_;
    double input_k_if_;
    double input_k_im_;
    double input_k_omega_;
    double input_k_R_;
    double input_mass_;
    double input_radius_;
    double input_maximum_force_cog_;
    double input_distance_integrator_treshold_;
    double input_attitude_integrator_treshold_;
    Eigen::Vector3d input_desired_position_;
    Eigen::Vector3d input_desired_velocity_;
    Eigen::Vector3d input_desired_angular_velocity_;
    Eigen::Vector3d input_desired_angular_acceleration_;
    Eigen::Vector3d input_desired_acceleration_;
    Eigen::Matrix3d input_inertia_;
    Eigen::Matrix3d input_R_des_;

};

class SkyeGeometricController
{
private:
    //Control coefficients
    double  k_x_, k_v_, k_if_, k_im_, k_R_, k_omega_;
    double maximum_force_cog_, maximum_acceleration_cog_,
           distance_integrator_treshold_, attitude_integrator_treshold_;
    int number_of_actuators_ ;

    //Skye's mass and radius
    double mass_, radius_;

    // desired poses
    Eigen::Vector3d desired_position_,
    desired_velocity_,
    desired_angular_velocity_,
    desired_angular_acceleration_,
    desired_acceleration_;

    Eigen::Vector3d position_error_,
    velocity_error_,
    attitude_error_,
    angular_velocity_error_,
    integrator_force_error_,
    integrator_acceleration_error_;

    Eigen::Matrix3d inertia_;
    Eigen::Matrix3d R_, R_des_;
    Eigen::Vector3d angular_velocity_;

    Eigen::Vector3d normalized_k_R_;
    Eigen::Vector3d normalized_k_omega_;

    inline void ComputeNormalizedGains();
    inline void SaturateForce(Eigen::Vector3d & output_force_);
    inline void SaturateAcceleration(Eigen::Vector3d & output_acceleration_);

public:
    SkyeGeometricController();

    void InitializeParams(SkyeParameters param);

    void UpdateParameters(Eigen::Vector3d & poisiton_,
                          Eigen::Vector3d & velocity_,
                          Eigen::Quaterniond & orientation,
                          Eigen::Vector3d & a_angular_velocity_);

    void UpdateGains(double k_x,
                     double k_v,
                     double k_if,
                     double k_im,
                     double k_R,
                     double k_omega);

    void ComputeForce(Eigen::Vector3d &output_force_);
    void ComputeAcceleration(Eigen::Vector3d & output_momentum_);


};

#endif // SKYE_GEOMETRIC_CONTROLLER_H
