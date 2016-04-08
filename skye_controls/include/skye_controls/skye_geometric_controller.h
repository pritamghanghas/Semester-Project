#ifndef SKYE_GEOMETRIC_CONTROLLER_H
#define SKYE_GEOMETRIC_CONTROLLER_H

#include <Eigen/Eigen>
#include <iostream>

class SkyeGeometricController
{
private:
    //Control coefficients
    double  k_x_, k_v_, k_R_, k_omega_;

    //Skye's mass
    //TODO: check what should go here!
    double mass_;

    // desired poses
    Eigen::Vector3d desired_position_,
                    desired_velocity_,
                    desired_angular_velocity_,
                    desired_angular_acceleration_,
                    desired_acceleration_;

    Eigen::Vector3d position_error_,
                    velocity_error_,
                    attitude_error_,
                    angular_velocity_error_;

    Eigen::Matrix3d inertia_;
    Eigen::Matrix3d R_, R_des_;
    Eigen::Vector3d angular_velocity_;

    Eigen::Vector3d normalized_k_R_;
    Eigen::Vector3d normalized_k_omega_;


public:
    SkyeGeometricController();

    Eigen::Vector3d desired_position();
    Eigen::Vector3d desired_velocity();


void initializeParams(double input_k_x_,
                      double input_k_v_,
                      double input_k_omega_,
                      double input_k_R_,
                      double input_mass_,
                      Eigen::Vector3d & input_desired_position_,
                      Eigen::Vector3d & input_desired_velocity_,
                      Eigen::Vector3d & input_desired_angular_velocity_,
                      Eigen::Vector3d & input_desired_angular_acceleration_,
                      Eigen::Vector3d & input_desired_acceleration_,
                      Eigen::Matrix3d inertia_);

void updateParameters(Eigen::Vector3d & poisiton_,
                      Eigen::Vector3d & velocity_,
                      Eigen::Quaterniond & orientation,
                      Eigen::Vector3d & a_angular_velocity_);

void computeForce(Eigen::Vector3d &output_force_);
void computeMomentum(Eigen::Vector3d & output_momentum_);



};

#endif // SKYE_GEOMETRIC_CONTROLLER_H
