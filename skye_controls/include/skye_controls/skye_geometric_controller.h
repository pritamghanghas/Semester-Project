#ifndef SKYE_GEOMETRIC_CONTROLLER_H
#define SKYE_GEOMETRIC_CONTROLLER_H

#include <Eigen/Eigen>

class SkyeGeometricController
{
private:
    double mass_; //relative mass
    //coeffs
    Eigen::Vector3d k_x_, k_v_, k_R_, k_omega_;
    // desired poses
    Eigen::Vector3d desired_position_,
                    desired_velocity_,
                    desired_angular_velocity_,
                    desired_angular_acceleration_,
                    desired_acceleration_;

    Eigen::Matrix3d inertia_;

    Eigen::Vector3d position_error_,
                    velocity_error_,
                    attitude_error_,
                    angular_velocity_error_;

    Eigen::Matrix3d R_, R_des_;
    Eigen::Vector3d angular_velocity_;


public:
    SkyeGeometricController();
    const static double gravity_acceleration_ = 9.81;

    Eigen::Vector3d desired_position();
    Eigen::Vector3d desired_velocity();


void InitializeParams();
void updateParameters(Eigen::Vector3d & poisiton_,
                      Eigen::Vector3d & velocity_,
                      Eigen::Quaterniond & orientation,
                      Eigen::Vector3d & a_angular_velocity_);

void computeForce(Eigen::Vector3d &output_force_);
void computeMomentum(Eigen::Vector3d & output_momentum_);



};

#endif // SKYE_GEOMETRIC_CONTROLLER_H
