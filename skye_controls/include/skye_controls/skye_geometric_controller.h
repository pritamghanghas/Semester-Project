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

    Eigen::Matrix3d desired_attitude_, inertia_;

    Eigen::Vector3d
//                    position_error_,
//                    velocity_error_,
                    attitude_error_,
                    angular_velocity_error_;

//    Eigen::Matrix3d R_;
    Eigen::Vector3d angular_velocity_;

public:
    SkyeGeometricController();
    const static double gravity_acceleration_ = 9.81;

    Eigen::Vector3d desired_position();
    Eigen::Vector3d desired_velocity();



void computeForce(Eigen::Vector3d &position_error_,
                  Eigen::Vector3d &velocity_error_,
                  Eigen::Matrix3d &R_,
                  Eigen::Vector3d &output_force_);
void InitializeParams();

};

#endif // SKYE_GEOMETRIC_CONTROLLER_H
