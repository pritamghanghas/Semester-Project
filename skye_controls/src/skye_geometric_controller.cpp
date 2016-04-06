/* Author: Marco Zorzi
 * Description: Controller class that computes force and momentum
 *              for Skye.
 * */

#include <skye_controls/skye_geometric_controller.h>

SkyeGeometricController::SkyeGeometricController()
{
}

void SkyeGeometricController::InitializeParams() {
//  gain_attitude_(0) = 3; //4
//  gain_attitude_(1) = 3; //4
//  gain_attitude_(2) = 0.035;

//  gain_angular_rate_(0) = 0.52;//0.6;
//  gain_angular_rate_(1) = 0.52;//0.6;
//  gain_angular_rate_(2) = 0.025;

//  amount_rotors_ = 6;
//  allocation_matrix_.resize(4,amount_rotors_);
//  allocation_matrix_ << sin(M_PI/6),  1,  sin(M_PI/6), -sin(M_PI/6), -1, -sin(M_PI/6),
//                       -cos(M_PI/6),  0,  cos(M_PI/6),  cos(M_PI/6), 0, -cos(M_PI/6),
//                       -1,  1, -1,  1, -1, 1,
//                        1,  1,  1,  1, 1, 1;

//  inertia_matrix_<< 0.0347563,  0,  0,
//                    0,  0.0458929,  0,
//                    0,  0, 0.0977;

//  // to make the tuning independent of the inertia matrix we divide here
//  gain_attitude_ = gain_attitude_.transpose() * inertia_matrix_.inverse();

//  // to make the tuning independent of the inertia matrix we divide here
//  gain_angular_rate_ = gain_angular_rate_.transpose() * inertia_matrix_.inverse();

//  const double rotor_force_constant = 0.00000854858;  //F_i = k_n * rotor_velocity_i^2
//  const double rotor_moment_constant = 0.016;  // M_i = k_m * F_i

//  angular_acc_to_rotor_velocities_.resize(amount_rotors_, 4);
//  const double arm_length = 0.215;

//  Eigen::Matrix4d K;
//  K.setZero();
//  K(0, 0) = arm_length * rotor_force_constant;
//  K(1, 1) = arm_length * rotor_force_constant;
//  K(2, 2) = rotor_force_constant * rotor_moment_constant;
//  K(3, 3) = rotor_force_constant;

//  Eigen::Matrix4d I;
//  I.setZero();
//  I.block<3, 3>(0, 0) = inertia_matrix_;
//  I(3, 3) = 1;
//  angular_acc_to_rotor_velocities_ = allocation_matrix_.transpose()
//      * (allocation_matrix_ * allocation_matrix_.transpose()).inverse() * K.inverse() * I;
//  initialized_params_ = true;
}


void SkyeGeometricController::computeForce(){

}
