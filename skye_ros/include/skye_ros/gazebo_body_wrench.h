/*
 * Copyright 2016 Marco Tranzatto, AEROTAIN, Zurich
 * All rights reserved.
 *
 */

#ifndef GAZEBO_BODY_WRENCH_H
#define GAZEBO_BODY_WRENCH_H

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <math.h>
#include <Eigen/Core>
#include "skye_ros/common.h"

namespace gazebo {

class GazeboBodyWrench {
 public:

  GazeboBodyWrench(const physics::WorldPtr &world,
                   const gazebo::physics::LinkPtr &body,
                   const gazebo::math::Vector3 &force_application_point);
  GazeboBodyWrench(const physics::WorldPtr &world,
                   const gazebo::physics::LinkPtr &body);
  ~GazeboBodyWrench();
  // Apply wrench do the link 
  void ApplyWrench();
  // Update wrench (without applying any filtering) using link's orientation
  void UpdateNoFilter();
  // Update wrench (applying any filtering) using link's orientation
  void UpdateFilter(const double &sampling_time);
  // Set Wrench, both force and torque
  void SetBodyWrench(const gazebo::math::Vector3 &force,
                     const gazebo::math::Vector3 &torque,
                     const ros::Time &start_time,
                     const ros::Duration &duration);
  // Set only force
  void SetBodyForce(const gazebo::math::Vector3 &force,
                    const ros::Time &start_time,
                    const ros::Duration &duration);
  // Set only troque
  void SetBodyTorque(const gazebo::math::Vector3 &torque,
                     const ros::Time &start_time,
                     const ros::Duration &duration);
  // Set start time
  void SetStartTime(const ros::Time &t_user, ros::Time &t_out);
  // Set duration
  //ros::Duration SetDuration(const ros::Duration &d);
  // Set filters matrices
  void SetFiltersMatrices(const Eigen::Ref< const Eigen::Matrix<double,1,1> > &A_orientation,
                          const Eigen::Ref< const Eigen::Matrix<double,1,1> > &B_orientation,
                          const Eigen::Ref< const Eigen::Matrix<double,2,2> > &A_thrust,
                          const Eigen::Ref< const Eigen::Matrix<double,2,1> > &B_thrust);

  // True if either the force or torque with start_time and duration should be applied
  bool ApplyVector(const ros::Time &start_time, const ros::Duration &duration);

 private:
  // Rotate a vector from body frame to Gazebo frame
  // void RotateBfToGz(const gazebo::math::Vector3 &in,
  //                   gazebo::math::Vector3 &out);

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the link
  gazebo::physics::LinkPtr body_;
  // Reference force and torque in body frame
  gazebo::math::Vector3 force_bf_ref_; // desired force in body frame
  gazebo::math::Vector3 torque_bf_ref_; // desired torque in body frame
  gazebo::math::Vector3 force_application_point_; // point of application of the force on the link
  gazebo::math::Vector3 force_bf_; // force in body frame to be applied in current interation
  gazebo::math::Vector3 torque_bf_; // torque in body frame to be applied in current interation
  // Start time and duration
  //ros::Time start_time_wrench_;
  ros::Time start_time_force_;
  ros::Time start_time_torque_;
  //ros::Duration duration_wrench_;
  ros::Duration duration_force_;
  ros::Duration duration_torque_;
  // Transfer functions 
  TransferFunction *tf_first_order_;
  TransferFunction *tf_second_order_;
  // Continuos time state space matrices
  Eigen::Matrix<double,1,1> A_orientation_;
  Eigen::Matrix<double,1,1> B_orientation_;
  Eigen::Matrix<double,2,2> A_thrust_;
  Eigen::Matrix<double,2,1> B_thrust_;
};
}

#endif // GAZEBO_BODY_WRENCH_H
