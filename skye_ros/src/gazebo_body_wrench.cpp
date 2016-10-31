/*
 * Copyright 2016 Marco Tranzatto, AEROTAIN, Zurich
 * All rights reserved.
 *
 */
#include <ros/console.h>
#include <ros/ros.h>

#include "skye_ros/gazebo_body_wrench.h"

namespace gazebo {

static int SKYE_COUNTING = 0; // TODO delete me

/*
 * @brief: Initializes body wrench object.
 *
 */
GazeboBodyWrench::GazeboBodyWrench(const physics::WorldPtr &world,
                                   const gazebo::physics::LinkPtr &body,
                                   const gazebo::math::Vector3 &force_application_point) {

  if(!world || !body) {
    ROS_ERROR("GazeboBodyWrench object can't be initialized without pointers ot wolrd and body");
  }

  world_ = world;
  body_ = body;
  force_application_point_ = force_application_point;

  // set vectors to 0
  force_bf_ref_.Set();
  torque_bf_ref_.Set();

  // time and duration
  //start_time_wrench_ = ros::Time();
  start_time_force_ = ros::Time();
  start_time_torque_ = ros::Time();
  //duration_wrench_ = ros::Duration(0);
  duration_force_ = ros::Duration(0);
  duration_torque_ = ros::Duration(0);

  // First and second order filters objects
  tf_first_order_ = new TransferFunction(1,1);
  tf_second_order_ = new TransferFunction(2,1);

  // Filters state space continous time matrixes
  A_orientation_ = Eigen::Matrix<double,1,1>::Identity();
  B_orientation_.setOnes();
  A_thrust_ = Eigen::Matrix<double,2,2>::Identity();
  B_thrust_.setOnes();
}

/*
 * @brief: Initializes body wrench object.
 *
 */
GazeboBodyWrench::GazeboBodyWrench(const physics::WorldPtr &world,
                                   const gazebo::physics::LinkPtr &body) {

  gazebo::math::Vector3 force_application_point;
  force_application_point.Set(0.0, 0.0, 0.0);

  GazeboBodyWrench(world, body, force_application_point);
}

/*
 * @brief: Destructor.
 *
 */
GazeboBodyWrench::~GazeboBodyWrench() {
  
}

/*
 * @brief: Applies a wrench expressed in Gazebo frame
 *
 */
void GazeboBodyWrench::ApplyWrench() {
  // Taken from gazebo_ros_api_plugin, keep on using this or use the API? (require source installation)
  // check times and apply wrench if necessary
  if (body_) // if body exists
  {
    if(ApplyVector(start_time_force_, duration_force_)){
      body_->AddRelativeForce(force_bf_);
    }
    if(ApplyVector(start_time_torque_, duration_torque_)){
      body_->AddRelativeTorque(torque_bf_);
    }
  }

//  if (ros::Time(world_->GetSimTime().Double()) >= start_time_)
//    if (ros::Time(world_->GetSimTime().Double()) <= start_time_ + duration_ ||
//        duration_.toSec() < 0.0)
//    {
//      if (body_) // if body exists
//      {
//        //body_->AddForceAtRelativePosition(force_bf_, force_application_point_);
//        //Bug in AddForceAtRelativePosition: does not allow to specify forc ein body frame

//        body_->AddRelativeForce(force_bf_);
//        //ROS_INFO_STREAM("[body_wrench]\tforce_bf:" << force_bf_);//TODO delete this
//        // TODO compute torque induced by application_point != 0 as
//        body_->AddRelativeTorque(torque_bf_);
//      }
//    }
}



/*
 * @brief: Update wrench without applying any filtering
 *
 */
void GazeboBodyWrench::UpdateNoFilter() {
  force_bf_ = force_bf_ref_;
  torque_bf_ = torque_bf_ref_;
}

/*
 * @brief: Update wrench applying filtering: first order for direction, second order for thrust.
 *
 */
void GazeboBodyWrench::UpdateFilter(const double &sampling_time) {
  
  // apply first order filter on thrust direction theta.
  Eigen::Matrix<double,1,1> first_order_state;
  Eigen::Matrix<double,1,1> theta_ref;
  double theta_k; // thrust direction to apply at this simulation step

  theta_ref << atan2(force_bf_ref_.y, force_bf_ref_.x);

  tf_first_order_->DiscretizeSS(A_orientation_,
                                B_orientation_,
                                sampling_time);

  tf_first_order_->UpdateState(theta_ref, first_order_state);
  theta_k = first_order_state(0,0);

  // apply second order filter on thrust magnitude
  Eigen::Matrix<double,2,1> second_order_state;
  Eigen::Matrix<double,1,1> thrust_ref;
  double thrust_k; // amount of thrust to apply at this simulation step

  thrust_ref << force_bf_ref_.GetLength();

  tf_second_order_->DiscretizeSS(A_thrust_,
                                 B_thrust_,
                                 sampling_time);

  tf_second_order_->UpdateState(thrust_ref, second_order_state);
  thrust_k = second_order_state(0,0);

  // put together magnitude and direcation to make the new force
  force_bf_.x = thrust_k * cos(theta_k);
  force_bf_.y = thrust_k * sin(theta_k);

}

/*
 * @brief: Sets a wrench expressed in body frame.
 *
 */
void GazeboBodyWrench::SetBodyWrench(const gazebo::math::Vector3 &force,
                                     const gazebo::math::Vector3 &torque,
                                     const ros::Time &start_time,
                                     const ros::Duration &duration) {

  SetBodyForce(force, start_time, duration);
  SetBodyTorque(torque, start_time, duration);
}

/*
 * @brief: Sets a force expressed in body frame.
 *
 */
void GazeboBodyWrench::SetBodyForce(const gazebo::math::Vector3 &force,
                                    const ros::Time &start_time,
                                    const ros::Duration &duration) {
  force_bf_ref_ = force;
  SetStartTime(start_time, start_time_force_);
  duration_force_ = duration;
}

/*
 * @brief: Sets a torque in body frame.
 *
 */
void GazeboBodyWrench::SetBodyTorque(const gazebo::math::Vector3 &torque,
                                     const ros::Time &start_time,
                                     const ros::Duration &duration) {
  torque_bf_ref_ = torque;
  SetStartTime(start_time, start_time_torque_);
  duration_torque_ = duration;
}

/*
 * @brief: Sets start time when wrench should be applied.
 *
 */
void GazeboBodyWrench::SetStartTime(const ros::Time &t_user, ros::Time &t_out){
  t_out = t_user;

  if(t_out < ros::Time(world_->GetSimTime().Double()))
    t_out = ros::Time(world_->GetSimTime().Double());
}

/*
 * @brief: Sets duration time for how long wrench should be applied.
 *
 */
//ros::Duration GazeboBodyWrench::SetDuration(const ros::Duration &d) {
//  duration_ = d;
//}

/*
 * @brief: Sets filters continous time matrices.
 *
 */
void GazeboBodyWrench::SetFiltersMatrices(const Eigen::Ref< const Eigen::Matrix<double,1,1> > &A_orientation,
                                          const Eigen::Ref< const Eigen::Matrix<double,1,1> > &B_orientation,
                                          const Eigen::Ref< const Eigen::Matrix<double,2,2> > &A_thrust,
                                          const Eigen::Ref< const Eigen::Matrix<double,2,1> > &B_thrust){
  A_orientation_ = A_orientation;
  B_orientation_ = B_orientation;
  A_thrust_ = A_thrust;
  B_thrust_ = B_thrust;

  /*ROS_INFO_STREAM("Set matrices: \n" << A_orientation_<< "\n" <<
                                        B_orientation_<< "\n" <<
                                        A_thrust_<< "\n" <<
                                        B_thrust_<< "\n");*/
}

/*
 * @brief: Check if either the force or torque witch start_time and duration should be applied
 *
 */
bool GazeboBodyWrench::ApplyVector(const ros::Time &start_time, const ros::Duration &duration){

  // Taken from gazebo_ros_api_plugin, keep on using this or use the API? (require source installation)
  // check times and apply wrench if necessary
  if (ros::Time(world_->GetSimTime().Double()) >= start_time)
    if (ros::Time(world_->GetSimTime().Double()) <= start_time + duration ||
        duration.toSec() < 0.0)
      return true;

  return false;
}

}
