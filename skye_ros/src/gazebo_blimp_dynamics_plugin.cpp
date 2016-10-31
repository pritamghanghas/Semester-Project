/*
 * Copyright 2016 Matthias Krebs, AEROTAIN, Zurich
 * All rights reserved.
 *
 */

#include "skye_ros/gazebo_blimp_dynamics_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboBlimpDynamicsPlugin::GazeboBlimpDynamicsPlugin()
    : ModelPlugin(),
      node_handle_(0),
      velocity_prev_W_(0, 0, 0),
      tau_(0),
      cd_(0),
      radius_(0),
      area_(0),
      fluid_density_(1.0)
{}

GazeboBlimpDynamicsPlugin::~GazeboBlimpDynamicsPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

/*
 * @brief: Called once at the beginning when model is loaded
 */
void GazeboBlimpDynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_blimp_dynamics_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_blimp_dynamics_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_blimp_dynamics_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  frame_id_ = link_name_;

  ROS_INFO("Blimp dynamics plugin loaded with parameters:");
  if (_sdf->HasElement("tau"))
    _sdf->GetElement("tau")->GetValue()->Get(tau_);
  ROS_INFO_STREAM("tau: " << tau_);

  if (_sdf->HasElement("cd"))
    _sdf->GetElement("cd")->GetValue()->Get(cd_);
  ROS_INFO_STREAM("cd: " << cd_);

  if (_sdf->HasElement("radius"))
    _sdf->GetElement("radius")->GetValue()->Get(radius_);
  ROS_INFO_STREAM("radius: " << radius_);

  if (_sdf->HasElement("fluid_density"))
    _sdf->GetElement("fluid_density")->GetValue()->Get(fluid_density_);
  ROS_INFO_STREAM("fluid_density: " << fluid_density_);

  // Update parameters
  area_ = 3.1415926 * radius_ * radius_;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboBlimpDynamicsPlugin::UpdateAerodynamicForces, this, _1));

}


/*
 * @brief: This gets called by the world update start event.
 *
 */
void GazeboBlimpDynamicsPlugin::UpdateAerodynamicForces(const common::UpdateInfo& _info) {

  ////////////////////////////////////
  // STEP 1: Get current state
  ////////////////////////////////////
  math::Pose T_WB = link_->GetWorldPose();
  math::Quaternion q_WB = T_WB.rot;
  math::Vector3 velocity_current_W = link_->GetWorldLinearVel();
  math::Vector3 angular_vel_B = link_->GetRelativeAngularVel();
  math::Matrix3 R_BW = q_WB.GetInverse().GetAsMatrix3();

  // Transform from World Frame (W) to Body Frame (B)
  math::Vector3 velocity_current_B = R_BW * velocity_current_W;

  ////////////////////////////////////
  // STEP 2: Calculate aerodynamic effects
  ////////////////////////////////////
  // Aerodynamic drag
  math::Vector3 linear_drag = - 0.5 * cd_ * radius_ * fluid_density_ * velocity_current_B * velocity_current_B.GetLength(); //TODO: lookup table for cd (dep. on Reynolds)

  // Rotational Friction
  // TODO: Assume the body is a sphere. Neglecting linear velocity, the average speed of the sphere's surface is
  // v_avg =
  math::Vector3 angular_drag = - tau_ * angular_vel_B * angular_vel_B.GetLength();


  ////////////////////////////////////
  // STEP 3: Apply to link
  ////////////////////////////////////
  link_->AddRelativeForce(linear_drag);
  link_->AddRelativeTorque(angular_drag);

}


GZ_REGISTER_MODEL_PLUGIN(GazeboBlimpDynamicsPlugin);
}
