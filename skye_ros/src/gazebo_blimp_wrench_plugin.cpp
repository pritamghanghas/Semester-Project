/*
 * Copyright 2016 Marco Tranzatto, AEROTAIN, Zurich
 * All rights reserved.
 *
 */

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <boost/bind.hpp>

#include "skye_ros/gazebo_blimp_wrench_plugin.h"
#include "skye_ros/common.h"

namespace gazebo {

GazeboBlimpWrenchPlugin::GazeboBlimpWrenchPlugin()
    : ModelPlugin(),
      node_handle_(0)
{
}

GazeboBlimpWrenchPlugin::~GazeboBlimpWrenchPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

/*
 * @brief: Called once at the beginning when model is loaded
 */
void GazeboBlimpWrenchPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  
  // Load base class parameters
  LoadBaseClassParams(_model, _sdf);

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboBlimpWrenchPlugin::OnUpdate, this, _1));

}

/*
 * @brief: Base class parameters
 *
 */
void GazeboBlimpWrenchPlugin::LoadBaseClassParams(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_blimp_bodyframe_wrench_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_blimp_bodyframe_wrench_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_blimp_bodyframe_wrench_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  frame_id_ = link_name_;

  // Create default service name
  std::string service_name_base = "/skye_gz/" + link_name_;
 
  ROS_INFO_STREAM_COND(SKYE_GAZEBO_PLUGINS_VERBOSE, 
                       link_name_ << ": skye_gazebo_wrench_plugin loaded with parameters");

  // application point of the force on the link
  math::Vector3 force_application_point(0.0, 0.0, 0.0);

  getSdfParam<math::Vector3>(_sdf, "centerOfAttack", 
                             force_application_point, force_application_point, 
                             SKYE_GAZEBO_PLUGINS_VERBOSE);

  // Service
  server_apply_wrench_bf_ = node_handle_->advertiseService(service_name_base + "/apply_wrench",
                                                          &GazeboBlimpWrenchPlugin::SetWrench,
                                                          this);

  server_apply_force_bf_ = node_handle_->advertiseService(service_name_base + "/apply_force",
                                                          &GazeboBlimpWrenchPlugin::SetForce,
                                                          this);

  server_apply_torque_bf_ = node_handle_->advertiseService(service_name_base + "/apply_torque",
                                                           &GazeboBlimpWrenchPlugin::SetTorque,
                                                           this);

  // Body wrench
  body_wrench_ = new GazeboBodyWrench(world_, link_, force_application_point);
}

/*
 * @brief: This gets called by the world update start event.
 *
 */
void GazeboBlimpWrenchPlugin::OnUpdate(const common::UpdateInfo& _info) {

  // update wrench without applying any filtering
  body_wrench_->UpdateNoFilter();
  // apply computed wrench to the link
  body_wrench_->ApplyWrench();
}

/*
 * @brief: This gets called when a user sets a new wrench in body frame.
 *
 */
bool GazeboBlimpWrenchPlugin::SetWrench(skye_ros::ApplyWrenchCogBf::Request   &req,
                                        skye_ros::ApplyWrenchCogBf::Response  &rep) {

  // check request's wrench is composed by numbers.
  if(!CheckVector3<geometry_msgs::Vector3>(req.wrench.force) && 
     !CheckVector3<geometry_msgs::Vector3>(req.wrench.torque)){

    rep.status_message = "Unvalid parameters in wrench.force or wrench.torque";
    rep.success = false;
    ROS_ERROR_STREAM("[gazebo_blimp_wrench_plugin] Error in specified body wrench of " << link_name_ 
                    <<", unvalid parameters");
    return false;
  }

  // save wrench expressed in body frame
  gazebo::math::Vector3 force_bf;
  gazebo::math::Vector3 torque_bf;
  CopyVector3(req.wrench.force, force_bf);
  CopyVector3(req.wrench.torque, torque_bf);
  body_wrench_->SetBodyWrench(force_bf, torque_bf, req.start_time, req.duration);

  rep.status_message = "";
  rep.success = true;

  return true;
}

/*
 * @brief: This gets called when a user sets a new force in body frame.
 *
 */
bool GazeboBlimpWrenchPlugin::SetForce(skye_ros::ApplyForceBf::Request   &req,
                                       skye_ros::ApplyForceBf::Response  &rep) {

  // check request's force is composed by numbers.
  if(!CheckVector3<geometry_msgs::Vector3>(req.force)){

    rep.status_message = "Unvalid parameters in force";
    rep.success = false;
    ROS_ERROR_STREAM("[gazebo_blimp_wrench_plugin] Error in specified body force of " << link_name_
                    <<", unvalid parameters");
    return false;
  }

  // save force expressed in body frame
  gazebo::math::Vector3 force_bf;
  CopyVector3(req.force, force_bf);
  body_wrench_->SetBodyForce(force_bf, req.start_time, req.duration);

  rep.status_message = "";
  rep.success = true;

  return true;
}

/*
 * @brief: This gets called when a user sets a new torque in body frame.
 *
 */
bool GazeboBlimpWrenchPlugin::SetTorque(skye_ros::ApplyTorqueBf::Request   &req,
                                        skye_ros::ApplyTorqueBf::Response  &rep) {

  // check request's torque is composed by numbers.
  if(!CheckVector3<geometry_msgs::Vector3>(req.torque)){

    rep.status_message = "Unvalid parameters in torque";
    rep.success = false;
    ROS_ERROR_STREAM("[gazebo_blimp_wrench_plugin] Error in specified body torque of " << link_name_
                    <<", unvalid parameters");
    return false;
  }

  // save torque expressed in body frame
  gazebo::math::Vector3 torque_bf;
  CopyVector3(req.torque, torque_bf);
  body_wrench_->SetBodyTorque(torque_bf, req.start_time, req.duration);

  rep.status_message = "";
  rep.success = true;

  return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboBlimpWrenchPlugin);
}
