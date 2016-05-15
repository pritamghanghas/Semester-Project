/*
 * Copyright 2016 Marco Tranzatto, AEROTAIN, Zurich
 * All rights reserved.
 *
 */

#ifndef GAZEBO_BLIMP_AU_PLUGIN_H
#define GAZEBO_BLIMP_AU_PLUGIN_H

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <Eigen/Geometry>

#include "skye_ros/gazebo_blimp_wrench_plugin.h"
#include "skye_ros/gazebo_body_wrench.h"
#include "skye_ros/ApplyForce2DCogBf.h"

namespace gazebo {

class GazeboBlimpAuPlugin : public GazeboBlimpWrenchPlugin {
 public:

  GazeboBlimpAuPlugin();
  ~GazeboBlimpAuPlugin();


 protected:
  // Load parameters from sdf file on start up
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  // Additional load function for further developments than the basic class GazeboBlimpWrenchPlugin
  void LoadDerivedClassParams(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  // Overridenn function of base class, called when Gazebo is updated
  void OnUpdate(const common::UpdateInfo& _info);
  // Set a 2D force sent via service call
  bool SetForce2D(skye_ros::ApplyForce2DCogBf::Request   &req,
                  skye_ros::ApplyForce2DCogBf::Response  &rep);


  // Advertised service to apply a 2D force in body frame
  ros::ServiceServer server_apply_force_2D_bf_;

  double prev_sim_time_;
  double sampling_time_;
};
}

#endif // GAZEBO_BLIMP_AU_PLUGIN_H
