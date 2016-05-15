/*
 * Copyright 2016 Matthias Krebs, AEROTAIN, Zurich
 * All rights reserved.
 *
 */

#ifndef GAZEBO_BLIMP_DYNAMICS_PLUGIN_H
#define GAZEBO_BLIMP_DYNAMICS_PLUGIN_H

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace gazebo {


class GazeboBlimpDynamicsPlugin : public ModelPlugin {
 public:

  GazeboBlimpDynamicsPlugin();
  ~GazeboBlimpDynamicsPlugin();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void UpdateAerodynamicForces(const common::UpdateInfo&);

 private:
  std::string namespace_;
  ros::NodeHandle* node_handle_;
  std::string frame_id_;
  std::string link_name_;

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  math::Vector3 velocity_prev_W_;

  // Parameters of model (dimension-free)
  double tau_;      // friction coefficient     [-]
  double cd_;       // drag coefficient         [-]

  // Parameters of model (dimensions)
  double radius_;   // radius of sphere         [m]
  double area_;     // cross section area       [m2]

  // Parameters of environment
  double fluid_density_;  // fluid density        [kg/m3]

};
}

#endif // GAZEBO_BLIMP_DYNAMICS_PLUGIN_H
