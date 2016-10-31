/*
 * Copyright 2016 Marco Tranzatto, AEROTAIN, Zurich
 * All rights reserved.
 *
 */

#ifndef GAZEBO_BLIMP_WRENCH_PLUGIN_H
#define GAZEBO_BLIMP_WRENCH_PLUGIN_H

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <Eigen/Geometry>


#include "skye_ros/ApplyWrenchCogBf.h"
#include "skye_ros/ApplyForceBf.h"
#include "skye_ros/ApplyTorqueBf.h"
#include "skye_ros/gazebo_body_wrench.h"

namespace gazebo {

class GazeboBlimpWrenchPlugin : public ModelPlugin {
 public:

  GazeboBlimpWrenchPlugin();
  ~GazeboBlimpWrenchPlugin();

 protected:

  // Load parameters from sdf file on start up
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  // Load function for base class parameters
  void LoadBaseClassParams(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  // Set a wrench sent via service call
  bool SetWrench(skye_ros::ApplyWrenchCogBf::Request   &req,
                 skye_ros::ApplyWrenchCogBf::Response  &rep);
  // Set a force sent via service call
  bool SetForce(skye_ros::ApplyForceBf::Request   &req,
                skye_ros::ApplyForceBf::Response  &rep);
  // Set a torque sent via service call
  bool SetTorque(skye_ros::ApplyTorqueBf::Request   &req,
                 skye_ros::ApplyTorqueBf::Response  &rep);
  // On Gazebo update
  void OnUpdate(const common::UpdateInfo&);
  // Check if a Vector3 is valid
  template<class T>
  bool CheckVector3(const T &v){
    bool ret = true;

    ret = !isnan(v.x) & !isinf(v.x) &
          !isnan(v.y) & !isinf(v.y) &
          !isnan(v.z) & !isinf(v.z);

    return ret;
  }
  // Copy Vector3
  template<class In, class Out>
  void CopyVector3(const In &v, Out &v_out){
    v_out.x = v.x;
    v_out.y = v.y;
    v_out.z = v.z;
  }

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
  // Orientation of the link's frame w.r.t. Gazebo's frame in the CoG of the AU
  //gazebo::math::Quaternion q_bfgz_au_;  
  // Advertised service to apply a wrench in body frame
  ros::ServiceServer server_apply_wrench_bf_;   
  // Advertised service to apply a force in body frame
  ros::ServiceServer server_apply_force_bf_;
  // Advertised service to apply a torque in body frame
  ros::ServiceServer server_apply_torque_bf_;
  // Wrench to apply
  GazeboBodyWrench *body_wrench_;

};
}

#endif // GAZEBO_BLIMP_WRENCH_PLUGIN_H
