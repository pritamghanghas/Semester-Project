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

#include "skye_ros/gazebo_blimp_au_plugin.h"
#include "skye_ros/common.h"

namespace gazebo {

GazeboBlimpAuPlugin::GazeboBlimpAuPlugin()
{
  prev_sim_time_ = 0.0;
  sampling_time_ = 0.01;
}

GazeboBlimpAuPlugin::~GazeboBlimpAuPlugin() {
  
}

/*
 * @brief: Called once at the beginning when model is loaded
 */
void GazeboBlimpAuPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  
  // Load base class parameters
  LoadBaseClassParams(_model, _sdf);
  // Load derived class parameters
  LoadDerivedClassParams(_model, _sdf);

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboBlimpAuPlugin::OnUpdate, this, _1));
}

/*
 * @brief: Load GazeboBlimpAuPlugin specific parameters
 *
 */
void GazeboBlimpAuPlugin::LoadDerivedClassParams(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  // shutdown service of base class 
  server_apply_wrench_bf_.shutdown();
  server_apply_force_bf_.shutdown();
  server_apply_torque_bf_.shutdown();

  // create default service name for AU plugin
  std::string service_name_ = "/skye_gz/" + link_name_ + "/apply_force_2D";
  ROS_INFO_STREAM_COND(SKYE_GAZEBO_PLUGINS_VERBOSE, 
                       "Advertised service name:" << service_name_);

//  server_apply_wrench_bf_ = node_handle_->advertiseService(service_name_,
//                                                           &GazeboBlimpAuPlugin::SetForce2D,
//                                                           this); // todo check this


  server_apply_force_2D_bf_ = node_handle_->advertiseService(service_name_,
                                                             &GazeboBlimpAuPlugin::SetForce2D,
                                                             this);
  // Load filters matrices
  Eigen::Matrix<double,1,1> A_orientation = Eigen::Matrix<double,1,1>::Identity();
  Eigen::Matrix<double,1,1> B_orientation = Eigen::Matrix<double,1,1>::Identity();
  Eigen::Matrix<double,2,2> A_thrust = Eigen::Matrix<double,2,2>::Identity();
  Eigen::Matrix<double,2,1> B_thrust;
  B_thrust.setOnes();

  getSdfMatrix(_sdf, "A_orientation", A_orientation, SKYE_GAZEBO_PLUGINS_VERBOSE);
  getSdfMatrix(_sdf, "B_orientation", B_orientation, SKYE_GAZEBO_PLUGINS_VERBOSE);

  getSdfMatrix(_sdf, "A_thrust", A_thrust, SKYE_GAZEBO_PLUGINS_VERBOSE);
  getSdfMatrix(_sdf, "B_thrust", B_thrust, SKYE_GAZEBO_PLUGINS_VERBOSE);

  body_wrench_->SetFiltersMatrices(A_orientation, B_orientation, A_thrust, B_thrust);

}

/*
 * @brief: This gets called by the world update start event.
 *
 */
void GazeboBlimpAuPlugin::OnUpdate(const common::UpdateInfo& _info) {

  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();

  // update wrench by applying filtering
  //body_wrench_->UpdateFilter(sampling_time_); TODO restore me
  body_wrench_->UpdateNoFilter();
  // apply computed wrench to the link
  body_wrench_->ApplyWrench();
}

/*
 * @brief: This gets called when a user sets a new 2D force to act tangential to the blimp.
 *
 */
bool GazeboBlimpAuPlugin::SetForce2D(skye_ros::ApplyForce2DCogBf::Request   &req,
                                     skye_ros::ApplyForce2DCogBf::Response  &rep) {

  // let's be optimistic
  rep.status_message = "";
  rep.success = true;
  gazebo::math::Vector3 force_bf;

  force_bf.x = req.Fx;
  force_bf.y = req.Fy;
  force_bf.z = 0.0;

  // check request's force is composed by numbers.
  if(!CheckVector3<gazebo::math::Vector3>(force_bf)){
    rep.status_message = "Unvalid parameters in force_2D field";
    rep.success = false; 

    ROS_ERROR_STREAM("[gazebo_blimp_au_plugin] Error in specified 2D force of " << link_name_ 
                    << ": " << rep.status_message);
  }
  else {
      //TODO adjust me
    // save 2D force expressed in body frame
    /*body_wrench_->SetBodyForce(force_bf);

    // update time and duration parameters
    body_wrench_->SetStartTime(req.start_time);
    body_wrench_->SetDuration(req.duration);*/

    rep.status_message = "";
    rep.success = true;
  }

  return true;
}


GZ_REGISTER_MODEL_PLUGIN(GazeboBlimpAuPlugin);
}
