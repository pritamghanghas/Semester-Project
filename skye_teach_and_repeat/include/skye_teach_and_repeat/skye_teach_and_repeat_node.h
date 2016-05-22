#ifndef SKYE_TEACH_AND_REPEAT_NODE_H
#define SKYE_TEACH_AND_REPEAT_NODE_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/LinkState.h>
#include <skye_ros/ApplyWrenchCogBf.h>
#include <std_msgs/Int16.h>

#include <skye_controls/skye_geometric_controller.h>
#include <skye_controls/skye_paramsConfig.h>
#include <skye_controls/waypoint_controller.h>

struct SkyeWaypoint{
  //TODO: add time
  Eigen::Vector3d waypoint_position_if;
  Eigen::Vector3d waypoint_velocity_if;
  Eigen::Vector3d waypoint_angular_velocity_bf;
  Eigen::Quaterniond waypoint_orientation_if;
};

struct SkyeAction{
  int action_id;
  std::vector<SkyeWaypoint> action_trajectory;
};

class SkyeTeachAndRepeatNode
{
public:
  SkyeTeachAndRepeatNode(ros::NodeHandle nh);
  ~SkyeTeachAndRepeatNode();
  void TeachPhase(const Eigen::Vector3d& position_if,
                  const Eigen::Vector3d& velocity_if,
                  const Eigen::Vector3d& angular_velocity_bf,
                  const Eigen::Quaterniond& orientation_if);
  void RepeatPhase();
  void AngularVelocityCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void StateCallback(const gazebo_msgs::LinkState::ConstPtr& msg);
  void ModeSelectionCallback(const std_msgs::Int16::ConstPtr& msg);
  void ExecuteActionCallback(const std_msgs::Int16::ConstPtr& msg);
  void PackParameters();
  bool CallService();
  int node_mode();
  bool teaching_done();
private:
  int node_mode_;
  int teaching_mode_; //1 for space, 2 for time
  bool teaching_done_;
  bool has_teaching_just_started_;
  double waypoints_distance_threshold_;
  int action_selected_;

  std::vector<SkyeAction> saved_data_;

  //Eigen variables
  /**
   * @brief position_if_ : position vector expresed in the inertial frame
   */
  Eigen::Vector3d position_if_;
  /**
   * @brief velocity_if_ : linear velocity vector expressed in the inertial frame
   */
  Eigen::Vector3d velocity_if_;
  /**
   * @brief angular_velocity_bf_ :  angular velocity vector expressed in the body frame
   */
  Eigen::Vector3d angular_velocity_bf_;
  /**
   * @brief orientation_if_ : skye's orientation expressed as a quaternion
   */
  Eigen::Quaterniond orientation_if_;
  /**
   * @brief control_force_bf_ : calculated control force expressed in the body frame
   */
  Eigen::Vector3d control_force_bf_;
  /**
   * @brief control_acceleration_bf_ : calculated control acceleration expressed in the vody frame
   */
  Eigen::Vector3d control_acceleration_bf_;
  /**
   * @brief control_momentum_bf_ : calculated control momentum expressed in the body fixed frame
   */
  Eigen::Vector3d control_momentum_bf_;

  /**
   * @brief wrench_service_name_ : name of the service to apply a wrench to the COG of skye in body frame
   */
  std::string wrench_service_name_;

  //ros stuff
  /**
   * @brief control_wrench_ : wrench created containing control force and momentum that are to be applied to Skye's cog
   */
  geometry_msgs::Wrench control_wrench_;
  /**
   * @brief wrench_service_ : the actual service object that performs the motion
   */
  ros::ServiceClient wrench_service_;
  /**
   * @brief srv_ : skye_ros service file to apply the wrench
   */
  skye_ros::ApplyWrenchCogBf srv_;


  //Geometric Controller
  WaypointControllerParameters waypoints_parameters_;
  WaypointController waiponts_controller_;

};

#endif // SKYE_TEACH_AND_REPEAT_NODE_H
