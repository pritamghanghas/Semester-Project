#ifndef SKYE_TEACH_AND_REPEAT_NODE_H
#define SKYE_TEACH_AND_REPEAT_NODE_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include <chrono>

#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>

#include <gazebo_msgs/LinkState.h>
#include <skye_ros/ApplyWrenchCogBf.h>
#include <std_msgs/Int16.h>

#include <skye_teach_and_repeat/skye_teach_and_repeat.h>
#include <skye_teach_and_repeat/skye_trparamsConfig.h>


class SkyeTeachAndRepeatNode
{
public:
  SkyeTeachAndRepeatNode(ros::NodeHandle nh);
  ~SkyeTeachAndRepeatNode();
  void AngularVelocityCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void StateCallback(const gazebo_msgs::LinkState::ConstPtr& msg);
  void ModeSelectionCallback(const std_msgs::Int16::ConstPtr& msg);
  void ExecuteActionCallback(const std_msgs::Int16::ConstPtr& msg);
  bool CallService();
  int get_node_mode();

  /**
   * @brief ConfigCallback : Callback for the dynamic configuration of the parameters.
   * @param config : the skye_params congifuration file
   * @param level : not used
   */
  void ConfigCallback(const skye_teach_and_repeat::skye_trparamsConfig &config, uint32_t level);


private:
  int node_mode_;

  double k_x_, k_v_, k_R_, k_omega_, k_if_, k_im_;

  ros::Publisher acc_pub_;

  SkyeTeachAndRepeat teach_and_repeat_obj_;
  //Eigen variables
  /**
   * @brief position_if_ : position vector expresed in the inertial frame
   */
  Eigen::Vector3d position_if_;
  /**
   * @brief velocity_if_ : linear velocity vector expressed in the inertial frame
   */
  Eigen::Vector3d velocity_if_;
  /**s
   * @brief angular_velocity_bf_ :  angular velocity vector expressed in the body frame
   */
  Eigen::Vector3d angular_velocity_bf_;
  Eigen::Vector3d acceleration_imu_;
  Eigen::Vector3d acceleration_bf_;
  Eigen::Vector3d angular_acceleration_bf_;
  Eigen::Vector3d previous_angular_velocity_bf_;
  Eigen::Vector3d radius_vector_;


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

  // acceleration stuff
  /**
   * @brief linear_acceleration_raw_ : acceleration from IMU
   */
  Eigen::Vector3d linear_acceleration_raw_;
  /**
   * @brief gravity_acceleration_ : constant vector for gravity in if
   */
  Eigen::Vector3d gravity_acceleration_if_;
  /**
   * @brief gravity_acceleration_imu_ : gravity in imu frame
   */
  Eigen::Vector3d gravity_acceleration_imu_;
  /**
   * @brief orientation_imu_if_ : rotation quaternion from imu to if
   */
  Eigen::Quaterniond orientation_imu_if_;
  /**
   * @brief orientation_R_if_ : rotates imu frame to inertial frame
   */
  Eigen::Matrix3d orientation_R_if_;
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

  /**
   * @brief skye_parameters_ : SKye's parameters to easily pass among classes and functions
   */
  SkyeParameters skye_parameters_;

  /**
   * @brief inertia_ : Skye's inertia matrix
   */
  Eigen::Matrix3d inertia_;

  //dynamic reconfigure server for dynamic parameters
  /**
   * @brief dr_srv_ : server for the dynamic reconfiuration of parameters
   */
  dynamic_reconfigure::Server<skye_teach_and_repeat::skye_trparamsConfig> dr_srv_;
  /**
   * @brief cb : type of the callback for skye dynamic parameters
   */
  dynamic_reconfigure::Server<skye_teach_and_repeat::skye_trparamsConfig>::CallbackType cb;


};

#endif // SKYE_TEACH_AND_REPEAT_NODE_H
