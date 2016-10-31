#ifndef SKYE_TEACH_AND_REPEAT_NODE_H
#define SKYE_TEACH_AND_REPEAT_NODE_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <chrono>

#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>

#include <gazebo_msgs/LinkState.h>
#include <skye_ros/ApplyWrenchCogBf.h>
#include <std_msgs/Int16.h>

#include <skye_teach_and_repeat/skye_teach_and_repeat.h>
#include <skye_teach_and_repeat/skye_trparamsConfig.h>

/**
 * @brief The SkyeTeachAndRepeatNode class is a ROS node that interfaces with the teach and repeat library
 * It gets data from ros, filters it and executes the Teach and repeat library
 */
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

  /**
   * @brief ConfigCallback : Callback for the dynamic configuration of the parameters.
   * @param config : the skye_params congifuration file
   * @param level : not used
   */
  void ConfigCallback(const skye_teach_and_repeat::skye_trparamsConfig &config, uint32_t level);


private:
  /**
   * @brief node_mode_ : current mode for the node, either teach, repeat or something else.
   * 1 = teach
   * 2 = repeat
   * 3 = standby
   * Everything else = error messages
   */
  int node_mode_;

  //Control coefficients - see geometric controller documentation
  /**
   * @brief k_x_ : the position control gain
   */
  double k_x_;
  /**
   * @brief k_v_ : the linear velocity control gain
   */
  double k_v_;
  /**
   * @brief k_if_ : the force integrator control gain
   */
  double k_if_;
  /**
   * @brief k_im_ : the momentum integrator control gain
   */
  double k_im_;
  /**
   * @brief k_omega_  : the angular velocity control gain
   */
  double k_omega_;
  /**
   * @brief k_R_ : the attitude control gain
   */
  double k_R_;
  /**
   * @brief acc_pub_ : ros publisher to analyze the calculated acceleration of the body frame
   */
  ros::Publisher acc_pub_;
  /**
   * @brief teach_and_repeat_obj_ : Teach and repeat object
   */
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
  /**
   * @brief angular_velocity_bf_ :  angular velocity vector expressed in the body frame
   */
  Eigen::Vector3d angular_velocity_bf_;
  /**
   * @brief acceleration_imu_ : acceleration measured by the body frame without the gravity
   */
  Eigen::Vector3d acceleration_imu_;
  /**
   * @brief acceleration_bf_ : linear acceleration of the body frame
   */
  Eigen::Vector3d acceleration_bf_;
  /**
   * @brief angular_acceleration_bf_ : angular acceleration in the body frame
   */
  Eigen::Vector3d angular_acceleration_bf_;
  /**
   * @brief previous_angular_velocity_bf_ : angular velocity from last iteration to compute its derivative
   */
  Eigen::Vector3d previous_angular_velocity_bf_;
  /**
   * @brief radius_vector_ : radius in every direction
   */
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
