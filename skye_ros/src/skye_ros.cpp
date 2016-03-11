#include <skye_ros/skye_ros.h>


namespace skye_ros {


SkyeRos::SkyeRos()
{
  /* Subscribe to topics. */
  imu_enu_subscriber_ = nh_.subscribe<sensor_msgs::Imu>("sensor_msgs/imu_enu",
                                                        1,
                                                        boost::bind(&SkyeRos::imuEnuCallback, this, _1));
  /* Advertise topics. */
  imu_ned_publisher_  = nh_.advertise<sensor_msgs::Imu>("skye_ros/sensor_msgs/imu_ned", 10);

  /* Services. */
  client_gz_apply_body_wrench_ = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("gazebo/apply_body_wrench");
  client_gz_get_link_state_ = nh_.serviceClient<gazebo_msgs::GetLinkState>("gazebo/get_link_state");
  server_apply_wrench_cog_ = nh_.advertiseService("skye_ros/apply_wrench_cog_ned",
                                                  &SkyeRos::applyWrenchCog,
                                                  this);
  server_get_link_state_ned_ = nh_.advertiseService("skye_ros/get_link_state_ned",
                                                    &SkyeRos::getLinkStateNed,
                                                    this);
  /* Rotation matrices and quaternions. */
  q_ned_enu_ = Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX());
  q_enu_ned_ = q_ned_enu_.inverse();

}

void SkyeRos::imuEnuCallback(const sensor_msgs::ImuConstPtr &imu_enu_p)
{
  /* Convert imu msg from ENU to NED and publish imu_ned message.   
  * By using q_ned_enu_ is it possibile to rotate from ENU frame
  * to NED frame.
  */
  sensor_msgs::Imu                          imu_ned;
  Eigen::Quaterniond                        q_orientation_ned;
  Eigen::Matrix<double,3,3,Eigen::RowMajor> orientation_cov_ned;
  Eigen::Matrix<double,3,1>                 angular_velocity_ned;
  Eigen::Matrix<double,3,3,Eigen::RowMajor> angular_vel_cov_ned;
  Eigen::Matrix<double,3,1>                 linear_acceleration_ned;
  Eigen::Matrix<double,3,3,Eigen::RowMajor> linear_acc_cov_ned;

  tf::Quaternion                            tf_quat;
  Eigen::Quaterniond                        eig_quat;
  Eigen::Matrix<double,3,1>                 eig_vec3;
  Eigen::Matrix<double,3,3>                 eig_matrix3;

  /* orientation: msg_sensor/imu to tf::quaternion to eigen::quaternion. */
  tf::quaternionMsgToTF(imu_enu_p->orientation, tf_quat);
  tf::quaternionTFToEigen(tf_quat, eig_quat);
  q_orientation_ned = q_ned_enu_ * eig_quat;

  /* orientation_covariance: msg_sensor/imu to eigen::matrix. */
  /** @todo find a better solution to this workaround: &(array[0]). */
  eig_matrix3 = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> >(&(imu_enu_p->orientation_covariance[0]));                                  
  orientation_cov_ned = q_ned_enu_.matrix() * eig_matrix3 * q_ned_enu_.matrix().transpose();

  /* angular velocity: msgs_sensor/imu to eigen::matrix. */
  tf::vectorMsgToEigen(imu_enu_p->angular_velocity, eig_vec3);
  angular_velocity_ned = q_ned_enu_.matrix() * eig_vec3;

  /* angular_velocity_covariance. */
  /** @todo find a better solution to this workaround: &(array[0]). */
  eig_matrix3 = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> >(&(imu_enu_p->angular_velocity_covariance[0]));                                  
  angular_vel_cov_ned = q_ned_enu_.matrix() * eig_matrix3 * q_ned_enu_.matrix().transpose();

  /* linear acceleration. */
  tf::vectorMsgToEigen(imu_enu_p->linear_acceleration, eig_vec3);
  linear_acceleration_ned = q_ned_enu_.matrix() * eig_vec3;

  /* linear_acceleration_covariance. */
  /** @todo find a better solution to this workaround: &(array[0]). */
  eig_matrix3 = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> >(&(imu_enu_p->linear_acceleration_covariance[0]));                                  
  linear_acc_cov_ned = q_ned_enu_.matrix() * eig_matrix3 * q_ned_enu_.matrix().transpose();

  /* fill the imu_ned message and publish it. */
  imu_ned.header = imu_enu_p->header;

  tf::quaternionEigenToMsg(q_orientation_ned, imu_ned.orientation);

  memcpy(&(imu_ned.orientation_covariance[0]),
         orientation_cov_ned.data(), 
         sizeof(double) * orientation_cov_ned.rows() * orientation_cov_ned.cols());

  tf::vectorEigenToMsg(angular_velocity_ned, imu_ned.angular_velocity);

  memcpy(&(imu_ned.angular_velocity_covariance[0]),
         angular_vel_cov_ned.data(), 
         sizeof(double) * angular_vel_cov_ned.rows() * angular_vel_cov_ned.cols());

  tf::vectorEigenToMsg(linear_acceleration_ned, imu_ned.linear_acceleration);

  memcpy(&(imu_ned.linear_acceleration_covariance[0]),
         linear_acc_cov_ned.data(), 
         sizeof(double) * linear_acc_cov_ned.rows() * linear_acc_cov_ned.cols());

  /* publish imu_ned*/
  imu_ned_publisher_.publish(imu_ned);  
}

bool SkyeRos::applyWrenchCog(skye_ros::ApplyWrenchCogNed::Request   &req,
                             skye_ros::ApplyWrenchCogNed::Response  &rep)
{
  /* Get skye::hull true orientation in Gazebo world frame and use it
   * to rotate the received wrench from skye's NED frame to Gazebo
   * world frame.
  */
  gazebo_msgs::LinkState      hull_state;
  Eigen::Quaterniond          q_world_skye_enu;   /* Orientation of Skye's ENU frame in Gazebo wolrd frame. */    
  Eigen::Matrix<double,3,3>   r_world_skye_ned;   /* Rotation matrix from Skye NED to Gazebo wolrd frame. */
  Eigen::Matrix<double,6,6>   rotation_matrix_wrench; 
  bool                        success;
  std::string                 status_message;

  /* Get skye::hull state information from Gazebo. */
  getLinkState("hull", hull_state, success, status_message);
  quaternionMsgToEigen(hull_state.pose.orientation, q_world_skye_enu);

  /* Rotation matrix for the wrench. */
  r_world_skye_ned = q_world_skye_enu.matrix() * q_enu_ned_.matrix();
  rotation_matrix_wrench.topLeftCorner(r_world_skye_ned.rows(), r_world_skye_ned.cols()) = r_world_skye_ned;
  rotation_matrix_wrench.bottomRightCorner(r_world_skye_ned.rows(), r_world_skye_ned.cols()) = r_world_skye_ned;

  /* Convert the requested wrench from Skye'NED frame to Gazebo world frame. */
  Eigen::Matrix<double,6,1>   wrench_cog_skye_ned;
  Eigen::Matrix<double,6,1>   wrench_cog_wolrd;
  tf::wrenchMsgToEigen(req.wrench, wrench_cog_skye_ned);
  wrench_cog_wolrd = rotation_matrix_wrench * wrench_cog_skye_ned;

  /* Send the converted wrench to Gazebo. */
  gazebo_msgs::ApplyBodyWrench  srv;

  srv.request.body_name = "skye::hull"; /** @todo parametrize body_name */
  srv.request.reference_frame = "";     /**< currently to leave "", otherwise get error. */
  srv.request.reference_point.x = 0.0;
  srv.request.reference_point.y = 0.0;
  srv.request.reference_point.z = 0.0;
  tf::wrenchEigenToMsg(wrench_cog_wolrd, srv.request.wrench);
  srv.request.start_time = req.start_time;
  srv.request.duration = req.duration;

  if (client_gz_apply_body_wrench_.call(srv))
  {
      ROS_INFO("wrench applied!");
  }
  else
  {
      ROS_ERROR("Failed to call service apply_body_wrench from gazebo_ros pkg!");
  }

  rep.success = srv.response.success;
  rep.status_message = srv.response.status_message;

  return true;
}


bool SkyeRos::getLinkStateNed(skye_ros::GetLinkStateNed::Request   &req,
                              skye_ros::GetLinkStateNed::Response  &rep)
{
  gazebo_msgs::LinkState      link_state;
  Eigen::Matrix<double,3,1>   p_skye_enu;         /* Position of link's ENU frame in Gazebo wolrd frame. */
  Eigen::Quaterniond          q_world_skye_enu;   /* Orientation of link's ENU frame in Gazebo wolrd frame. */
  Eigen::Matrix<double,3,1>   v_skye_enu;         /* Linear velocity of link's ENU frame in Gazebo wolrd frame. */
  Eigen::Matrix<double,3,1>   w_skye_enu;         /* Angular velocity of link's ENU frame in Gazebo wolrd frame. */
  bool                        success;
  std::string                 status_message;

  Eigen::Matrix<double,3,1>   p_skye_ned;         /* Position of the link in NED wolrd frame. */
  Eigen::Quaterniond          q_world_skye_ned;   /* Orientation of the link in NED wolrd frame. */
  Eigen::Matrix<double,3,1>   v_skye_ned;         /* Linear velocity of the link in NED wolrd frame. */
  Eigen::Matrix<double,3,1>   w_skye_ned;         /* Angular velocity of the link in NED wolrd frame. */

  /* Get link state, from Gazebo, expressed in Gazebo's ENU fixed world frame.
   * Convert in in a NED fixed frame with same origin than Gazebo's wolrd frame.
   */
  getLinkState(req.link_name, link_state, success, status_message);

  tf::pointMsgToEigen(link_state.pose.position, p_skye_enu);
  quaternionMsgToEigen(link_state.pose.orientation, q_world_skye_enu);
  tf::vectorMsgToEigen(link_state.twist.linear, v_skye_enu);
  tf::vectorMsgToEigen(link_state.twist.angular, w_skye_enu);

  p_skye_ned = q_ned_enu_.matrix() * p_skye_enu;
  q_world_skye_ned = q_ned_enu_ * q_world_skye_enu;
  v_skye_ned = q_ned_enu_.matrix() * v_skye_enu;
  w_skye_ned = q_ned_enu_.matrix() * w_skye_enu;

  /* Fill the service response. */
  rep.link_state.link_name = link_state.link_name;
  tf::pointEigenToMsg(p_skye_ned, rep.link_state.pose.position);
  tf::quaternionEigenToMsg(q_world_skye_ned, rep.link_state.pose.orientation);
  tf::vectorEigenToMsg(v_skye_ned, rep.link_state.twist.linear);
  tf::vectorEigenToMsg(w_skye_ned, rep.link_state.twist.angular);

  rep.success = success;
  rep.status_message = status_message;

  return true;
}


bool SkyeRos::getLinkState(const std::string            &link_name,
                           gazebo_msgs::LinkState       &link_state,
                           bool                         &success,
                           std::string                  &status_message)
{
  gazebo_msgs::GetLinkState   srv;
  bool                        ret = true;

  srv.request.link_name = link_name;
  srv.request.reference_frame = ""; /**< currently to leave "", otherwise get error. */

  if (client_gz_get_link_state_.call(srv))
  {
      link_state = srv.response.link_state;
  }
  else
  {
      ROS_ERROR("Failed to call service get_link_state from gazebo_ros pkg!");
      ret = false;
  }

  success = srv.response.success;
  status_message = srv.response.status_message;

  return ret;
}

void SkyeRos::quaternionMsgToEigen(const geometry_msgs::Quaternion  &quat_in,
                                   Eigen::Quaterniond               &quat_out)
{
  tf::Quaternion tf_quat; 
  tf::quaternionMsgToTF(quat_in, tf_quat);
  tf::quaternionTFToEigen(tf_quat, quat_out);
}

} // namespace skye_ros


int main(int argc, char **argv)
{
  ros::init(argc, argv, "skye_ros_node");

  skye_ros::SkyeRos skye_ros_interface;

  ros::spin();

  return 0;
}
