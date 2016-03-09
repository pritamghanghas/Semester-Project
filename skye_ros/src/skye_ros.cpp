#include <skye_ros/skye_ros.h>


namespace skye_ros {


SkyeRos::SkyeRos()
{
  /* subscribe to topics. */
  imu_enu_subscriber_ =   nh_.subscribe<sensor_msgs::Imu>("sensor_msgs/imu_enu",
                                                          1,
                                                          boost::bind(&SkyeRos::imuEnuCallback, this, _1));
  /* advertise topics. */
  imu_ned_publisher_  =   nh_.advertise<sensor_msgs::Imu>("sensor_msgs/imu_ned", 10);

  /* services. */
  client_gz_apply_body_wrench_  = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("gazebo/apply_body_wrench");
  client_gz_get_link_state_     = nh_.serviceClient<gazebo_msgs::GetLinkState>("gazebo/get_link_state");

  /* rotation matrices and quaternions. */
  q_ned_enu_          =   Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX());
  q_enu_ned_          =   q_ned_enu_.inverse();

}

void SkyeRos::imuEnuCallback(const sensor_msgs::ImuConstPtr &imu_enu_p)
{
  /* convert imu msg from ENU to NED and publish imu_ned message. */
  sensor_msgs::Imu                          imu_ned;
  Eigen::Quaterniond                        q_orientation_ned;
  Eigen::Matrix<double,3,3,Eigen::RowMajor> orientation_cov_ned;
  Eigen::Matrix<double,3,1>                 angular_velocity_ned;
  Eigen::Matrix<double,3,3,Eigen::RowMajor> angular_vel_cov_ned;
  Eigen::Matrix<double,3,1>                 linear_acceleration_ned;
  Eigen::Matrix<double,3,3,Eigen::RowMajor> linear_acc_cov_ned;

  tf::Quaternion              tf_quat;
  Eigen::Quaterniond          eig_quat;
  Eigen::Matrix<double,3,1>   eig_vec3;
  Eigen::Matrix<double,3,3>   eig_matrix3;
  std::vector<double>         std_vecd;

  /* orientation: msg_sensor/imu to tf::quaternion to eigen::quaternion. */
  tf::quaternionMsgToTF(      imu_enu_p->orientation,     tf_quat);
  tf::quaternionTFToEigen(    tf_quat,                  eig_quat);
  q_orientation_ned         = q_ned_enu_ * eig_quat;
  /* orientation_covariance: msg_sensor/imu to eigen::matrix. */
  /** @todo find a better solution to this workaround. */
  std_vecd.assign(&(imu_enu_p->orientation_covariance[0]), 
                  (&(imu_enu_p->orientation_covariance[0])) + 9);
  eig_matrix3               = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> >(std_vecd.data());                                  
  orientation_cov_ned       = q_ned_enu_.matrix() * eig_matrix3 * q_ned_enu_.matrix().transpose();

  /* angular velocity: msgs_sensor/imu to eigen::matrix. */
  tf::vectorMsgToEigen(       imu_enu_p->angular_velocity,  eig_vec3);
  angular_velocity_ned      = q_ned_enu_.matrix() * eig_vec3;
  /* angular_velocity_covariance. */
  /** @todo find a better solution to this workaround. */
  std_vecd.assign(&(imu_enu_p->angular_velocity_covariance[0]), 
                  (&(imu_enu_p->angular_velocity_covariance[0])) + 9);
  eig_matrix3               = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> >(std_vecd.data());                                  
  angular_vel_cov_ned       = q_ned_enu_.matrix() * eig_matrix3 * q_ned_enu_.matrix().transpose();

  /* linear acceleration. */
  tf::vectorMsgToEigen(       imu_enu_p->linear_acceleration,  eig_vec3);
  linear_acceleration_ned   = q_ned_enu_.matrix() * eig_vec3;
  /* linear_acceleration_covariance. */
  /** @todo find a better solution to this workaround. */
  std_vecd.assign(&(imu_enu_p->linear_acceleration_covariance[0]), 
                  (&(imu_enu_p->linear_acceleration_covariance[0])) + 9);
  eig_matrix3               = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> >(std_vecd.data());                                  
  linear_acc_cov_ned        = q_ned_enu_.matrix() * eig_matrix3 * q_ned_enu_.matrix().transpose();


  /* fill the imu_ned message and publish it. */
  imu_ned.header            = imu_enu_p->header;

  tf::quaternionEigenToMsg(   q_orientation_ned,        imu_ned.orientation);
  /** @todo orientation_cov_ned. */

  tf::vectorEigenToMsg(       angular_velocity_ned,     imu_ned.angular_velocity);
  /** @todo angular_vel_cov_ned */
  tf::vectorEigenToMsg(       linear_acceleration_ned,  imu_ned.linear_acceleration);
  /** @todo linear_acc_cov_ned */
  
  /* publish imu_ned*/
  imu_ned_publisher_.publish(imu_ned);


  // TEST, DELETE THIS --- Ok, it's working
  /*gazebo_msgs::ApplyBodyWrench  srv;

  srv.request.body_name         = "skye::hull";
  srv.request.reference_frame   = "";

  srv.request.reference_point.x = 0.0;
  srv.request.reference_point.y = 0.0;
  srv.request.reference_point.z = 0.0;

  srv.request.wrench.force.x    = 5.0;
  srv.request.wrench.force.y    = 0.0;
  srv.request.wrench.force.z    = 0.0;

  srv.request.wrench.torque.x   = 0.0;
  srv.request.wrench.torque.y   = 0.0;
  srv.request.wrench.torque.z   = 0.0;

  srv.request.start_time        = ros::Time::now();
  srv.request.duration          = ros::Duration(-1);

  if (client_gz_apply_body_wrench_.call(srv))
  {
    ROS_INFO("Wrench applied!");
  }
  else
  {
    ROS_ERROR("Failed to call service apply_body_wrench!");
  }*/

  //END TEST
  
  // TEST, DELETE THIS --- Ok, it's working
  /*gazebo_msgs::GetLinkState  srv;

  srv.request.link_name         = "hull";
  srv.request.reference_frame   = "";

  if (client_gz_get_link_state_.call(srv))
  {
    ROS_INFO("Hull position: %f, %f, %f", srv.response.link_state.pose.position.x,
                                          srv.response.link_state.pose.position.y,
                                          srv.response.link_state.pose.position.z);
  }
  else
  {
    ROS_ERROR("Failed to call service get_link_state!");
  }*/

  //END TEST

}

void SkyeRos::myStdVecToMsg(const Eigen::MatrixXd           &matrix,
                            const int                       msg_length,
                            double                          *array_p)
{
  const double  *p  = matrix.data();

  for(int i = 0; i < msg_length; ++i) {
    array_p[i]  = p[i];
  }
}

} // namespace skye_ros


int main(int argc, char **argv)
{
  ros::init(argc, argv, "skye_ros_node");

  skye_ros::SkyeRos skye_ros_interface;

  ros::spin();

  return 0;
}
