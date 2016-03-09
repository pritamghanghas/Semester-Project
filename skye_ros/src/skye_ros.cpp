#include <skye_ros/skye_ros.h>


namespace skye_ros {


SkyeRos::SkyeRos()
{
  /* Subscribe to topics. */
  imu_enu_subscriber_ =   nh_.subscribe<sensor_msgs::Imu>("sensor_msgs/imu_enu",
                                                          1,
                                                          boost::bind(&SkyeRos::imuEnuCallback, this, _1));
  /* Advertise topics. */
  imu_ned_publisher_  =   nh_.advertise<sensor_msgs::Imu>("sensor_msgs/imu_ned", 10);

  /* Services. */
  client_gz_apply_body_wrench_  = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("gazebo/apply_body_wrench");
  client_gz_get_link_state_     = nh_.serviceClient<gazebo_msgs::GetLinkState>("gazebo/get_link_state");

  /* Rotation matrices and quaternions. */
  q_ned_enu_          =   Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX());
  q_enu_ned_          =   q_ned_enu_.inverse();

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
  tf::quaternionMsgToTF(        imu_enu_p->orientation,     tf_quat);
  tf::quaternionTFToEigen(      tf_quat,                  eig_quat);
  q_orientation_ned         =   q_ned_enu_ * eig_quat;

  /* orientation_covariance: msg_sensor/imu to eigen::matrix. */
  /** @todo find a better solution to this workaround: &(array[0]). */
  eig_matrix3               =   Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> >(&(imu_enu_p->orientation_covariance[0]));                                  
  orientation_cov_ned       =   q_ned_enu_.matrix() * eig_matrix3 * q_ned_enu_.matrix().transpose();

  /* angular velocity: msgs_sensor/imu to eigen::matrix. */
  tf::vectorMsgToEigen(         imu_enu_p->angular_velocity,  eig_vec3);
  angular_velocity_ned      =   q_ned_enu_.matrix() * eig_vec3;

  /* angular_velocity_covariance. */
  /** @todo find a better solution to this workaround: &(array[0]). */
  eig_matrix3               =   Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> >(&(imu_enu_p->angular_velocity_covariance[0]));                                  
  angular_vel_cov_ned       =   q_ned_enu_.matrix() * eig_matrix3 * q_ned_enu_.matrix().transpose();

  /* linear acceleration. */
  tf::vectorMsgToEigen(         imu_enu_p->linear_acceleration,  eig_vec3);
  linear_acceleration_ned   =   q_ned_enu_.matrix() * eig_vec3;

  /* linear_acceleration_covariance. */
  /** @todo find a better solution to this workaround: &(array[0]). */
  eig_matrix3               =   Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> >(&(imu_enu_p->linear_acceleration_covariance[0]));                                  
  linear_acc_cov_ned        =   q_ned_enu_.matrix() * eig_matrix3 * q_ned_enu_.matrix().transpose();

  /* fill the imu_ned message and publish it. */
  imu_ned.header            =   imu_enu_p->header;

  tf::quaternionEigenToMsg(     q_orientation_ned,        imu_ned.orientation);

  memcpy(                       &(imu_ned.orientation_covariance[0]),
                                orientation_cov_ned.data(), 
                                sizeof(double) * orientation_cov_ned.rows() * orientation_cov_ned.cols());

  tf::vectorEigenToMsg(         angular_velocity_ned,     imu_ned.angular_velocity);

  memcpy(                       &(imu_ned.angular_velocity_covariance[0]),
                                angular_vel_cov_ned.data(), 
                                sizeof(double) * angular_vel_cov_ned.rows() * angular_vel_cov_ned.cols());
  
  tf::vectorEigenToMsg(         linear_acceleration_ned,  imu_ned.linear_acceleration);

  memcpy(                       &(imu_ned.linear_acceleration_covariance[0]),
                                linear_acc_cov_ned.data(), 
                                sizeof(double) * linear_acc_cov_ned.rows() * linear_acc_cov_ned.cols());
  
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


} // namespace skye_ros


int main(int argc, char **argv)
{
  ros::init(argc, argv, "skye_ros_node");

  skye_ros::SkyeRos skye_ros_interface;

  ros::spin();

  return 0;
}
