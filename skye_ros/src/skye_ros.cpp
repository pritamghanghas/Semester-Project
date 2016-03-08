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

void SkyeRos::imuEnuCallback(const sensor_msgs::ImuConstPtr &imu_enu)
{
  /* convert imu msg from ENU to NED and publish imu_ned message. */
  sensor_msgs::Imu            imu_ned;
  Eigen::Quaterniond          q_orientation_ned;
  Eigen::Matrix<std_msgs::Float64,3,3,Eigen::RowMajor> orientation_cov_ned;
  Eigen::Matrix<double,3,1>   angular_velocity_ned;
  Eigen::Matrix<std_msgs::Float64,3,3,Eigen::RowMajor> angular_vel_cov_ned;
  Eigen::Matrix<double,3,1>   linear_acceleration_ned;
  Eigen::Matrix<std_msgs::Float64,3,3,Eigen::RowMajor> linear_acc_vel_cov_ned;

  tf::Quaternion              tf_quat;
  Eigen::Quaterniond          eig_quat;
  Eigen::Matrix<double,3,1>   eig_vec3;

  /* conversion in three steps: msg_sensor/imu to tf::quaternion to eigen::quaternion. */

  /* orientation. */
  tf::quaternionMsgToTF(      imu_enu->orientation,     tf_quat);
  tf::quaternionTFToEigen(    tf_quat,                  eig_quat);
  q_orientation_ned         = q_ned_enu_ * eig_quat;

  /*
  ROS_INFO("q_enu: %f, %f, %f, %f", eig_quat.w(),
                                    eig_quat.x(),
                                    eig_quat.y(),
                                    eig_quat.z());
  ROS_INFO("q_ned: %f, %f, %f, %f", q_orientation_ned.w(),
                                    q_orientation_ned.x(),
                                    q_orientation_ned.y(),
                                    q_orientation_ned.z());*/

  /** @todo Add orientation_covariance_ned */

  /* angular velocity. */
  tf::vectorMsgToEigen(       imu_enu->angular_velocity,  eig_vec3);
  angular_velocity_ned      = q_ned_enu_.matrix() * eig_vec3;

  /*
  ROS_INFO("omega_enu: %f, %f, %f",     eig_vec3.x(),
                                        eig_vec3.y(),
                                        eig_vec3.z());

  ROS_INFO("omega_ned: %f, %f, %f",     angular_velocity_ned.x(),
                                        angular_velocity_ned.y(),
                                        angular_velocity_ned.z());*/

  /** @todo Add angular_velocity_covariance_ned */

  /* linear acceleration. */
  tf::vectorMsgToEigen(       imu_enu->linear_acceleration,  eig_vec3);
  linear_acceleration_ned   = q_ned_enu_.matrix() * eig_vec3;
  /*
  ROS_INFO("a_enu: %f, %f, %f",         eig_vec3.x(),
                                        eig_vec3.y(),
                                        eig_vec3.z());

  ROS_INFO("a_ned: %f, %f, %f",         linear_acceleration_ned.x(),
                                        linear_acceleration_ned.y(),
                                        linear_acceleration_ned.z());*/

  /** @todo Add linear_acceleration_covariance_ned */


  //Eigen::Matrix<std_msgs::Float64,3,3> matx;
  //matx(1,1) = imu_enu->orientation_covariance[0];
  /*double pippo[9];
  for(int i=0; i<9; i++){ 
    pippo[i] = imu_enu->orientation_covariance[i];
  }*/
  //orientation_cov_ned       = Eigen::Map<Eigen::Matrix<std_msgs::Float64,3,3,Eigen::RowMajor> >(imu_enu->orientation_covariance, 9); 
                              /*r_ned_enu_.transpose();*/
  //Eigen::Map<Eigen::VectorXd> (static_cast<double*>(imu_enu->orientation_covariance), 9);
  Eigen::Matrix<double,3,3,Eigen::RowMajor> m1 = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> >(imu_enu->orientation_covariance, 9); 


  /* fill the imu_ned message and publish it. */
  imu_ned.header            = imu_enu->header;
  tf::quaternionEigenToMsg(   q_orientation_ned,        imu_ned.orientation);
  /** @todo Add orientation_covariance_ned */
  tf::vectorEigenToMsg(       angular_velocity_ned,     imu_ned.angular_velocity);
  /** @todo Add angular_velocity_covariance_ned */
  tf::vectorEigenToMsg(       linear_acceleration_ned,  imu_ned.linear_acceleration);
  /** @todo Add linear_acceleration_covariance_ned */


  //tf::matrixEigenToMsg(       orientation_cov_ned,      imu_ned.orientation_covariance);
  //
  
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

void SkyeRos::myMsgToEig(const std_msgs::Float64         *msg,
                         const int                       msg_length,
                         Eigen::MatrixXd                 &matrix) 
{
  /* temporary workaround. */
  /** @todo fix this with a more appropriate solution. */
  /*double*   copy_msg  = new double[msg_length];

  for(int i = 0; i < msg_length; ++i) {
    copy_msg  = msg[i];
  }

  matrix  = Eigen::Map<Eigen::Matrix<double,-1,-1,Eigen::RowMajor> >(copy_msg, msg_length);*/
}

} // namespace skye_ros


int main(int argc, char **argv)
{
  ros::init(argc, argv, "skye_ros_node");

  skye_ros::SkyeRos skye_ros_interface;

  ros::spin();

  return 0;
}
