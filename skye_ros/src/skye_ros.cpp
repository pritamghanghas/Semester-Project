#include <skye_ros/skye_ros.h>


namespace skye_ros {


SkyeRos::SkyeRos()
{
  /* Subscribe to topics. */
  imu_gz_sk_subscriber_ = nh_.subscribe<sensor_msgs::Imu>("sensor_msgs/imu_gz",
                                                          1,
                                                          boost::bind(&SkyeRos::imuCallback, this, _1));
  /* Advertise topics. */
  imu_sk_publisher_  = nh_.advertise<sensor_msgs::Imu>("skye_ros/sensor_msgs/imu_sk", 10);
  hull_ground_truth_publisher_ = nh_.advertise<gazebo_msgs::LinkState>("skye_ros/ground_truth/hull", 10);

  /* Services. */
  client_gz_apply_body_wrench_ = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("gazebo/apply_body_wrench");
  client_gz_get_link_state_ = nh_.serviceClient<gazebo_msgs::GetLinkState>("gazebo/get_link_state");
  server_apply_wrench_cog_ = nh_.advertiseService("skye_ros/apply_wrench_cog_bf",
                                                  &SkyeRos::applyWrenchCog,
                                                  this);

  /* Quaternions. */
  q_ned_enu_ = Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX()) * 
               Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()); 
  q_enu_ned_ = q_ned_enu_.inverse();

  q_gz_sk_ = Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX());
  q_sk_gz_ = q_gz_sk_.inverse();

}

void SkyeRos::imuCallback(const sensor_msgs::ImuConstPtr &imu_gz_sk_p)
{
  /* Converts imu msg from Gazebo's IMU frame to Skye's IMU frame. */
  sensor_msgs::Imu                          imu_sk;
  Eigen::Quaterniond                        q_orientation_ned;

  Eigen::Matrix<double,3,3,Eigen::RowMajor> orientation_cov_bf;
  Eigen::Matrix<double,3,3,Eigen::RowMajor> angular_vel_cov_bf;
  Eigen::Matrix<double,3,3,Eigen::RowMajor> linear_acc_cov_bf;
  Eigen::Quaterniond                        eig_quat;
  Eigen::Matrix<double,3,3>                 eig_matrix3;

  /* Orientation: msg_sensor/imu to tf::quaternion to eigen::quaternion. */
  quaternionMsgToEigen(imu_gz_sk_p->orientation, eig_quat);
  /* Rotation from Skye's IMU frame to Gazebo's IMU frame (q_gz_sk_). 
   * Then rotation from Gazebo's IMU frame to Gazebo's world ENU (eig_quat). 
   * Finally rotation from Gazebo's world ENU to world NED frame (q_ned_enu_).
   */
  q_orientation_ned = q_ned_enu_ * eig_quat * q_gz_sk_;

  /* orientation_covariance: msg_sensor/imu to eigen::matrix. */
  /** @todo find a better solution to this workaround: &(array[0]). */
  eig_matrix3 = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> >(&(imu_gz_sk_p->orientation_covariance[0]));                                  
  orientation_cov_bf = q_sk_gz_.matrix() * eig_matrix3 * q_sk_gz_.matrix().transpose();

  /* angular velocity: invert Y and Z axis. */
  imu_sk.angular_velocity.x =   imu_gz_sk_p->angular_velocity.x;
  imu_sk.angular_velocity.y = -(imu_gz_sk_p->angular_velocity.y);
  imu_sk.angular_velocity.z = -(imu_gz_sk_p->angular_velocity.z);

  /* angular_velocity_covariance. */
  /** @todo find a better solution to this workaround: &(array[0]). */
  eig_matrix3 = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> >(&(imu_gz_sk_p->angular_velocity_covariance[0]));                                  
  angular_vel_cov_bf = q_sk_gz_.matrix() * eig_matrix3 * q_sk_gz_.matrix().transpose();

  /* linear acceleration: invert Y and Z axis. */
  imu_sk.linear_acceleration.x =   imu_gz_sk_p->linear_acceleration.x;
  imu_sk.linear_acceleration.y = -(imu_gz_sk_p->linear_acceleration.y);
  imu_sk.linear_acceleration.z = -(imu_gz_sk_p->linear_acceleration.z);

  /* linear_acceleration_covariance. */
  /** @todo find a better solution to this workaround: &(array[0]). */
  eig_matrix3 = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> >(&(imu_gz_sk_p->linear_acceleration_covariance[0]));                                  
  linear_acc_cov_bf = q_sk_gz_.matrix() * eig_matrix3 * q_sk_gz_.matrix().transpose();

  /* fill the imu_sk message and publish it. */
  imu_sk.header = imu_gz_sk_p->header;

  tf::quaternionEigenToMsg(q_orientation_ned, imu_sk.orientation);

  memcpy(&(imu_sk.orientation_covariance[0]),
         orientation_cov_bf.data(), 
         sizeof(double) * orientation_cov_bf.rows() * orientation_cov_bf.cols());

  /* angular velocity already filled. */

  memcpy(&(imu_sk.angular_velocity_covariance[0]),
         angular_vel_cov_bf.data(), 
         sizeof(double) * angular_vel_cov_bf.rows() * angular_vel_cov_bf.cols());

  /* linear acceleration already filled. */

  memcpy(&(imu_sk.linear_acceleration_covariance[0]),
         linear_acc_cov_bf.data(), 
         sizeof(double) * linear_acc_cov_bf.rows() * linear_acc_cov_bf.cols());

  imu_sk_publisher_.publish(imu_sk); 
}

bool SkyeRos::applyWrenchCog(skye_ros::ApplyWrenchCogBf::Request   &req,
                             skye_ros::ApplyWrenchCogBf::Response  &rep)
{
  /* Get skye::hull true orientation in Gazebo world frame and use it
   * to rotate the received wrench from skye's body frame frame to Gazebo
   * world frame.
  */
  gazebo_msgs::LinkState      hull_state;
  Eigen::Quaterniond          q_world_skye_gzbf; /* Orientation of Skye's body frame in Gazebo ENU wolrd frame. */    
  Eigen::Matrix<double,3,3>   r_world_skye_bf;   /* Rotation matrix from Skye's body frame to Gazebo ENU wolrd frame. */
  Eigen::Matrix<double,6,6>   rotation_matrix_wrench; 
  bool                        success;
  std::string                 status_message;

  /* Get skye::hull state information from Gazebo. This is the state of the Gazebo's local frame
   * attached to Skye, expressed in Gazebo ENU world frame.
   */
  if(!getLinkState("hull", hull_state, success, status_message)){ /**@todo remove hardcoded string "hull" */
    ROS_ERROR("[skye_ros] Error in getting link state. Msg: %s", status_message.c_str());
    return false;
  }
     
  quaternionMsgToEigen(hull_state.pose.orientation, q_world_skye_gzbf);

  /* Rotation matrix for the wrench. */
  r_world_skye_bf = q_world_skye_gzbf.matrix() * q_gz_sk_.matrix();
  rotation_matrix_wrench.topLeftCorner(r_world_skye_bf.rows(), r_world_skye_bf.cols()) = r_world_skye_bf;
  rotation_matrix_wrench.bottomRightCorner(r_world_skye_bf.rows(), r_world_skye_bf.cols()) = r_world_skye_bf;

  /* Convert the requested wrench from Skye's body frame to Gazebo ENU world frame. */
  Eigen::Matrix<double,6,1>   wrench_cog_skye_bf;
  Eigen::Matrix<double,6,1>   wrench_cog_wolrd;
  tf::wrenchMsgToEigen(req.wrench, wrench_cog_skye_bf);
  wrench_cog_wolrd = rotation_matrix_wrench * wrench_cog_skye_bf;

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
    //ROS_INFO("wrench applied!");
  }
  else
  {
    ROS_ERROR("[skye_ros] Failed to call service apply_body_wrench from gazebo_ros pkg");
  }

  rep.success = srv.response.success;
  rep.status_message = srv.response.status_message;

  return true;
}


bool SkyeRos::getLinkStateNed(const std::string                    link_name,
                              skye_ros::GetLinkStateNed::Response  &rep)
{
  gazebo_msgs::LinkState      link_state;
  Eigen::Matrix<double,3,1>   p_link_enu;         /* Position of link's Gazebo body frame in Gazebo ENU wolrd frame. */
  Eigen::Quaterniond          q_world_link_enu;   /* Orientation of link's Gazebo body frame in Gazebo ENU wolrd frame. */
  Eigen::Matrix<double,3,1>   v_link_enu;         /* Linear velocity of link's Gazebo body frame in Gazebo ENU wolrd frame. */
  Eigen::Matrix<double,3,1>   w_link_enu;         /* Angular velocity of link's Gazebo body frame in Gazebo ENU wolrd frame. */
  bool                        success;
  std::string                 status_message;

  Eigen::Matrix<double,3,1>   p_link_ned;         /* Position of the link in Gazebo NED wolrd frame. */
  Eigen::Quaterniond          q_world_link_ned;   /* Orientation of the link in Gazebo NED wolrd frame. */
  Eigen::Matrix<double,3,1>   v_link_ned;         /* Linear velocity of the link in Gazebo NED wolrd frame. */
  Eigen::Matrix<double,3,1>   w_link_ned;         /* Angular velocity of the link in Gazebo NED wolrd frame. */

  /* Get link state, from Gazebo, expressed in Gazebo's ENU fixed world frame.
   * Convert it in a NED world frame with same origin of Gazebo's wolrd frame. This is
   * the Gazebo NED world frame.
   */
  if(!getLinkState(link_name, link_state, success, status_message)){
    ROS_ERROR("[skye_ros] Error in getting link state. Msg: %s", status_message.c_str());
    return false;
  }

  tf::pointMsgToEigen(link_state.pose.position, p_link_enu);
  quaternionMsgToEigen(link_state.pose.orientation, q_world_link_enu);
  tf::vectorMsgToEigen(link_state.twist.linear, v_link_enu);
  tf::vectorMsgToEigen(link_state.twist.angular, w_link_enu);


  p_link_ned = q_ned_enu_.matrix() * p_link_enu;
  q_world_link_ned = q_ned_enu_ * q_world_link_enu * q_gz_sk_; //orientation of link's NED frame in Gazebo's NED f.
  v_link_ned = q_ned_enu_.matrix() * v_link_enu;
  //w_link_ned = q_ned_enu_.matrix() * w_link_enu;/** @todo see how to properly feel this. */
  //w_link_ned = w_link_enu; //only for test
  w_link_ned = Eigen::Matrix<double,3,1>::Zero();/** @todo delt this and replace with right values. */

  /* Debug */
  /*Eigen::Vector3d euler_angles = q_world_link_ned.matrix().eulerAngles(2, 1, 0);
  ROS_INFO("yaw: %f\tpitch: %f\troll: %f",  euler_angles[0] * 180.0 / M_PI,
                                            euler_angles[1] * 180.0 / M_PI, 
                                            euler_angles[2] * 180.0 / M_PI);

  ROS_INFO("p.x: %f\t p.y: %f\tp.z: %f",  p_link_ned[0],
                                          p_link_ned[1],
                                          p_link_ned[2]);

  ROS_INFO("av.x: %f\t av.y: %f\tav.z: %f", w_link_ned[0],
                                            w_link_ned[1],
                                            w_link_ned[2]);

  ROS_INFO("lv.x: %f\t lv.y: %f\tlv.z: %f", v_link_ned[0],
                                            v_link_ned[1],
                                            v_link_ned[2]);*/
  /* End Debug. */

  /* Fill the service response. */
  rep.link_state.link_name = link_state.link_name;
  tf::pointEigenToMsg(p_link_ned, rep.link_state.pose.position);
  tf::quaternionEigenToMsg(q_world_link_ned, rep.link_state.pose.orientation);
  tf::vectorEigenToMsg(v_link_ned, rep.link_state.twist.linear);
  tf::vectorEigenToMsg(w_link_ned, rep.link_state.twist.angular);

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
      ROS_ERROR("[skye_ros] Failed to call service get_link_state from gazebo_ros pkg");
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

void SkyeRos::pubHullGroundTruth()
{
  /* Get Hull's ground truth information from Gazebo. */
  skye_ros::GetLinkStateNed::Response  res;
  getLinkStateNed("hull", res);

  /* Publish the obtained information. */
  gazebo_msgs::LinkState hull_ground_truth;

  hull_ground_truth.pose = res.link_state.pose;
  hull_ground_truth.twist = res.link_state.twist;
  hull_ground_truth.reference_frame = "";/**@see  SkyeRos::getLinkState*/

  hull_ground_truth_publisher_.publish(hull_ground_truth);
}

} // namespace skye_ros


int main(int argc, char **argv)
{
  ros::init(argc, argv, "skye_ros_node");

  skye_ros::SkyeRos skye_ros_interface;

  ros::Rate loop_rate(HULL_GROUND_TRUTH_HZ);

  /* Sleep fo half a second before entering the main loop. */
  ros::Duration(0.5).sleep();

  while (ros::ok())
  {
    /* Use the frequency set in loop_rate to publish ground truth information. */
    skye_ros_interface.pubHullGroundTruth();

    /* Once per loop allow callback functions to be called. */
    ros::spinOnce();

    /* sleep untill next loop. */
    loop_rate.sleep();
  }

  return 0;
}
