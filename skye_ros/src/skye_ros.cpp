#include <skye_ros/skye_ros.h>
#include <math.h>


namespace skye_ros {


SkyeRos::SkyeRos()
{
  /* Get parameters. */
  getConfiguraionParams();

  /* Subscribe to topics. */
  imu_gz_sk_subscriber_ = nh_.subscribe<sensor_msgs::Imu>(param_.topic_imu_gazebo,
                                                          1,
                                                          boost::bind(&SkyeRos::imuCallback, this, _1));
  /* Advertise topics. */
  imu_sk_publisher_  = nh_.advertise<sensor_msgs::Imu>(param_.topic_imu_skye, 10);
  hull_ground_truth_publisher_ = nh_.advertise<gazebo_msgs::LinkState>(param_.topic_ground_truth_hull, 10);

  /* Services. */
  client_gz_get_link_state_ = nh_.serviceClient<gazebo_msgs::GetLinkState>(param_.service_get_link_state);

  /* Quaternions. */
  q_ned_enu_ = Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX()) * 
               Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()); 
  q_enu_ned_ = q_ned_enu_.inverse();

}

void SkyeRos::imuCallback(const sensor_msgs::ImuConstPtr &imu_gz_sk_p)
{
  /* Converts imu msg from Gazebo's IMU frame to Skye's IMU frame. */
  sensor_msgs::Imu                          imu_sk;
  Eigen::Quaterniond                        q_orientation_ned;

  Eigen::Quaterniond                        eig_quat;

  /* Orientation: msg_sensor/imu to tf::quaternion to eigen::quaternion. */
  quaternionMsgToEigen(imu_gz_sk_p->orientation, eig_quat);
  /* Rotation from Gazebo's IMU frame to Gazebo's world ENU (eig_quat).
   * Finally rotation from Gazebo's world ENU to world NED frame (q_ned_enu_).
   */

  q_orientation_ned = q_ned_enu_ * eig_quat;

  //TODO delete me
  /*Eigen::Vector3d ea = eig_quat.matrix().eulerAngles(2, 1, 0);
  printf("EA_GzEnu_SkNed: yaw %f, pitch %f, roll %f \n", (double)ea.x() * 180.0/ M_PI,
                                             (double)ea.y() * 180.0/ M_PI,
                                             (double)ea.z() * 180.0/ M_PI);

  ea = q_orientation_ned.matrix().eulerAngles(2, 1, 0);
  printf("EA_GzNed_SkNed: yaw %f, pitch %f, roll %f \n", (double)ea.x() * 180.0/ M_PI,
                                             (double)ea.y() * 180.0/ M_PI,
                                             (double)ea.z() * 180.0/ M_PI);*/

  //TODO end delete me

  /* fill the imu_sk message and publish it. */
  imu_sk = *imu_gz_sk_p;

  tf::quaternionEigenToMsg(q_orientation_ned, imu_sk.orientation);
  imu_sk_publisher_.publish(imu_sk); 
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
  q_world_link_ned = q_ned_enu_ * q_world_link_enu; //orientation of link's frame in Gazebo's NED f.
  v_link_ned = q_ned_enu_.matrix() * v_link_enu;
  //w_link_ned = q_ned_enu_.matrix() * w_link_enu; // Right way to publish
  //w_link_ned = w_link_enu; //only for test
  w_link_ned = Eigen::Matrix<double,3,1>::Zero();
  /*There is a problem with gazebo's service gazebo/get_link_state: it replaces z values by x values.
  --> the topic gazebo/link_states works however. Therefore: need to change the service call to topic subscription.
  In the meanwhile, send 0.*/

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
  getLinkStateNed(param_.hull_name, res);

  /* Publish the obtained information. */
  gazebo_msgs::LinkState hull_ground_truth;

  hull_ground_truth.pose = res.link_state.pose;
  hull_ground_truth.twist = res.link_state.twist;
  hull_ground_truth.reference_frame = "";/**@see  SkyeRos::getLinkState*/

  hull_ground_truth_publisher_.publish(hull_ground_truth);
}

void SkyeRos::getConfiguraionParams()
{
  bool complete_list_params = true;

  complete_list_params &= nh_.getParam("topic_imu_gazebo", param_.topic_imu_gazebo);
  complete_list_params &= nh_.getParam("topic_imu_skye", param_.topic_imu_skye);
  complete_list_params &= nh_.getParam("topic_ground_truth_hull", param_.topic_ground_truth_hull);
  complete_list_params &= nh_.getParam("service_get_link_state", param_.service_get_link_state);
  complete_list_params &= nh_.getParam("hull_ground_truth_update_frequency", param_.hull_ground_truth_update_frequency);
  complete_list_params &= nh_.getParam("blimp_name", param_.blimp_name);
  complete_list_params &= nh_.getParam("hull_name", param_.hull_name);

  if(!complete_list_params)
    ROS_DEBUG("Parameter(s) missing in yaml file.");

}

double SkyeRos::getGroundTruthFrequency(){
  return param_.hull_ground_truth_update_frequency;
}

bool SkyeRos::checkVector3(const geometry_msgs::Vector3   &v){
  bool ret = true;

  ret = !isnan(v.x) & !isinf(v.x) &
        !isnan(v.y) & !isinf(v.y) &
        !isnan(v.z) & !isinf(v.z);

  return ret;
}

} // namespace skye_ros


int main(int argc, char **argv)
{
  ros::init(argc, argv, "skye_ros_node");

  skye_ros::SkyeRos skye_ros_interface;

  ros::Rate loop_rate(skye_ros_interface.getGroundTruthFrequency());

  /* Sleep half a second before entering the main loop. */
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
