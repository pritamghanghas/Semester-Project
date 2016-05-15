//* SkyeRos class
/**
 * SkyeRos class provides an interface to interact with Gazebo simulator.
 */
#ifndef SKYE_ROS_H
#define SKYE_ROS_H

#include <ros/ros.h>
#include <ros/console.h>

#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <cstring>


#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/Vector3.h>

#include "skye_ros/GetLinkStateNed.h"

namespace skye_ros {

//#define HULL_GROUND_TRUTH_HZ 50 //publishing frequency of ground truth inf of the hull

struct skye_ros_parameters{
    /* Topics and services. */
    std::string topic_imu_gazebo; 
    std::string topic_imu_skye;
    std::string topic_ground_truth_hull;
    std::string service_get_link_state;
    double hull_ground_truth_update_frequency;
    std::string blimp_name; // name of the whole model in Gazebo
    std::string hull_name;  // name of the hull link in Gazebo
};                         

class SkyeRos
{
public:
    /**
     * @brief      Default constructor.
     */
    SkyeRos();

    /**
     * @brief      Callback function when a new imu msg has been received from
     *             Gazebo.
     *
     *             Converts Imu orientation and expresses it with respect a NED world frame.
     *
     * @param[in]  imu_gz_sk_p  imu data in Gazebo IMU frame.
     */
    void imuCallback(const sensor_msgs::ImuConstPtr  &imu_gz_sk_p);

    /**
     * @brief      Callback function when get_link_state_ned service is called.
     *
     *             Return the link state expressed in a NED world fixed frame.
     *
     * @param[in]  link_name  name of the link to get the state
     * @param      req   Service response.
     *
     * @return     { description_of_the_return_value }
     */
    bool getLinkStateNed(const std::string                    link_name,
                         skye_ros::GetLinkStateNed::Response  &rep);

    /**
     * @brief      Publish ground truth infomration of the Skye's hull.
     */
    void pubHullGroundTruth();

    /**
     * @brief      Get ground truth frequency update set from yaml file.
     *
     * @return     Frequency update of ground truth topic.
     */
    double getGroundTruthFrequency();

private:
    ros::NodeHandle     nh_;                          /**< Main access point to communicate with ROS. */
    ros::Subscriber     imu_gz_sk_subscriber_;        /**< Sub. to gazebo IMU frame data. */
    ros::Publisher      imu_sk_publisher_;            /**< Pub. of Skye Imu frame data. */
    ros::Publisher      hull_ground_truth_publisher_; /**< Pub. of the true ground state of the hull. */
    ros::ServiceClient  client_gz_get_link_state_;    /**< Client to get link state in Gazebo. */
    Eigen::Quaterniond  q_ned_enu_;                   /**< Quaternion from ENU world frame to NED world frame. */
    Eigen::Quaterniond  q_enu_ned_;                   /**< Quaternion from NED world frame to ENU world frame. */
    skye_ros_parameters param_;                       /**< Parameters loaded from yaml file. */

    /**
     * @brief      Get link state from Gazebo.
     *
     * @param[in]  link_name        link_name in Gazebo.
     * @param[out] link_state       link state: pose and twist.
     * @param[out] success          return true if get info is successful
     * @param[out] status_message   comments if available
     *
     */
    bool getLinkState(const std::string       &link_name,
                      gazebo_msgs::LinkState  &link_state,
                      bool                    &success,
                      std::string             &status_message);

    /**
     * @brief      Convert ROS geometry_msgs::Quaternion to Eigen::Quaternion.
     *
     * @param[in]  quat_in         ROS quaternion.
     * @param[out] quat_out        Eigen quaternion.
     */
    void quaternionMsgToEigen(const geometry_msgs::Quaternion   &quat_in,
                              Eigen::Quaterniond                &quat_out);

    /**
     * @brief      Load configuration parameters from yaml file.
     */
    void getConfiguraionParams();

    /**
     * @brief      Check if a vector is composed by valid numbers
     *
     * @param[in]  v     input vactor3
     *
     * @return     true on success
     */
    bool checkVector3(const geometry_msgs::Vector3   &v);

    

};

} // namespace skye_ros

#endif // SKYE_ROS_H
