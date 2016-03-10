//* SkyeRos class
/**
 * SkyeRos class provides an interface to interact with Gazebo simulator.
 */
#ifndef SKYE_ROS_H
#define SKYE_ROS_H


#include <ros/ros.h>


#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <cstring>


#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/LinkState.h>

#include "skye_ros/ApplyWrenchCog.h"

namespace skye_ros {

class SkyeRos
{
public:
    /**
     * @brief      Default constructor.
     */
    SkyeRos();

    /**
     * @brief      Callback function when a new imu msg has been received from Gazebo.
     * 
     * Converts Imu data from local ENU frame, attached to the Imu box in Gazebo, to
     * local NED frame and publish this new data. 
     * This NED frame has the same origin of the ENU frame.
     *
     * @param[in]  imu_enu  Imu data expressed in local ENU frame, IMU attached to Skye.
     */
    void imuEnuCallback(const sensor_msgs::ImuConstPtr      &imu_enu_p);

    /**
     * @brief      Callback function when apply_wrench_cog service is called.
     *
     * @param      req   Service request.
     * @param      rep   Service response.
     *
     */
    bool applyWrenchCog(skye_ros::ApplyWrenchCog::Request   &req,
                        skye_ros::ApplyWrenchCog::Response  &rep);

private:
    ros::NodeHandle     nh_;                        /**< Main access point to communicate with ROS. */
    ros::Subscriber     imu_enu_subscriber_;        /**< Sub. to ENU Imu data. */
    ros::Publisher      imu_ned_publisher_;         /**< Pub. of NED Imu data. */
    Eigen::Quaterniond  q_ned_enu_;                 /**< Quaternion from ENU to NED frame. */
    Eigen::Quaterniond  q_enu_ned_;                 /**< Quaternion from NED to ENU frame. */
    ros::ServiceClient  ct_gz_apply_body_wrench_;   /**< client to apply a body wrench in Gazebo. */
    ros::ServiceClient  ct_gz_get_link_state_;      /**< client to get link state in Gazebo. */
    ros::ServiceServer  sr_apply_wrench_cog_;       /**< client to apply wrench in cog of Skye. */

    /**
     * @brief      Get link state from Gazebo.
     *
     * @param[in]  link_name   link_name in Gazebo.
     * @param[out]      link_state  link state: pose and twist.
     *
     * @return     true on success, false otherwise.
     */
    bool getLinkState(const std::string                   &link_name,
                      gazebo_msgs::LinkState              &link_state);

    /**
     * @brief      Convert ROS geometry_msgs::Quaternion to Eigen::Quaternion.
     *
     * @param[in]  quat_in   ROS quaternion.
     * @param[out] quat_out  Eigen quaternion.
     */
    void quaternionMsgToEigen(const geometry_msgs::Quaternion     &quat_in,
                              Eigen::Quaterniond                  &quat_out);
};

} // namespace skye_ros

#endif // SKYE_ROS_H
