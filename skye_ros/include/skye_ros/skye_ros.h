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

#include "skye_ros/ApplyWrenchCogNed.h"
#include "skye_ros/GetLinkStateNed.h"
#include "skye_ros/ImuAttitudeNed.h"

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
     * a local NED frame and publish this new data. 
     * This NED frame has the same origin of the ENU frame.
     *
     * @param[in]  imu_enu  Imu data expressed in local ENU frame, IMU attached to Skye.
     */
    void imuEnuCallback(const sensor_msgs::ImuConstPtr  &imu_enu_p);

    /**
     * @brief      Callback function when apply_wrench_cog service is called.
     * 
     * Converts the requested wrench, expressed in Skye's NED local frame, to
     * a wrench expressed in Gazebo's ENU world frame and apply it in Gazebo.
     *
     * @param[in]  req   Service request.
     * @param[out] rep   Service response.
     *
     */
    bool applyWrenchCog(skye_ros::ApplyWrenchCogNed::Request   &req,
                        skye_ros::ApplyWrenchCogNed::Response  &rep);

    /**
     * @brief      Callback function when get_link_state_ned service is called.
     * 
     * Return the link state expressed in a NED world fixed frame.
     *
     * @param      req   Service request.   
     * @param      req   Service response.
     *
     */
    bool getLinkStateNed(skye_ros::GetLinkStateNed::Request   &req,
                         skye_ros::GetLinkStateNed::Response  &rep);

private:
    ros::NodeHandle     nh_;                          /**< Main access point to communicate with ROS. */
    ros::Subscriber     imu_enu_subscriber_;          /**< Sub. to ENU Imu data. */
    ros::Publisher      imu_ned_publisher_;           /**< Pub. of NED Imu data. */
    Eigen::Quaterniond  q_ned_enu_;                   /**< Quaternion from ENU to NED frame. */
    Eigen::Quaterniond  q_enu_ned_;                   /**< Quaternion from NED to ENU frame. */
    ros::ServiceClient  client_gz_apply_body_wrench_; /**< Client to apply a body wrench in Gazebo. */
    ros::ServiceClient  client_gz_get_link_state_;    /**< Client to get link state in Gazebo. */
    ros::ServiceServer  server_apply_wrench_cog_;     /**< Server to apply wrench in cog of Skye. */

    /* Removed server_get_link_state_ned_; gazebo provides wrong twist.angular data! 
     * Do not allow an external user to use this service.
     * The function getLinkStateNed can be still used internally to retrieve 
     * information about the pose of one link.
    */
    //ros::ServiceServer  server_get_link_state_ned_;   /**< Server to get link state in world NED frame. */

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
     * @brief      Convert IMU data from a local ENU frame attached to the IMU to local NED frame.
     *
     * @param[in]  imu_enu_p  imu data in local ENU frame attached to the IMU
     * @param[out] imu_ned    imu data in local NED frame attached to the IMU
     */
    void imuEnuToNed(const sensor_msgs::ImuConstPtr     &imu_enu_p,
                     sensor_msgs::Imu                   &imu_ned);
};

} // namespace skye_ros

#endif // SKYE_ROS_H
