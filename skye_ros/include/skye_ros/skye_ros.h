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
//#include <tf_conversions/transform_datatypes.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/GetLinkState.h>

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
    void imuEnuCallback(    const sensor_msgs::ImuConstPtr &imu_enu_p);



private:
    ros::NodeHandle     nh_;                            /**< Main access point to communicate with ROS. */
    ros::Subscriber     imu_enu_subscriber_;            /**< Sub. to ENU Imu data. */
    ros::Publisher      imu_ned_publisher_;             /**< Pub. of NED Imu data. */
    Eigen::Quaterniond  q_ned_enu_;                     /**< Quaternion from ENU to NED frame. */
    Eigen::Quaterniond  q_enu_ned_;                     /**< Quaternion from NED to ENU frame. */
    ros::ServiceClient  client_gz_apply_body_wrench_;   /**< client to apply a body wrench in Gazebo. */
    ros::ServiceClient  client_gz_get_link_state_;      /**< client to get link state in Gazebo. */



};

} // namespace skye_ros

#endif // SKYE_ROS_H
