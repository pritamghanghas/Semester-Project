
/*
 * Author: Marco Zorzi
 * Description: ROS node for Skye pose control
 */

#ifndef POSE_CONTROL_NODE_H
#define POSE_CONTROL_NODE_H

#include <iostream>
#include <random>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/LinkState.h>

#include <skye_ros/ApplyWrenchCogBf.h>
#include <skye_controls/skye_geometric_controller.h>

#include <skye_controls/skye_paramsConfig.h>
#include <skye_controls/waypoint_controller.h>



class PoseControllerNode {
public:
    /**
     * @brief PoseControllerNode : the constructor initializes the dynamic parameters, creates the desired controller sets up the service to control skye
     * @param nh : Ros node handle to create the service
     *
     * The constructor has a fixed service name that is saved as a class member in the main function. The successful creation of this
     * node is conditioned upon succesfully finding the service. In case the service is wrong or not published the node does not created.
     * The dynamic parameters are used for dynamically configure control parameters
     */
    PoseControllerNode(ros::NodeHandle nh);
    ~PoseControllerNode ();

    /**
     * @brief ConfigCallback : Callback for the dynamic configuration of the parameters.
     * @param config : the skye_params congifuration file
     * @param level : not used
     */
    void ConfigCallback(const skye_controls::skye_paramsConfig &config, uint32_t level);
    /**
     * @brief PositionCallback : this callback is called by the topic providing Skye's ground truth.
     * @param msg : the message containing the lik state
     * This is used as callback to perform the control loop, which means that control inputs are applied every time a new
     * linkstate is received.
     */
    void PositionCallback(const gazebo_msgs::LinkState::ConstPtr& msg);
    /**
     * @brief AngularVelocityCallback : this callback receives the angular velocity from the IMU
     * @param msg :  the imu message containing angular velocity
     */
    void AngularVelocityCallback(const sensor_msgs::Imu::ConstPtr& msg);
    /**
     * @brief CallService : utility wrapped function to call the service
     * @return : true if the service was called correctly
     */
    bool CallService();

private:

    //variables
    /**
     * @brief k_x_ : the position control gain
     */
    double k_x_;
    /**
     * @brief k_v_ : the linear velocity control gain
     */
    double k_v_;
    /**
     * @brief k_if_ : the force integrator control gain
     */
    double k_if_;
    /**
     * @brief k_im_ : the momentum integrator control gain
     */
    double k_im_;
    /**
     * @brief k_omega _ : the angular velocity control gain
     */
    double k_omega_;
    /**
     * @brief k_R_ : the attitude control gain
     */
    double k_R_;

    double wind_x_;
    double wind_y_;
    double wind_z_;
    double wind_x_var_;
    double wind_y_var_;
    double wind_z_var_;
    std::default_random_engine generator_;
    std::normal_distribution<double> *x_dist_;
    std::normal_distribution<double> *y_dist_;
    std::normal_distribution<double> *z_dist_;

    /**
     * @brief wrench_service_name_ : name of the service to apply a wrench to the COG of skye in body frame
     */
    std::string wrench_service_name_;

    //ros stuff
    /**
     * @brief control_wrench_ : wrench created containing control force and momentum that are to be applied to Skye's cog
     */
    geometry_msgs::Wrench control_wrench_;
    /**
     * @brief wrench_service_ : the actual service object that performs the motion
     */
    ros::ServiceClient wrench_service_;
    /**
     * @brief srv_ : skye_ros service file to apply the wrench
     */
    skye_ros::ApplyWrenchCogBf srv_;

    //dynamic reconfigure server for dynamic parameters
    /**
     * @brief dr_srv_ : server for the dynamic reconfiuration of parameters
     */
    dynamic_reconfigure::Server<skye_controls::skye_paramsConfig> dr_srv_;
    /**
     * @brief cb : type of the callback for skye dynamic parameters
     */
    dynamic_reconfigure::Server<skye_controls::skye_paramsConfig>::CallbackType cb;

    //Waypoint Controllers
    /**
     * @brief waypoint_controller_ : waypoint controller for multiple waypoint Skye's control
     */
    WaypointController waypoint_controller_;
    /**
     * @brief waypoint_parameters_ : waypoint parameters object that is in charge of parsing the waypoints
     */
    WaypointControllerParameters waypoint_parameters_;

    //Skye node
    /**
     * @brief geometric_controller_ : geometric trajectory tracking controller as per Lee & al. paper
     */
    SkyeGeometricController geometric_controller_;
    /**
     * @brief skye_parameters_ : SKye's parameters to easily pass among classes and functions
     */
    SkyeParameters skye_parameters_;

    //Eigen variables
    /**
     * @brief position_if_ : position vector expresed in the inertial frame
     */
    Eigen::Vector3d position_if_;
    /**
     * @brief velocity_if_ : linear velocity vector expressed in the inertial frame
     */
    Eigen::Vector3d velocity_if_;
    /**
     * @brief angular_velocity_bf_ :  angular velocity vector expressed in the body frame
     */
    Eigen::Vector3d angular_velocity_bf_;
    /**
     * @brief control_force_bf_ : calculated control force expressed in the body frame
     */
    Eigen::Vector3d control_force_bf_;
    /**
     * @brief control_acceleration_bf_ : calculated control acceleration expressed in the vody frame
     */
    Eigen::Vector3d control_acceleration_bf_;
    /**
     * @brief control_momentum_bf_ : calculated control momentum expressed in the body fixed frame
     */
    Eigen::Vector3d control_momentum_bf_;
    /**
     * @brief orientation_if_ : skye's orientation expressed as a quaternion
     */
    Eigen::Quaterniond orientation_if_;
    /**
     * @brief R_des_if_ : desired rotation matrix to express Skye's attitude
     */
    Eigen::Matrix3d R_des_if_;
    /**
     * @brief inertia_ : Skye's inertia matrix
     */
    Eigen::Matrix3d inertia_;

    //functions
    /**
     * @brief ParseParameters : Function that parses parameters from ROS.. these can be modifided in  ./inputs/skye_controls.yaml file
     * @param nh : ros node handle used to fetch parameters
     * @return : returns true if parameters where correctly parsed
     * This also fetches the dynamic parameters for the first time. Then it packs the SkyeParameters struct for later.
     */
    bool ParseParameters(ros::NodeHandle nh);




};


#endif // POSE_CONTROL_NODE_H
