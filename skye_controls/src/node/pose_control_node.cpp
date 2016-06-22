#include <pose_control_node.h>
#include <waypoints_parser.h>

//-----------------------------------------------------------------------------------------------------------------------
PoseControllerNode::PoseControllerNode(ros::NodeHandle nh){
    // setting to zero the control outputs for initialization
    control_force_bf_ << 0,0,0;
    control_momentum_bf_ << 0,0,0;

    // When the object is created, first parse the parameters
    this->ParseParameters(nh);

    // Configure callback for dynamic parameters
    cb = boost::bind(&PoseControllerNode::ConfigCallback, this, _1, _2);
    dr_srv_.setCallback(cb);

    //Initialize the controller passing all the parameters
    geometric_controller_.InitializeParams(skye_parameters_);

    //Setup service for control input if the service is ready
    ros::service::waitForService(wrench_service_name_);
    wrench_service_ = nh.serviceClient<skye_ros::ApplyWrenchCogBf>(wrench_service_name_, true);
}

//-----------------------------------------------------------------------------------------------------------------------
PoseControllerNode::~PoseControllerNode (){
};


//-----------------------------------------------------------------------------------------------------------------------
bool PoseControllerNode::ParseParameters(ros::NodeHandle nh){
    Eigen::Vector3d zero_v;
    zero_v << 0,0,0;

    //Declare parameters to be imported
    double inertia_11, inertia_12, inertia_13,
            inertia_21, inertia_22, inertia_23,
            inertia_31, inertia_32, inertia_33;

    //Get the parameters
    bool read_all_parameters = nh.getParam("wrench_service_name", wrench_service_name_) &&
                               nh.getParam("mass", skye_parameters_.input_mass) &&
                               nh.getParam("radius_", skye_parameters_.input_radius) &&
                               nh.getParam("number_of_actuators_", skye_parameters_.input_number_of_actuators) &&
                               nh.getParam("maximum_force_cog_", skye_parameters_.input_maximum_force_cog) &&
                               nh.getParam("distance_integrator_treshold_", skye_parameters_.input_distance_integrator_treshold) &&
                               nh.getParam("attitude_integrator_treshold_", skye_parameters_.input_attitude_integrator_treshold) &&
                               nh.getParam("maximum_force_integrator_", skye_parameters_.input_maximum_force_integrator) &&
                               nh.getParam("maximum_momentum_integrator_", skye_parameters_.input_maximum_momentum_integrator) &&
                               nh.getParam("windup_force_threshold", skye_parameters_.input_windup_force_threshold) &&
                               nh.getParam("windup_acceleration_threshold", skye_parameters_.input_windup_acceleration_threshold) &&
                               nh.getParam("wind_x", wind_x_) &&
                               nh.getParam("wind_y", wind_y_) &&
                               nh.getParam("wind_z", wind_z_) &&
                               nh.getParam("wind_x_var", wind_x_var_) &&
                               nh.getParam("wind_y_var", wind_y_var_) &&
                               nh.getParam("wind_z_var", wind_z_var_) &&
                               nh.getParam("inertia_11", inertia_11) &&
                               nh.getParam("inertia_12", inertia_12) &&
                               nh.getParam("inertia_13", inertia_13) &&
                               nh.getParam("inertia_21", inertia_21) &&
                               nh.getParam("inertia_22", inertia_22) &&
                               nh.getParam("inertia_23", inertia_23) &&
                               nh.getParam("inertia_31", inertia_31) &&
                               nh.getParam("inertia_32", inertia_32) &&
                               nh.getParam("inertia_33", inertia_33);

    // Check if Skye's parameters where imported
    if (! read_all_parameters){
        ROS_ERROR("Geometric Parameters not imported");
        return false;
    }

    //Get the parameters for the waypoint
    read_all_parameters = nh.getParam("goal_change_threshold", waypoint_parameters_.input_position_change_threshold) &&
                          nh.getParam("orientation_change_threshold", waypoint_parameters_.input_orientation_change_threshold);

    if (! read_all_parameters){
        ROS_ERROR("Waypoint controller Parameters not imported");
        return false;
    }

    // Parse the waypoints
    WaypointsParser parser(nh, &waypoint_parameters_.input_positions,
                           &waypoint_parameters_.input_orientations );
    for (int i = 0; i < waypoint_parameters_.input_orientations.size(); ++i) {
        waypoint_parameters_.input_velocities.push_back(zero_v);
        waypoint_parameters_.input_angular_velocities.push_back(zero_v);
        waypoint_parameters_.input_accelerations.push_back(zero_v);
    }

    bool is_waypoint_controller_ok =  waypoint_controller_.InitParameters(waypoint_parameters_);
    if (! is_waypoint_controller_ok) {
        ROS_ERROR("Waypoint controller not properly initialized, check sizes");
    }

    // Pack desired position
    skye_parameters_.input_desired_position_if<< 0,0,0;

    // Pack Skye's inertia
    inertia_ << inertia_11, inertia_12, inertia_13,
            inertia_21, inertia_22, inertia_23,
            inertia_31, inertia_32, inertia_33;

    // Initialize desired rotation matrix
    R_des_if_ << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;

    // Save matrices in SkyeParameters struct
    skye_parameters_.input_inertia = inertia_ ;
    skye_parameters_.input_R_des_if = R_des_if_;

    // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
    // of the node can be run simultaneously while using different parameters.
    ros::NodeHandle pnh("~");
    pnh.param("k_x_", k_x_, k_x_);
    pnh.param("k_v_", k_v_, k_v_);
    pnh.param("k_R_", k_R_, k_R_);
    pnh.param("k_omega_", k_omega_, k_omega_);
    pnh.param("k_if_", k_if_, k_if_);
    pnh.param("k_im_", k_im_, k_im_);

    // Initialize dynamic parameters into SkyeParameters struct
    skye_parameters_.input_k_x = k_x_;
    skye_parameters_.input_k_v = k_v_;
    skye_parameters_.input_k_omega = k_omega_;
    skye_parameters_.input_k_R = k_R_;
    skye_parameters_.input_k_if = k_if_;
    skye_parameters_.input_k_im = k_im_;

    // Initialize temporary stuff to zero
    skye_parameters_.input_desired_acceleration_if = zero_v;
    skye_parameters_.input_desired_angular_acceleration_bf = zero_v;
    skye_parameters_.input_desired_angular_velocity_bf = zero_v;
    skye_parameters_.input_desired_velocity_if = zero_v;

    // Initialize wind random distributions
    x_dist_ = new std::normal_distribution<double>(wind_x_,wind_x_var_);
    y_dist_ = new std::normal_distribution<double>(wind_y_,wind_y_var_);
    z_dist_ = new std::normal_distribution<double>(wind_z_,wind_z_var_);

    return true;
}


//-----------------------------------------------------------------------------------------------------------------------
void PoseControllerNode::ConfigCallback(const skye_controls::skye_paramsConfig &config, uint32_t level)
{
    // Set class members to new values. They match what is input at the dynamic reconfigure GUI.
    k_x_ = config.k_x_;
    k_v_ = config.k_v_;
    k_R_ = config.k_R_;
    k_omega_ = config.k_omega_;
    k_if_ = config.k_if_;
    k_im_ = config.k_im_;
}

//-----------------------------------------------------------------------------------------------------------------------
void PoseControllerNode::AngularVelocityCallback(const sensor_msgs::Imu::ConstPtr& msg){
    // Save angular velocity from IMU topic
    // this is in the imu frame but since it is a rigid body they are the same in terms of angular velocity
    angular_velocity_bf_ << msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z;
}

//-----------------------------------------------------------------------------------------------------------------------
void PoseControllerNode::PositionCallback(const gazebo_msgs::LinkState::ConstPtr& msg){
    // pack last known position in inertial frame
    position_if_ << msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z;

    // pack last known velocity in the inertial frame
    velocity_if_ << msg->twist.linear.x,
            msg->twist.linear.y,
            msg->twist.linear.z;

    // pack last known quaternion in the inertial frame
    orientation_if_.x() = msg->pose.orientation.x;
    orientation_if_.y() = msg->pose.orientation.y;
    orientation_if_.z() = msg->pose.orientation.z;
    orientation_if_.w() = msg->pose.orientation.w;

    //perform waypoint control only if there is at least one waypoint
    if (waypoint_parameters_.input_positions.size() > 0) {

        WaypointPose new_pose;

        //Computes the new goal position
        waypoint_controller_.ComputeGoalPosition(position_if_, orientation_if_, &new_pose);

        //Updates desired values for the geometric controller
        geometric_controller_.UpdateDesiredPose(new_pose.position, new_pose.velocity,
                                                new_pose.angular_velocity, new_pose.acceleration, new_pose.orientation);

        // Update the gains with new dynamic parameters
        geometric_controller_.UpdateGains(k_x_ ,k_v_, k_if_, k_im_, k_R_, k_omega_);
        // Update last known state
        geometric_controller_.UpdateParameters(position_if_, velocity_if_, orientation_if_, angular_velocity_bf_);
        // Calculate control force
        geometric_controller_.ComputeForce(&control_force_bf_);
        // Calculate control acceleration
        geometric_controller_.ComputeAcceleration(&control_acceleration_bf_);
        // Calculate control momentum
        control_momentum_bf_ = inertia_*control_acceleration_bf_;
    }
}

//-----------------------------------------------------------------------------------------------------------------------
bool PoseControllerNode::CallService(){
    srv_.request.start_time.nsec = 0;
    srv_.request.duration.sec =  1;

    //Initialize random seed with current time to increase randomness
    srand (time(NULL));
    control_wrench_.force.x = control_force_bf_(0) + (*x_dist_)(generator_); //add random wind here, acts as disturbance
    control_wrench_.force.y = control_force_bf_(1) + (*y_dist_)(generator_);
    control_wrench_.force.z = control_force_bf_(2) + (*z_dist_)(generator_);
    control_wrench_.torque.x = control_momentum_bf_(0);
    control_wrench_.torque.y = control_momentum_bf_(1);
    control_wrench_.torque.z = control_momentum_bf_(2);

    srv_.request.wrench = control_wrench_;
    if (!wrench_service_.call(srv_)){
        ROS_ERROR("Failed to contact service. Could not pass wrench");
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){
    //Init ros and create node handle
    ros::init(argc, argv, "skye_position_controller_node");
    ros::NodeHandle nh;

    // Import ros parameter for service and topic names
    std::string imu_topic, ground_truth_topic;
    bool read_all_parameters = nh.getParam("ground_truth_topic", ground_truth_topic) &&
                               nh.getParam("imu_topic", imu_topic);

    // Check if names have correctly been imported
    if (! read_all_parameters) ROS_ERROR("Parameters not imported");

    //Initialize node and subscribe to topics
    PoseControllerNode node(nh);
    ros::Subscriber pos_sub_ground_truth = nh.subscribe (ground_truth_topic, 1, &PoseControllerNode::PositionCallback, &node);
    ros::Subscriber pos_sub_imu = nh.subscribe (imu_topic, 1, &PoseControllerNode::AngularVelocityCallback, &node);

    ros::Rate r(50); // 50 hz
    while (nh.ok())
    {
        if (!node.CallService()) return -1;
        ros::spinOnce();
        r.sleep();
    }
}
