#include <skye_teach_and_repeat/skye_teach_and_repeat_node.h>

//---------------------------------------------------------------------------------------------------------
SkyeTeachAndRepeatNode::SkyeTeachAndRepeatNode(ros::NodeHandle nh){
    //Prepare variables for the parameters
    double waypoints_distance_threshold, orientation_distance_threshold;
    int teaching_mode;
    //Declare parameters to be imported
    double inertia_11, inertia_12, inertia_13,
            inertia_21, inertia_22, inertia_23,
            inertia_31, inertia_32, inertia_33;
    //Get the parameters
    bool read_all_parameters = nh.getParam("wrench_service_name", wrench_service_name_) &&
                               nh.getParam("teaching_mode", teaching_mode) &&
                               nh.getParam("waypoints_distance_threshold", waypoints_distance_threshold) &&
                               nh.getParam("orientation_distance_threshold", orientation_distance_threshold) &&
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
                               //                               nh.getParam("k_x", k_x_) &&
                               //                               nh.getParam("k_v", k_v_) &&
                               //                               nh.getParam("k_R", k_R_) &&
                               //                               nh.getParam("k_omega", k_omega_) &&
                               //                               nh.getParam("k_if", k_if_) &&
                               //                               nh.getParam("k_im", k_im_) &&
                               //                               nh.getParam("wind_x", wind_x_) &&
                               //                               nh.getParam("wind_y", wind_y_) &&
                               //                               nh.getParam("wind_z", wind_z_) &&
                               //                               nh.getParam("wind_x_var", wind_x_var_) &&
                               //                               nh.getParam("wind_y_var", wind_y_var_) &&
                               //                               nh.getParam("wind_z_var", wind_z_var_) &&
                               nh.getParam("inertia_11", inertia_11) &&
                               nh.getParam("inertia_12", inertia_12) &&
                               nh.getParam("inertia_13", inertia_13) &&
                               nh.getParam("inertia_21", inertia_21) &&
                               nh.getParam("inertia_22", inertia_22) &&
                               nh.getParam("inertia_23", inertia_23) &&
                               nh.getParam("inertia_31", inertia_31) &&
                               nh.getParam("inertia_32", inertia_32) &&
                               nh.getParam("inertia_33", inertia_33);

    // Check if parameters where imported succesfully
    if (! read_all_parameters) ROS_ERROR("Teach and repeate Parameters not imported");


    // Pack Skye's inertia
    inertia_ << inertia_11, inertia_12, inertia_13,
            inertia_21, inertia_22, inertia_23,
            inertia_31, inertia_32, inertia_33;

    // Pack desired position
    skye_parameters_.input_desired_position_if<< 0,0,0;

    radius_vector_ << skye_parameters_.input_radius, 0,0;

    // Save matrices in SkyeParameters struct
    skye_parameters_.input_inertia = inertia_ ;
    skye_parameters_.input_k_x = k_x_;
    skye_parameters_.input_k_v = k_v_;
    skye_parameters_.input_k_omega = k_omega_;
    skye_parameters_.input_k_R = k_R_;
    skye_parameters_.input_k_if = k_if_;
    skye_parameters_.input_k_im = k_im_;
    // Initialize temporary stuff to zero
    Eigen::Vector3d zero_v;
    zero_v << 0,0,0;
    skye_parameters_.input_desired_acceleration_if = zero_v;
    skye_parameters_.input_desired_angular_acceleration_bf = zero_v;
    skye_parameters_.input_desired_angular_velocity_bf = zero_v;
    skye_parameters_.input_desired_velocity_if = zero_v;
    skye_parameters_.input_R_des_if << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;

    gravity_acceleration_if_ << 0,0,9.800;

    //Initialize parameters of the teach and repeat object
    teach_and_repeat_obj_.InitializeParameters(waypoints_distance_threshold,
                                               orientation_distance_threshold,
                                               teaching_mode, inertia_,
                                               skye_parameters_);

    // Configure callback for dynamic parameters
    cb = boost::bind(&SkyeTeachAndRepeatNode::ConfigCallback, this, _1, _2);
    dr_srv_.setCallback(cb);

    //Setup service for control input if the service is ready
    ros::service::waitForService(wrench_service_name_);
    wrench_service_ = nh.serviceClient<skye_ros::ApplyWrenchCogBf>(wrench_service_name_, true);

    acc_pub_ = nh.advertise<sensor_msgs::Imu>("/skye_T_a_R/imu_bf", 1);
    previous_time_ = std::chrono::high_resolution_clock::now();
    previous_angular_velocity_bf_ << 0,0,0;

    ROS_INFO("Teach and Repeat node initialized correctly, waiting for mode selection");
}
//---------------------------------------------------------------------------------------------------------
SkyeTeachAndRepeatNode::~SkyeTeachAndRepeatNode(){
}

//---------------------------------------------------------------------------------------------------------
void SkyeTeachAndRepeatNode::ConfigCallback(const skye_teach_and_repeat::skye_trparamsConfig &config, uint32_t level)
{
    // Set class variables to new values. They match what is input at the dynamic reconfigure GUI.
    k_x_ = config.k_x_tr;
    k_v_ = config.k_v_tr;
    k_R_ = config.k_R_tr;
    k_omega_ = config.k_omega_tr;
    k_if_ = config.k_if_tr;
    k_im_ = config.k_im_tr;

    if (1) {
        std::cout << "***********************************************************" << std::endl <<
                     "CALLED PARAMETERS CALLBACK!" << std::endl << std::endl;
        std::cout << "k_x_tr: " << config.k_x_tr <<
                     " | k_v_tr: " << config.k_v_tr <<
                     " | k_R_tr: " << config.k_R_tr <<
                     " | k_omega_tr: " << config.k_omega_tr <<
                     " | k_if_tr: " << config.k_if_tr <<
                     " | k_omega_: " << config.k_im_tr <<
                     std::endl << std::endl;
        std::cout << "***********************************************************" <<
                     std::endl << std::endl;

    }


}

//---------------------------------------------------------------------------------------------------------
void SkyeTeachAndRepeatNode::AngularVelocityCallback(const sensor_msgs::Imu::ConstPtr& msg){
    current_time_ = std::chrono::high_resolution_clock::now();

    // Save angular velocity from IMU topic
    // this is in the imu frame but since it is a rigid body they are the same in terms of angular velocity
    angular_velocity_bf_ << msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z;


    linear_acceleration_raw_ << msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z;

    orientation_imu_if_.x() =  msg->orientation.x;
    orientation_imu_if_.y() =  msg->orientation.y;
    orientation_imu_if_.z() =  msg->orientation.z;
    orientation_imu_if_.w() =  msg->orientation.w;

    orientation_R_if_ = orientation_imu_if_.matrix();
    gravity_acceleration_imu_ = orientation_R_if_.transpose()*gravity_acceleration_if_;

    acceleration_imu_ = linear_acceleration_raw_ - gravity_acceleration_imu_;

    angular_acceleration_bf_ = (angular_velocity_bf_ - previous_angular_velocity_bf_)/ 0.02; //0.02 because 50hz loop

    acceleration_bf_ = acceleration_imu_ - angular_acceleration_bf_.cross(radius_vector_) - angular_velocity_bf_.cross(angular_velocity_bf_.cross(radius_vector_));

    sensor_msgs::Imu mess;
    mess.header.frame_id = msg->header.frame_id;
    mess.header.stamp = msg->header.stamp;
    mess.linear_acceleration.x = acceleration_bf_(0);
    mess.linear_acceleration.y = acceleration_bf_(1);
    mess.linear_acceleration.z = acceleration_bf_(2);
    acc_pub_.publish(mess);

    previous_angular_velocity_bf_ = angular_velocity_bf_;
    previous_time_ = current_time_;

}
//---------------------------------------------------------------------------------------------------------
void SkyeTeachAndRepeatNode::StateCallback(const gazebo_msgs::LinkState::ConstPtr& msg){
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

    // Interface for updating controller parameters online, not required by ROS node
    teach_and_repeat_obj_.UpdateControllerParameters(k_x_,
                                                     k_v_,
                                                     k_if_,
                                                     k_im_,
                                                     k_R_,
                                                     k_omega_);

    // This should allow new parameters from outside (ie. QGC)
    teach_and_repeat_obj_.ExecuteTeachAndRepeat(position_if_,
                                                velocity_if_,
                                                angular_velocity_bf_,
                                                acceleration_imu_,
                                                orientation_if_,
                                                &control_force_bf_,
                                                &control_momentum_bf_);



    /******************** DEBUG ***********************
     * WHEN REMOVING THIS CODE DO NOT FORGET TO REMOVE IOSTREAM INCLUSION
     * IN THE HEADER FILE!!!!
     *
     */
    if (1) {
        std::cout << "--------------------- T & r-----------------------------" << std::endl <<
                     "position_if_: " << position_if_(0) <<
                     " | y: " << position_if_(1) <<
                     " | z: " << position_if_(2) <<
                     std::endl << std::endl;


        std::cout << "velocity_if_: " << velocity_if_(0) <<
                     " | y: " << velocity_if_(1) <<
                     " | z: " << velocity_if_(2) <<
                     std::endl << std::endl;

        std::cout << "angular_velocity_bf_: " << angular_velocity_bf_(0) <<
                     " | y: " << angular_velocity_bf_(1) <<
                     " | z: " << angular_velocity_bf_(2) <<
                     std::endl << std::endl;


        std::cout << "acceleration_if_: " << acceleration_imu_(0) <<
                     " | y: " << acceleration_imu_(1) <<
                     " | z: " << acceleration_imu_(2) <<
                     std::endl << std::endl;

        std::cout << "orientation_if_: " << orientation_if_.x() <<
                     " | y: " << orientation_if_.y() <<
                     " | z: " << orientation_if_.z() <<
                     " | w: " << orientation_if_.w() <<
                     std::endl << std::endl;


        // ATTITUDE

        std::cout << "control_force_bf_: " << control_force_bf_(0) <<
                     " | y: " << control_force_bf_(1) <<
                     " | z: " << control_force_bf_(2) <<
                     std::endl << std::endl;


        std::cout << "control_momentum_bf_: " << control_momentum_bf_(0) <<
                     " | y: " << control_momentum_bf_(1) <<
                     " | z: " << control_momentum_bf_(2) <<
                     std::endl << std::endl;



        std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << std::endl;



    }

    /******************** END DEBUG *************************/


    if(node_mode_ == REPEAT_MODE) {
        this->CallService();
    }

}

//---------------------------------------------------------------------------------------------------------
void SkyeTeachAndRepeatNode::ExecuteActionCallback(const std_msgs::Int16::ConstPtr& msg){
    int action_to_repeat = msg->data;
    if (teach_and_repeat_obj_.AssignNewActionToRepeat(action_to_repeat)) {
        std::cout << "next action to exectue is: " << action_to_repeat << std::endl;
    }
}

//---------------------------------------------------------------------------------------------------------
void SkyeTeachAndRepeatNode::ModeSelectionCallback(const std_msgs::Int16::ConstPtr& msg){
    int new_mode = msg->data;
    if ( teach_and_repeat_obj_.CheckModeChange(new_mode) ) {
        node_mode_ = new_mode;
        std::cout << "New mode found is: " << new_mode << std::endl;
    }
}

//---------------------------------------------------------------------------------------------------------
int SkyeTeachAndRepeatNode::get_node_mode(){
    return teach_and_repeat_obj_.node_mode();
}

//---------------------------------------------------------------------------------------------------------
bool SkyeTeachAndRepeatNode::CallService(){
    srv_.request.start_time.nsec = 0;
    srv_.request.duration.sec =  1;

    control_wrench_.force.x =  control_force_bf_(0);// + (*x_dist_)(generator_);
    control_wrench_.force.y = control_force_bf_(1);// + (*y_dist_)(generator_);
    control_wrench_.force.z = control_force_bf_(2);// + (*z_dist_)(generator_);
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

//---------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){
    //Init ros and create node handle
    ros::init(argc, argv, "skye_position_controller_node");
    ros::NodeHandle nh;
    // Import ros parameter for service and topic names
    std::string mode_topic, choice_topic, imu_topic, ground_truth_topic;
    bool read_all_parameters = nh.getParam("ground_truth_topic", ground_truth_topic) &&
                               nh.getParam("imu_topic", imu_topic) &&
                               nh.getParam("choice_topic", choice_topic) &&
                               nh.getParam("mode_topic", mode_topic);
    // Check if names have correctly been imported
    if (! read_all_parameters) ROS_ERROR("Main Parameters not imported");

    //Advertise topics that need the callback afterwards
    std_msgs::Int16 zero; //(new std_msgs::Int16);
    zero.data = 0;
    ros::Publisher mode_sel_pub = nh.advertise<std_msgs::Int16>(mode_topic, 1);
    ros::Publisher repeat_choice_pub = nh.advertise<std_msgs::Int16>(choice_topic, 1);


    mode_sel_pub.publish(zero);
    repeat_choice_pub.publish(zero);

    ros::Publisher acc_pub = nh.advertise<sensor_msgs::Imu>("/skye_T_a_R/imu_bf", 1);

    //Initialize node and subscribe to topics
    SkyeTeachAndRepeatNode node(nh);
    ros::Subscriber pos_sub_ground_truth = nh.subscribe (ground_truth_topic, 1, &SkyeTeachAndRepeatNode::StateCallback, &node);
    ros::Subscriber pos_sub_imu = nh.subscribe (imu_topic, 1, &SkyeTeachAndRepeatNode::AngularVelocityCallback, &node);
    ros::Subscriber mode_sel_sub = nh.subscribe (mode_topic, 1, &SkyeTeachAndRepeatNode::ModeSelectionCallback, &node);
    ros::Subscriber choice_sel_sub = nh.subscribe (choice_topic, 1, &SkyeTeachAndRepeatNode::ExecuteActionCallback, &node);

    ros::Rate r(50); // 50 hz
    while (nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

}
