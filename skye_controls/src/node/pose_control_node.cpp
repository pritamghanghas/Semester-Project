#include <pose_control_node.h>

bool PoseControllerNode::ParseParameters(ros::NodeHandle nh){
    //Declare parameters to be imported
    double desired_position_x, desired_position_y, desired_position_z;
    double inertia_11, inertia_12, inertia_13,
            inertia_21, inertia_22, inertia_23,
            inertia_31, inertia_32, inertia_33;
    double R_des_11, R_des_12, R_des_13,
            R_des_21, R_des_22, R_des_23,
            R_des_31, R_des_32, R_des_33;

    //Get the parameters
    bool read_all_parameters = nh.getParam("wrench_service_name", wrench_service_name_) &&
                               nh.getParam("mass", parameters.input_mass_) &&
                               nh.getParam("radius_", parameters.input_radius_) &&
                               nh.getParam("number_of_actuators_", parameters.input_number_of_actuators_) &&
                               nh.getParam("maximum_force_cog_", parameters.input_maximum_force_cog_) &&
                               nh.getParam("distance_integrator_treshold_", parameters.input_distance_integrator_treshold_) &&
                               nh.getParam("attitude_integrator_treshold_", parameters.input_attitude_integrator_treshold_) &&
                               nh.getParam("desired_position_x", desired_position_x) &&
                               nh.getParam("desired_position_y", desired_position_y) &&
                               nh.getParam("desired_position_z", desired_position_z) &&
                               nh.getParam("inertia_11", inertia_11) &&
                               nh.getParam("inertia_12", inertia_12) &&
                               nh.getParam("inertia_13", inertia_13) &&
                               nh.getParam("inertia_21", inertia_21) &&
                               nh.getParam("inertia_22", inertia_22) &&
                               nh.getParam("inertia_23", inertia_23) &&
                               nh.getParam("inertia_31", inertia_31) &&
                               nh.getParam("inertia_32", inertia_32) &&
                               nh.getParam("inertia_33", inertia_33) &&
                               nh.getParam("R_des_11", R_des_11) &&
                               nh.getParam("R_des_12", R_des_12) &&
                               nh.getParam("R_des_13", R_des_13) &&
                               nh.getParam("R_des_21", R_des_21) &&
                               nh.getParam("R_des_22", R_des_22) &&
                               nh.getParam("R_des_23", R_des_23) &&
                               nh.getParam("R_des_31", R_des_31) &&
                               nh.getParam("R_des_32", R_des_32) &&
                               nh.getParam("R_des_33", R_des_33);
    if (! read_all_parameters){
        ROS_ERROR("Parameters not imported");
        return false;
    }

    parameters.input_desired_position_<< desired_position_x,
            desired_position_y,
            desired_position_z;

    inertia_ << inertia_11, inertia_12, inertia_13,
            inertia_21, inertia_22, inertia_23,
            inertia_31, inertia_32, inertia_33;
    R_des_ << R_des_11, R_des_12, R_des_13,
            R_des_21, R_des_22, R_des_23,
            R_des_31, R_des_32, R_des_33;


    parameters.input_inertia_ = inertia_ ;
    parameters.input_R_des_ << R_des_11, R_des_12, R_des_13,
            R_des_21, R_des_22, R_des_23,
            R_des_31, R_des_32, R_des_33;

    // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
    // of the node can be run simultaneously while using different parameters.
    ros::NodeHandle pnh("~");
    pnh.param("k_x", k_x, k_x);
    pnh.param("k_v", k_v, k_v);
    pnh.param("k_R", k_R, k_R);
    pnh.param("k_omega", k_omega, k_omega);
    pnh.param("k_if", k_if, k_if);
    pnh.param("k_im", k_im, k_im);


    parameters.input_k_x_ = k_x;
    parameters.input_k_v_ = k_v;
    parameters.input_k_omega_ = k_omega;
    parameters.input_k_R_ = k_R;
    parameters.input_k_if_ = k_if;
    parameters.input_k_im_ = k_im;

    Eigen::Vector3d zero_v_;
    zero_v_ << 0,0,0;
    parameters.input_desired_acceleration_ = zero_v_;
    parameters.input_desired_angular_acceleration_ = zero_v_;
    parameters.input_desired_angular_velocity_ = zero_v_;
    parameters.input_desired_velocity_ = zero_v_;

    return true;

}


PoseControllerNode::PoseControllerNode(ros::NodeHandle nh){

    this->ParseParameters(nh);

    // Configure callback for dynamic parameters
    cb = boost::bind(&PoseControllerNode::ConfigCallback, this, _1, _2);
    dr_srv_.setCallback(cb);

    //Initialize the controller passing all the parameters
    geometric_controller.InitializeParams(parameters);

    //Setup service for control input if the service is ready
    ros::service::waitForService(wrench_service_name_);
    wrench_service_ = nh.serviceClient<skye_ros::ApplyWrenchCogBf>(wrench_service_name_, true);
}

PoseControllerNode::~PoseControllerNode (){};

void PoseControllerNode::ConfigCallback(skye_controls::skye_paramsConfig& config, uint32_t level)
{
    // Set class variables to new values. They match what is input at the dynamic reconfigure GUI.
    k_x = config.k_x;
    k_v = config.k_v;
    k_R = config.k_R;
    k_omega = config.k_omega;
    k_if = config.k_if;
    k_im = config.k_im;
}


void PoseControllerNode::AngularVelocityCallback(const sensor_msgs::Imu::ConstPtr& msg){
    angular_velocity_ << msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z;

    //TODO: check frame, is this correct?
}

void PoseControllerNode::PositionCallback(const gazebo_msgs::LinkState::ConstPtr& msg){

    position_ <<msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z;

    velocity_ << msg->twist.linear.x,
            msg->twist.linear.y,
            msg->twist.linear.z;

    orientation_.x() = msg->pose.orientation.x;
    orientation_.y() = msg->pose.orientation.y;
    orientation_.z() = msg->pose.orientation.z;
    orientation_.w() = msg->pose.orientation.w;

    geometric_controller.UpdateGains(k_x,k_v, k_if, k_im, k_R, k_omega);
    geometric_controller.UpdateParameters(position_, velocity_, orientation_, angular_velocity_);
    geometric_controller.ComputeForce(control_force_);
    geometric_controller.ComputeAcceleration(control_acceleration_);
    control_momentum_ = inertia_*control_acceleration_;


    /******************** DEBUG *************************/
    std::cout << "force: " << control_force_(0) <<
                 " | y: " << control_force_(1) <<
                 " | z: " << control_force_(2) <<
                 std::endl;

    std::cout << "acceleration: " << control_acceleration_(0) <<
                 " | y: " << control_acceleration_(1) <<
                 " | z: " << control_acceleration_(2) <<
                 std::endl;

    std::cout << "momentum: " << control_momentum_(0) <<
                 " | y: " << control_momentum_(1) <<
                 " | z: " << control_momentum_(2) <<
                 std::endl;
    /******************** END DEBUG *************************/

}

bool PoseControllerNode::CallService(){
    //std::cout << "a: " << a_ << " b: " << b_ <<std::endl;
    srv_.request.start_time.nsec = 0;
    srv_.request.duration.sec =  -1;

    temporaryWrench_.force.x =  control_force_(0);
    temporaryWrench_.force.y =  control_force_(1);
    temporaryWrench_.force.z =  control_force_(2);
    temporaryWrench_.torque.x = control_momentum_(0);
    temporaryWrench_.torque.y = control_momentum_(1);
    temporaryWrench_.torque.z = control_momentum_(2);

    srv_.request.wrench = temporaryWrench_;

    if (!wrench_service_.call(srv_)){
        ROS_ERROR("Failed to contact service. Could not pass wrench");
    }
}

int main(int argc, char **argv){
    //Init ros and create node handle
    ros::init(argc, argv, "skye_position_controller_node");
    ros::NodeHandle nh;

    // Import ros parameter for service and topic names
    std::string wrench_service_name, imu_topic, ground_truth_topic;
    bool read_all_parameters = nh.getParam("wrench_service_name", wrench_service_name) &&
                               nh.getParam("ground_truth_topic", ground_truth_topic) &&
                               nh.getParam("imu_topic", imu_topic);
    // Check if names have correctly been imported
    if (! read_all_parameters) ROS_ERROR("Parameters not imported");

    //Initialize node and subscribe to topics
    PoseControllerNode node_(nh);
    ros::Subscriber pos_sub_ground_truth = nh.subscribe (ground_truth_topic, 1, &PoseControllerNode::PositionCallback, &node_);
    ros::Subscriber pos_sub_imu = nh.subscribe (imu_topic, 1, &PoseControllerNode::AngularVelocityCallback, &node_);

    ros::Rate r(50); // 50 hz
    while (nh.ok())
    {
        node_.CallService();
        ros::spinOnce();
        r.sleep();
    }

}
