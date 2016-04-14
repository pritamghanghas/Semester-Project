#include <pose_control_node.h>

PoseControllerNode::PoseControllerNode(ros::NodeHandle nh){

    //Declare parameters to be imported
    std::string wrench_service_name;
    double mass;
    double desired_position_x, desired_position_y, desired_position_z;
    double inertia_11,
            inertia_12,
            inertia_13,
            inertia_21,
            inertia_22,
            inertia_23,
            inertia_31,
            inertia_32,
            inertia_33;

    double R_des_11,
            R_des_12,
            R_des_13,
            R_des_21,
            R_des_22,
            R_des_23,
            R_des_31,
            R_des_32,
            R_des_33;

    Eigen::Vector3d desired_position_,desired_velocity_,
            desired_angular_velocity_,desired_angular_acceleration_,
            desired_acceleration_;

    //Get the parameters
    bool read_all_parameters = nh.getParam("wrench_service_name", wrench_service_name) &&
                               nh.getParam("mass", mass) &&
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
                               nh.getParam("R_des_33", inertia_33);

    desired_position_<< desired_position_x,
            desired_position_y,
            desired_position_z;

    inertia_ <<inertia_11, inertia_12, inertia_13,
            inertia_21, inertia_22, inertia_23,
            inertia_31, inertia_32, inertia_33;

    R_des_ <<R_des_11, R_des_12, R_des_13,
            R_des_21, R_des_22, R_des_23,
            R_des_31, R_des_32, R_des_33;


    if (! read_all_parameters) ROS_ERROR("Parameters not imported");

    cb = boost::bind(&PoseControllerNode::ConfigCallback, this, _1, _2);
    dr_srv_.setCallback(cb);

    // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
    // of the node can be run simultaneously while using different parameters.
    ros::NodeHandle pnh("~");
    pnh.param("k_x", k_x, k_x);
    pnh.param("k_v", k_v, k_v);
    pnh.param("k_R", k_R, k_R);
    pnh.param("k_omega", k_omega, k_omega);

    geometric_controller.InitializeParams(k_x, k_v, k_omega, k_R, mass,
                                          desired_position_,desired_velocity_,
                                          desired_angular_velocity_,desired_angular_acceleration_,
                                          desired_acceleration_, inertia_, R_des_);

    ros::service::waitForService(wrench_service_name);
    //Setup service
    wrench_service_ = nh.serviceClient<skye_ros::ApplyWrenchCogBf>(wrench_service_name, true);


}

PoseControllerNode::~PoseControllerNode (){};

void PoseControllerNode::ConfigCallback(skye_controls::skye_paramsConfig& config, uint32_t level)
{
    // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
    k_x = config.k_x;
    k_v = config.k_v;
    k_R = config.k_R;
    k_omega = config.k_omega;
}


void PoseControllerNode::AngularVelocityCallback(const sensor_msgs::Imu::ConstPtr& msg){
    angular_velocity_ << msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z;

    //TODO: check frame, IMU should be in IMU but i want BF
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

    geometric_controller.UpdateGains(k_x,k_v, k_R, k_omega);
    geometric_controller.UpdateParameters(position_, velocity_, orientation_, angular_velocity_);
    geometric_controller.ComputeForce(control_force_);
    geometric_controller.ComputeAcceleration(control_momentum_);

    control_momentum_ = inertia_*control_momentum_;
    /******************** DEBUG *************************/
    std::cout << "force: " << control_force_(0) <<
                 " | y: " << control_force_(1) <<
                 " | z: " << control_force_(2) <<
                 std::endl;

    std::cout << "momentum: " << control_momentum_(0) <<
                 " | y: " << control_momentum_(1) <<
                 " | z: " << control_momentum_(2) <<
                 std::endl;
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

    ros::init(argc, argv, "skye_position_controller_node");
    ros::NodeHandle nh;

    std::string wrench_service_name, imu_topic, ground_truth_topic;
    bool read_all_parameters = nh.getParam("wrench_service_name", wrench_service_name) &&
                               nh.getParam("ground_truth_topic", ground_truth_topic) &&
                               nh.getParam("imu_topic", imu_topic);

    if (! read_all_parameters) ROS_ERROR("Parameters not imported");

    PoseControllerNode node_(nh);

    ROS_INFO("ABOUT TO SUBSCRIBE TO GROUND TRUTH");
    ros::Subscriber pos_sub_grounf_truth = nh.subscribe (ground_truth_topic, 1, &PoseControllerNode::PositionCallback, &node_);
    ROS_INFO("ABOUT TO SUBSCRIBE TO IMU");
    ros::Subscriber pos_sub_imu = nh.subscribe (imu_topic, 1, &PoseControllerNode::AngularVelocityCallback, &node_);

    ros::Rate r(50); // 50 hz
    while (nh.ok())
    {
        node_.CallService();
        ros::spinOnce();
        r.sleep();
    }

}
