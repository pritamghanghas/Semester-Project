#include <skye_teach_and_repeat/skye_teach_and_repeat_node.h>

SkyeTeachAndRepeatNode::SkyeTeachAndRepeatNode(ros::NodeHandle nh){
    node_mode_ = 0;
    teaching_done_ = false;
    //Get the parameters
    bool read_all_parameters = nh.getParam("wrench_service_name", wrench_service_name_) &&
                               nh.getParam("teaching_mode", teaching_mode_);
    // Check if Skye's parameters where imported
    if (! read_all_parameters){
        ROS_ERROR("Geometric Parameters not imported");
    }
}

SkyeTeachAndRepeatNode::~SkyeTeachAndRepeatNode(){
}

void SkyeTeachAndRepeatNode::TeachPhase(const Eigen::Vector3d& position_if,
                                        const Eigen::Vector3d& velocity_if,
                                        const Eigen::Vector3d& angular_velocity_bf,
                                        const Eigen::Quaterniond& orientation_if){
    if (teaching_mode_ == 1) { //space teaching mode
        //if just started save the first waypoint
        if (saved_data_.size() == 0) {
            SkyeWaypoint a_waypoint;
            a_waypoint.waypoint_position_if = position_if;
            a_waypoint.waypoint_velocity_if = velocity_if;
            a_waypoint.waypoint_angular_velocity_bf = angular_velocity_bf;
            a_waypoint.waypoint_orientation_if = orientation_if;

        }


        // check norm distance from last SkyeWaypoint
        // Save new waypoint into SkyeAction vector


    } else if (teaching_mode_ == 2) { //time teaching mode
        //if just started save the first waypoint
        // check time passed from last SkyeWaypoint
        // Save new waypoint into SkyeAction vector
    } else {
        ROS_ERROR("[Skye Teach And Repeat] Teaching mode is not set correctly. Please set it in the yaml parameter file");
    }

}

void SkyeTeachAndRepeatNode::RepeatPhase(){

}

void SkyeTeachAndRepeatNode::AngularVelocityCallback(const sensor_msgs::Imu::ConstPtr& msg){
    // Save angular velocity from IMU topic
    // this is in the imu frame but since it is a rigid body they are the same in terms of angular velocity
    angular_velocity_bf_ << msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z;
}

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

    if (node_mode_ == 1) {
        // teach phase
        this->TeachPhase(position_if_, velocity_if_, angular_velocity_bf_, orientation_if_);
        if (!teaching_done_) teaching_done_ = true;
    }
    else if (node_mode_ == 2) {
        //repeat phase
        this->RepeatPhase();
    }
}

void SkyeTeachAndRepeatNode::ExecuteActionCallback(const std_msgs::Int16::ConstPtr& msg){

}

void SkyeTeachAndRepeatNode::ModeSelectionCallback(const std_msgs::Int16::ConstPtr& msg){
    node_mode_ = msg->data;
}

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

int SkyeTeachAndRepeatNode::node_mode(){
    return node_mode_;
}

bool SkyeTeachAndRepeatNode::teaching_done(){
    return teaching_done_;
}

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
    if (! read_all_parameters) ROS_ERROR("Parameters not imported");

    //Advertise topics that need the callback afterwards
    std_msgs::Int16 zero; //(new std_msgs::Int16);
    zero.data = 0;
    ros::Publisher mode_sel_pub = nh.advertise<std_msgs::Int16>(mode_topic, 1);
    ros::Publisher repeat_choice_pub = nh.advertise<std_msgs::Int16>(choice_topic, 1);
    mode_sel_pub.publish(zero);
    repeat_choice_pub.publish(zero);

    //Initialize node and subscribe to topics
    SkyeTeachAndRepeatNode node(nh);
    ros::Subscriber pos_sub_ground_truth = nh.subscribe (ground_truth_topic, 1, &SkyeTeachAndRepeatNode::StateCallback, &node);
    ros::Subscriber pos_sub_imu = nh.subscribe (imu_topic, 1, &SkyeTeachAndRepeatNode::AngularVelocityCallback, &node);
    ros::Subscriber mode_sel_sub = nh.subscribe (mode_topic, 1, &SkyeTeachAndRepeatNode::ModeSelectionCallback, &node);
    ros::Subscriber choice_sel_sub = nh.subscribe (choice_topic, 1, &SkyeTeachAndRepeatNode::ExecuteActionCallback, &node);

    ros::Rate r(50); // 50 hz
    while (nh.ok())
    {
        if (node.node_mode() == 2) {
            if (!node.CallService()) return -1;
        }

        ros::spinOnce();
        r.sleep();
    }

}
