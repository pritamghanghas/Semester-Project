#include <skye_teach_and_repeat/skye_teach_and_repeat_node.h>

SkyeTeachAndRepeatNode::SkyeTeachAndRepeatNode()
{
}

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

//    //Initialize node and subscribe to topics
//    PoseControllerNode node(nh);
//    ros::Subscriber pos_sub_ground_truth = nh.subscribe (ground_truth_topic, 1, &PoseControllerNode::PositionCallback, &node);
//    ros::Subscriber pos_sub_imu = nh.subscribe (imu_topic, 1, &PoseControllerNode::AngularVelocityCallback, &node);

    ros::Rate r(50); // 50 hz
    while (nh.ok())
    {
//        if (!node.CallService()) return -1;
        ros::spinOnce();
        r.sleep();
    }

}
