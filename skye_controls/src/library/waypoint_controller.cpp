#include <skye_controls/waypoint_controller.h>

WaypointController::WaypointController()
{
}

WaypointController::~WaypointController(){

}

void WaypointController::InitParameters(WaypointControllerParameters parameters){
    waypoints_ = parameters.input_waypoints;
    orientations_ = parameters.input_orientations;
    goal_change_threshold_ = parameters.input_goal_change_threshold;
}

void WaypointController::ComputeGoal(const Eigen::Vector3d &current_position_if,
                                     WaypointPose *new_pose){

    Eigen::Vector3d position_error_if;
    position_error_if << (waypoints_.at(0))(0) - current_position_if(0),
                        (waypoints_.at(0))(1) - current_position_if(1),
                        (waypoints_.at(0))(2) - current_position_if(2);

    if (position_error_if.norm() < goal_change_threshold_ && waypoints_.size()>1) {
        new_pose->position = waypoints_.at(1);
        new_pose->orientation = orientations_.at(1);
        waypoints_.erase(waypoints_.begin());
        orientations_.erase(orientations_.begin());
    } else {
        new_pose->position = waypoints_.at(0);
        new_pose->orientation = orientations_.at(0);
    }

    /******************** DEBUG *******************************************
     * WHEN REMOVING THIS CODE DO NOT FORGET TO REMOVE IOSTREAM INCLUSION
     * IN THE HEADER FILE!!!!
     *
    std::cout << "--------------------------------------------------" << std::endl <<
                 "position_: " << current_position_if(0) <<
                 " | y: " << current_position_if(1) <<
                 " | z: " << current_position_if(2) <<
                 std::endl << std::endl;

    std::cout << "new_goal: " << (new_pose->position)(0) <<
                 " | y: " << (new_pose->position)(1) <<
                 " | z: " << (new_pose->position)(2) <<
                 std::endl << std::endl;

    std::cout << "position_: " << current_position_if(0) <<
                 " | y: " << current_position_if(1) <<
                 " | z: " << current_position_if(2) <<
                 std::endl << std::endl;

    std::cout << "new_orientation: " << new_pose->orientation.x() <<
                 " | y: " << new_pose->orientation.y() <<
                 " | z: " << new_pose->orientation.z() <<
                 " | w: " << new_pose->orientation.w() <<
                 std::endl << std::endl;


    std::cout << "position_error_if.norm(): " << position_error_if.norm() <<
                 std::endl << std::endl;

    std::cout << "goal_change_threshold_: " << goal_change_threshold_ <<
                 std::endl << std::endl;

    /****/

}
