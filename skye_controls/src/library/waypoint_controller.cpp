#include <skye_controls/waypoint_controller.h>

WaypointController::WaypointController()
{
}

WaypointController::~WaypointController(){

}

void WaypointController::InitParameters(WaypointControllerParameters some_param){
    waypoints_ = some_param.input_waypoints;
    goal_change_threshold_ = some_param.input_goal_change_threshold;
}

void WaypointController::ComputeGoal(const Eigen::Vector3d &current_position_if,
                                     Eigen::Vector3d *new_goal){

    Eigen::Vector3d position_error_if;
    position_error_if << (waypoints_.at(0))(0) - current_position_if(0),
                        (waypoints_.at(0))(1) - current_position_if(1),
                        (waypoints_.at(0))(2) - current_position_if(2);

    if (position_error_if.norm() < goal_change_threshold_ && waypoints_.size()>1) {
        *new_goal = waypoints_.at(1);
        waypoints_.erase(waypoints_.begin());
    } else {
        *new_goal = waypoints_.at(0);
    }

    /******************** DEBUG *************************/
    std::cout << "--------------------------------------------------" << std::endl <<
                 "position_: " << current_position_if(0) <<
                 " | y: " << current_position_if(1) <<
                 " | z: " << current_position_if(2) <<
                 std::endl << std::endl;

    std::cout << "new_goal: " << (*new_goal)(0) <<
                 " | y: " << (*new_goal)(1) <<
                 " | z: " << (*new_goal)(2) <<
                 std::endl << std::endl;

    std::cout << "position_error_if.norm(): " << position_error_if.norm() <<
                 std::endl << std::endl;

    std::cout << "goal_change_threshold_: " << goal_change_threshold_ <<
                 std::endl << std::endl;



}
