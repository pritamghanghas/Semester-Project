#include <skye_controls/waypoint_controller.h>

WaypointController::WaypointController()
{
}

WaypointController::~WaypointController(){

}

void WaypointController::InitParameters(WaypointControllerParameters parameters){
    positions_ = parameters.input_positions;
    velocities_ = parameters.input_velocities;
    angular_velocities_ = parameters.input_angular_velocities;
    accelerations_ = parameters.input_accelerations;
    orientations_ = parameters.input_orientations;
    goal_change_threshold_ = parameters.input_goal_change_threshold;
    orientation_change_threshold_ = parameters.input_orientation_change_threshold;
}

void WaypointController::ComputeGoalPosition(const Eigen::Vector3d &current_position_if,
                                             const Eigen::Quaterniond &current_orientation_if,
                                             WaypointPose *new_pose){

    Eigen::Vector3d position_error_if;
    position_error_if << (positions_.at(0))(0) - current_position_if(0),
            (positions_.at(0))(1) - current_position_if(1),
            (positions_.at(0))(2) - current_position_if(2);
    Eigen::Quaterniond orientation_error;
    orientation_error.x() = orientations_.at(0).x() - current_orientation_if.x();
    orientation_error.y() = orientations_.at(0).y() - current_orientation_if.y();
    orientation_error.z() = orientations_.at(0).z() - current_orientation_if.z();
    orientation_error.w() = orientations_.at(0).w() - current_orientation_if.w();

    bool switch_condition = position_error_if.norm() < goal_change_threshold_ ;
//                           && orientation_error.norm() < orientation_change_threshold_;

    if ( switch_condition && positions_.size()>1) {
        new_pose->position = positions_.at(1);
        new_pose->velocity = velocities_.at(1);
        new_pose->angular_velocity = angular_velocities_.at(1);
        new_pose->orientation = orientations_.at(1);
        new_pose->acceleration = accelerations_.at(1);
        positions_.erase(positions_.begin());
        velocities_.erase(velocities_.begin());
        angular_velocities_.erase(angular_velocities_.begin());
        orientations_.erase(orientations_.begin());
        accelerations_.erase(accelerations_.begin());
    } else {
        new_pose->position = positions_.at(0);
        new_pose->velocity = velocities_.at(0);
        new_pose->angular_velocity = angular_velocities_.at(0);
        new_pose->orientation = orientations_.at(0);
        new_pose->acceleration = accelerations_.at(0);
    }

    /******************** DEBUG *******************************************
     * WHEN REMOVING THIS CODE DO NOT FORGET TO REMOVE IOSTREAM INCLUSION
     * IN THE HEADER FILE!!!!
     */

    std::cout << "--------------------------------------------------" << std::endl <<
                 "orientation_error.norm(): " << orientation_error.norm() << std::endl <<
                 "--------------------------------------------------" <<
                 std::endl << std::endl;

    /*
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
