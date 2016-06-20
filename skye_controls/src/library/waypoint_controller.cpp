#include <skye_controls/waypoint_controller.h>
//---------------------------------------------------------------------------------------------------------
WaypointController::WaypointController(){
}

//---------------------------------------------------------------------------------------------------------
WaypointController::~WaypointController(){

}

//---------------------------------------------------------------------------------------------------------
bool WaypointController::InitParameters(WaypointControllerParameters parameters){
    //Store the parameters for the initialization
    positions_ = parameters.input_positions;
    velocities_ = parameters.input_velocities;
    angular_velocities_ = parameters.input_angular_velocities;
    accelerations_ = parameters.input_accelerations;
    orientations_ = parameters.input_orientations;
    goal_change_threshold_ = parameters.input_goal_change_threshold;
    orientation_change_threshold_ = parameters.input_orientation_change_threshold;

    //Double check that parameters are consistent in size
    if (positions_.size() != velocities_.size() ||
            velocities_.size() != angular_velocities_.size() ||
            angular_velocities_.size() != accelerations_.size() ||
            accelerations_.size() != orientations_.size() ) {
        return false;
    }
    return true;
}

//---------------------------------------------------------------------------------------------------------
void WaypointController::ComputeGoalPosition(const Eigen::Vector3d &current_position_if,
                                             const Eigen::Quaterniond &current_orientation_if,
                                             WaypointPose *new_pose){

    //Calculate position error
    Eigen::Vector3d position_error_if;
    position_error_if << (positions_.at(0))(0) - current_position_if(0),
            (positions_.at(0))(1) - current_position_if(1),
            (positions_.at(0))(2) - current_position_if(2);

    //Calculate orientation error
    Eigen::Quaterniond orientation_error;
    orientation_error.x() = orientations_.at(0).x() - current_orientation_if.x();
    orientation_error.y() = orientations_.at(0).y() - current_orientation_if.y();
    orientation_error.z() = orientations_.at(0).z() - current_orientation_if.z();
    orientation_error.w() = orientations_.at(0).w() - current_orientation_if.w();

    bool switch_condition = position_error_if.norm() < goal_change_threshold_;
//                            && orientation_error.norm() < orientation_change_threshold_;

    //check whether it is possible to switch and there are more than one waypoint left
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
    } else { //if false just return the last waypoint
        new_pose->position = positions_.at(0);
        new_pose->velocity = velocities_.at(0);
        new_pose->angular_velocity = angular_velocities_.at(0);
        new_pose->orientation = orientations_.at(0);
        new_pose->acceleration = accelerations_.at(0);
    }
}
