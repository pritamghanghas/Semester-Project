#include <skye_teach_and_repeat/skye_teach_and_repeat.h>
//----------------------------------------------------------------------------------------
// Constructor - Destructor
//----------------------------------------------------------------------------------------
SkyeTeachAndRepeat::SkyeTeachAndRepeat() {
    node_mode_ = 0;
    teaching_done_ = false;
    has_teaching_just_started_ = true;

}

SkyeTeachAndRepeat::~SkyeTeachAndRepeat() {
}

//----------------------------------------------------------------------------------------
// Getters
//----------------------------------------------------------------------------------------
int SkyeTeachAndRepeat::node_mode(){
    return node_mode_;
}

bool SkyeTeachAndRepeat::teaching_done(){
    return teaching_done_;
}

//----------------------------------------------------------------------------------------
// Setters
//----------------------------------------------------------------------------------------
bool SkyeTeachAndRepeat::CheckModeChange(int new_mode){
    if (node_mode_ != new_mode) {
        node_mode_ = new_mode;
        has_teaching_just_started_ = true;
        return true;
    }
    return false;
}

bool SkyeTeachAndRepeat::AssignNewActionToRepeat(int action_to_repeat){
    if (action_to_repeat == 0) {
        return false;
    } else if (action_to_repeat > saved_data_.size()) {
        return false;
    } else {
        action_selected_ = action_to_repeat;
        this->PackParameters();
        waypoints_parameters_.input_goal_change_threshold = waypoints_distance_threshold_;
        waypoints_controller_.InitParameters(waypoints_parameters_);
        return true;
    }
}

void SkyeTeachAndRepeat::InitializeParameters(double waypoints_threshold,
                                              int teaching_mode,
                                              const Eigen::Matrix3d& inertia,
                                              SkyeParameters geometric_params) {
    waypoints_distance_threshold_ =  waypoints_threshold;
    teaching_mode_ = teaching_mode;
    inertia_ = inertia;
    geometric_controller_parameters_ = geometric_params;
    geometric_controller_.InitializeParams(geometric_params);
}

void SkyeTeachAndRepeat::UpdateControllerParameters(double k_x,
                                double k_v,
                                double k_if,
                                double k_im,
                                double k_R,
                                double k_omega){
    // Update the gains with new dynamic parameters
    geometric_controller_.UpdateGains(k_x ,k_v, k_if, k_im, k_R, k_omega);

}



//----------------------------------------------------------------------------------------
// Manipulation
//----------------------------------------------------------------------------------------
void SkyeTeachAndRepeat::PackParameters(){
    waypoints_parameters_.input_goal_change_threshold = 0.5;
    SkyeAction *action_to_repeat;
    *action_to_repeat =  saved_data_.at(action_selected_);
    std::vector<Eigen::Vector3d> waypoints;
    std::vector<Eigen::Quaterniond> orientations;
    for (int i = 0; i < action_to_repeat->action_trajectory.size(); ++i) {
        Eigen::Vector3d position, velocity, angular_velocity;
        Eigen::Quaterniond orientation;
        SkyeWaypoint *current_waypoint;
        *current_waypoint = action_to_repeat->action_trajectory.at(i);
        position = current_waypoint->waypoint_position_if;
        orientation = current_waypoint->waypoint_orientation_if;
        velocity = current_waypoint->waypoint_velocity_if;
        angular_velocity= current_waypoint->waypoint_angular_velocity_bf;
        waypoints_parameters_.input_positions.push_back(position);
        waypoints_parameters_.input_velocities.push_back(velocity);
        waypoints_parameters_.input_angular_velocities.push_back(angular_velocity);
        waypoints_parameters_.input_orientations.push_back(orientation);
        //TODO implement interaces in geometric controller and other controller
    }
}

//----------------------------------------------------------------------------------------
// Execution
//----------------------------------------------------------------------------------------
void SkyeTeachAndRepeat::ExecuteTeachAndRepeat(const Eigen::Vector3d& position_if,
                                               const Eigen::Vector3d& velocity_if,
                                               const Eigen::Vector3d& angular_velocity_bf,
                                               const Eigen::Quaterniond& orientation_if,
                                               Eigen::Vector3d* control_force_bf,
                                               Eigen::Vector3d* control_acceleration_bf){

    if (node_mode_ == 1) {
        // teach phase
        this->TeachPhase(position_if, velocity_if, angular_velocity_bf, orientation_if);
        if (!teaching_done_) teaching_done_ = true;
    }
    else if (node_mode_ == 2) {
        //repeat phase
        this->RepeatPhase(position_if, velocity_if, angular_velocity_bf,
                          orientation_if, control_force_bf, control_acceleration_bf);
    }
}

void SkyeTeachAndRepeat::TeachPhase(const Eigen::Vector3d& position_if,
                                    const Eigen::Vector3d& velocity_if,
                                    const Eigen::Vector3d& angular_velocity_bf,
                                    const Eigen::Quaterniond& orientation_if){

    if (teaching_mode_ == 1) { //space teaching mode
        // if new action, add it
        if (has_teaching_just_started_) {
            has_teaching_just_started_ = false;
            SkyeAction new_action;
            if (saved_data_.size() == 0) {
                new_action.action_id = 1;
            } else {
                new_action.action_id = saved_data_.back().action_id + 1;
            }
            saved_data_.push_back(new_action);
            std::cout << "Saved new action - number of actions: "
                      << saved_data_.size()
                      << std::endl;
        }
        int last_element = saved_data_.size() - 1;
        SkyeAction *last_action = &(saved_data_.at(last_element));

        //if just started save the first waypoint
        if (last_action->action_trajectory.size() == 0) {
            SkyeWaypoint new_waypoint;
            new_waypoint.waypoint_position_if = position_if;
            new_waypoint.waypoint_velocity_if = velocity_if;
            new_waypoint.waypoint_angular_velocity_bf = angular_velocity_bf;
            new_waypoint.waypoint_orientation_if = orientation_if;
            last_action->action_trajectory.push_back(new_waypoint);
            std::cout << "Saved FIRST waypoint in action: "
                      << last_action->action_id
                      << " with #waypoints: "
                      << last_action->action_trajectory.size()
                      << " | number of actions: " << saved_data_.size()
                      << std::endl;
        }
        // check norm distance from last SkyeWaypoint
        Eigen::Vector3d distance;
        std::vector<SkyeWaypoint> *current_trajectory = &(last_action->action_trajectory);
        distance = current_trajectory->at(current_trajectory->size() -1 ).waypoint_position_if - position_if;
        if (distance.norm() > waypoints_distance_threshold_) {

            // Save new waypoint into SkyeAction vector
            SkyeWaypoint new_waypoint;
            new_waypoint.waypoint_position_if = position_if;
            new_waypoint.waypoint_velocity_if = velocity_if;
            new_waypoint.waypoint_angular_velocity_bf = angular_velocity_bf;
            new_waypoint.waypoint_orientation_if = orientation_if;
            last_action->action_trajectory.push_back(new_waypoint);
            std::cout << "New W traj size: " << last_action->action_trajectory.size()
                      << " | dist.norm: " << distance.norm()
                      << " | number of actions: " << saved_data_.size()
                      << std::endl;
        }
        /*   */
    } else if (teaching_mode_ == 2) { //time teaching mode
        //if just started save the first waypointforce: 8.25136e-05 |
        // check time passed from last SkyeWaypoint
        // Save new waypoint into SkyeAction vector
    } else {
        ROS_ERROR("[Skye Teach And Repeat] Teaching mode is not set correctly. Please set it in the yaml parameter file");
    }

}

void SkyeTeachAndRepeat::RepeatPhase(const Eigen::Vector3d& position_if,
                                     const Eigen::Vector3d& velocity_if,
                                     const Eigen::Vector3d& angular_velocity_bf,
                                     const Eigen::Quaterniond& orientation_if,
                                     Eigen::Vector3d* control_force_bf,
                                     Eigen::Vector3d* control_momentum_bf){

    //    std::cout << "------------------------------------------------------" << std::endl
    //              << " | number of actions: " << saved_data_.size() << std::endl;
    //    for (int i = 0; i < saved_data_.size(); ++i) {
    //        std::cout << "Action: [" << i
    //                  << "]  with ID: [" << saved_data_.at(i).action_id
    //                  <<"] has this amount of wayp: " << saved_data_.at(i).action_trajectory.size()
    //                 << std::endl;
    //    }

    //call waypoint controller and give it the waypoints, set some parameters
    WaypointPose new_pose;
    new_pose.position = position_if;
    new_pose.velocity = velocity_if;
    new_pose.angular_velocity = angular_velocity_bf;
    new_pose.orientation = orientation_if;
    waypoints_controller_.ComputeGoalPosition(position_if, &new_pose);

    geometric_controller_.UpdateDesiredPose(new_pose.position, new_pose.velocity,
                                            new_pose.angular_velocity, new_pose.orientation);

    // Update last known state
    geometric_controller_.UpdateParameters(position_if, velocity_if, orientation_if, angular_velocity_bf);
    // Calculate control force
    geometric_controller_.ComputeForce(control_force_bf);
    // Calculate control acceleration
    Eigen::Vector3d control_acceleration_bf;
    geometric_controller_.ComputeAcceleration(&control_acceleration_bf);
    // Calculate control momentum
    (*control_momentum_bf) = inertia_*(control_acceleration_bf);


}

