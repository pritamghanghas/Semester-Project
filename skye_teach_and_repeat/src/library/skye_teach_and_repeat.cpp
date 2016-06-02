#include <skye_teach_and_repeat/skye_teach_and_repeat.h>
//----------------------------------------------------------------------------------------
// Constructor - Destructor
//----------------------------------------------------------------------------------------
SkyeTeachAndRepeat::SkyeTeachAndRepeat() {
    node_mode_ = 0;
    action_selected_ = 0;
    are_parameters_initialized_ = false;
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

//----------------------------------------------------------------------------------------
// Setters
//----------------------------------------------------------------------------------------
bool SkyeTeachAndRepeat::CheckModeChange(int new_mode){
    if (node_mode_ != new_mode) {
        node_mode_ = new_mode;
        has_teaching_just_started_ = true;
        if (new_mode == REPEAT_MODE && action_selected_ >0 && !are_parameters_initialized_) {
            std::cout << "saved_data size: " << saved_data_.size()
                      << " | action_selected_: " << action_selected_
                      << "Packing parameters....";
            this->PackParameters();
            waypoints_parameters_.input_goal_change_threshold = waypoints_distance_threshold_;
            waypoints_controller_.InitParameters(waypoints_parameters_);
            are_parameters_initialized_ = true;
            std::cout << " ... Done!" << std::endl;

        } else {
            are_parameters_initialized_ = false;
        }
        return true;
    }
    return false;
}

bool SkyeTeachAndRepeat::AssignNewActionToRepeat(int action_to_repeat){
    if (action_to_repeat < 1) {
        ROS_ERROR("Warning: action < 1, invalid, try again");
        return false;
    } else if (action_to_repeat > saved_data_.size()) {
        ROS_ERROR("Warning: action not set, action # > action saved");
        return false;
    } else {
        action_selected_ = action_to_repeat;
        if (node_mode_ == REPEAT_MODE && !are_parameters_initialized_) {
            std::cout << "saved_data size: " << saved_data_.size()
                      << " | action_selected_: " << action_selected_
                      << " | Packing parameters....";
            this->PackParameters();
            waypoints_parameters_.input_goal_change_threshold = waypoints_distance_threshold_;
            waypoints_controller_.InitParameters(waypoints_parameters_);
            std::cout << " ... Done!" << std::endl;
            are_parameters_initialized_ = true;


        } else {
            are_parameters_initialized_ = false;
        }


        return true;
    }
}

void SkyeTeachAndRepeat::InitializeParameters(double waypoints_threshold,
                                              double orientation_threshold,
                                              int teaching_mode,
                                              const Eigen::Matrix3d& inertia,
                                              SkyeParameters geometric_params) {
    waypoints_distance_threshold_ =  waypoints_threshold;
    orientation_distance_threshold_ = orientation_threshold;
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
        std::cout << "got request to pack parameters" << std::endl;
    SkyeAction action_to_repeat;

        std::cout << "Found the action to repeat, it is #: " << action_selected_ - 1 << std::endl;
        std::cout << "saved_data_.size(): " << saved_data_.size()<< std::endl;
    action_to_repeat =  saved_data_.at(action_selected_ - 1);
        std::cout << "Found the action to repeat, it is #: " << action_selected_ - 1 << std::endl;
        std::cout << "action_to_repeat->action_trajectory.size(): " << action_to_repeat.action_trajectory.size() << std::endl;
    // Go through all the waypoints
    for (int i = 0; i < action_to_repeat.action_trajectory.size(); i++) {
        std::cout << "DIOPORCO i: " << i << std::endl;
        //Get current waypoint
        SkyeWaypoint current_waypoint;
        current_waypoint = action_to_repeat.action_trajectory.at(i);

        // Get current waypoint data
        Eigen::Vector3d position, velocity, angular_velocity;
        Eigen::Quaterniond orientation;
        position = current_waypoint.waypoint_position_if;
        orientation = current_waypoint.waypoint_orientation_if;
        velocity = current_waypoint.waypoint_velocity_if;
        angular_velocity= current_waypoint.waypoint_angular_velocity_bf;

        //Save stuff into waypoints_parameters struct
        waypoints_parameters_.input_positions.push_back(position);
        waypoints_parameters_.input_velocities.push_back(velocity);
        waypoints_parameters_.input_angular_velocities.push_back(angular_velocity);
        waypoints_parameters_.input_orientations.push_back(orientation);
        //TODO implement interaces in geometric controller and other controller
    }
    std::cout << "Finished packing parameters, leaving." << std::endl;

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

    if (node_mode_ == TEACH_MODE) {
        // teach phase
        this->TeachPhase(position_if, velocity_if, angular_velocity_bf, orientation_if);
    }
    else if (node_mode_ == REPEAT_MODE) {
        if (saved_data_.size() > 0 && action_selected_ > 0) {

            //repeat phase
            if (are_parameters_initialized_) {
                repeat_starting_time_= std::chrono::high_resolution_clock::now();
                this->RepeatPhase(position_if, velocity_if, angular_velocity_bf,
                                  orientation_if, control_force_bf, control_acceleration_bf);
            }
        } else if (node_mode_ == 3){
            std::cout << "------------------------------------------------------" << std::endl
                         << " | number of actions: " << saved_data_.size() << std::endl;
               for (int i = 0; i < saved_data_.size(); ++i) {
                   std::cout << "Action: [" << i
                             << "]  with ID: [" << saved_data_.at(i).action_id
                             <<"] has this amount of wayp: " << saved_data_.at(i).action_trajectory.size()
                            << std::endl;
               }
               std::vector<SkyeWaypoint> temp = saved_data_.at(action_selected_-1).action_trajectory;
               for (int i = 0; i < temp.size(); ++i) {
                   std::cout << "P[" << i
                             << "]  position: "
                             << temp.at(i).waypoint_position_if(0) << ", "
                             << temp.at(i).waypoint_position_if(1) << ", "
                             << temp.at(i).waypoint_position_if(2) << std::endl
                             << "]  velocity: "
                             << temp.at(i).waypoint_velocity_if(0) << ", "
                             << temp.at(i).waypoint_velocity_if(1) << ", "
                             << temp.at(i).waypoint_velocity_if(2) << std::endl
                             << "]  ang_velo: "
                             << temp.at(i).waypoint_angular_velocity_bf(0) << ", "
                             << temp.at(i).waypoint_angular_velocity_bf(1) << ", "
                             << temp.at(i).waypoint_angular_velocity_bf(2) << std::endl
                             << "]  orientat: "
                             << temp.at(i).waypoint_orientation_if.x() << ", "
                             << temp.at(i).waypoint_orientation_if.y() << ", "
                             << temp.at(i).waypoint_orientation_if.z() << ", "
                             << temp.at(i).waypoint_orientation_if.w() << std::endl
                            << std::endl;
               }

        } else {
            ROS_INFO("Please perform teaching phase first using the appropriate topic and select an action to repeat using the appropriate topic");
        }

    } else {
        ROS_INFO("Wrong mode chosen");
    }
}

void SkyeTeachAndRepeat::TeachPhase(const Eigen::Vector3d& position_if,
                                    const Eigen::Vector3d& velocity_if,
                                    const Eigen::Vector3d& angular_velocity_bf,
                                    const Eigen::Quaterniond& orientation_if){

    if (teaching_mode_ == 1) { //space teaching mode
        // if new action, add it
        if (has_teaching_just_started_) {
            teach_starting_time_ = std::chrono::high_resolution_clock::now();
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
//            new_waypoint.waypoint_time = time(0)*1000  - starting_time_ms_;
            current_time_ = std::chrono::high_resolution_clock::now();
            time_difference_ = std::chrono::duration_cast<std::chrono::duration<double>>(current_time_ - teach_starting_time_);
            new_waypoint.waypoint_time = time_difference_.count();

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

        Eigen::Quaterniond orientation_error;
        orientation_error.x() = current_trajectory->at(current_trajectory->size() -1 ).waypoint_orientation_if.x() - orientation_if.x();
        orientation_error.y() = current_trajectory->at(current_trajectory->size() -1 ).waypoint_orientation_if.y() - orientation_if.y();
        orientation_error.z() = current_trajectory->at(current_trajectory->size() -1 ).waypoint_orientation_if.z() - orientation_if.z();
        orientation_error.w() = current_trajectory->at(current_trajectory->size() -1 ).waypoint_orientation_if.w() - orientation_if.w();


        if (distance.norm() > waypoints_distance_threshold_ ||
                orientation_error.norm() > orientation_distance_threshold_) {

            // Save new waypoint into SkyeAction vector
            SkyeWaypoint new_waypoint;
//            new_waypoint.waypoint_time = time(0)*1000 - starting_time_ms_;
            current_time_ = std::chrono::high_resolution_clock::now();
            time_difference_ = std::chrono::duration_cast<std::chrono::duration<double>>(current_time_ - teach_starting_time_);
            new_waypoint.waypoint_time = time_difference_.count();

            new_waypoint.waypoint_position_if = position_if;

            new_waypoint.waypoint_velocity_if = velocity_if;
            new_waypoint.waypoint_angular_velocity_bf = angular_velocity_bf;
//            new_waypoint.waypoint_velocity_if << 0,0,0;
//            new_waypoint.waypoint_angular_velocity_bf << 0,0,0;

            new_waypoint.waypoint_orientation_if = orientation_if;
            last_action->action_trajectory.push_back(new_waypoint);
            std::cout << "Size: " << last_action->action_trajectory.size()
                      << " | dist.norm: " << distance.norm()
                      << " | time: " << new_waypoint.waypoint_time
                      << " | pos: "
                      <<  position_if(0) << ", "
                      <<  position_if(1) << ", "
                      <<  position_if(2)
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

    //call waypoint controller and give it the waypoints, set some parameters
    WaypointPose new_pose;
    new_pose.position = position_if;
    new_pose.velocity = velocity_if;
    new_pose.angular_velocity = angular_velocity_bf;
    new_pose.orientation = orientation_if;

    waypoints_controller_.ComputeGoalPosition(position_if, orientation_if ,&new_pose);

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


    current_time_= std::chrono::high_resolution_clock::now();
    time_difference_ = std::chrono::duration_cast<std::chrono::duration<double>>(current_time_ - repeat_starting_time_);
    elapsed_time_ = time_difference_.count();

    /********************* DEBUG *************************/
    std::cout << "*************** GOAL DATA ***************" <<  std::endl <<
                 "POSITION: " << new_pose.position(0) <<
                 " | y: " << new_pose.position(1) <<
                 " | z: " << new_pose.position(2) <<
                 std::endl << std::endl;


    std::cout << "new_pose.velocity: " << new_pose.velocity(0) <<
                 " | y: " << new_pose.velocity(1) <<
                 " | z: " << new_pose.velocity(2) <<
                 std::endl <<  std::endl;

    std::cout << "new_pose.angular_velocity: " << new_pose.angular_velocity(0) <<
                 " | y: " << new_pose.angular_velocity(1) <<
                 " | z: " << new_pose.angular_velocity(2) <<
                 std::endl <<  std::endl;

    std::cout << "elapsed_time_: " << elapsed_time_ <<
                 std::endl << std::endl;

    std::cout << "new_pose.angular_velocity: " << new_pose.angular_velocity(0) <<
                 " | y: " << new_pose.angular_velocity(1) <<
                 " | z: " << new_pose.angular_velocity(2) <<
                 std::endl <<  std::endl;


    std::cout << "force: " << (*control_force_bf)(0) <<
                 " | y: " << (*control_force_bf)(1) <<
                 " | z: " << (*control_force_bf)(2) <<
                 std::endl;

    std::cout << "acceleration: " << control_acceleration_bf(0) <<
                 " | y: " << control_acceleration_bf(1) <<
                 " | z: " << control_acceleration_bf(2) <<
                 std::endl;

    std::cout << "momentum: " << (*control_momentum_bf)(0) <<
                 " | y: " << (*control_momentum_bf)(1) <<
                 " | z: " << (*control_momentum_bf)(2) <<
                 std::endl;


    /********************* END DEBUG *************************/

}

