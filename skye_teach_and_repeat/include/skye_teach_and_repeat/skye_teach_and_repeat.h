 #ifndef SKYE_TEACH_AND_REPEAT_H
#define SKYE_TEACH_AND_REPEAT_H

#include <Eigen/Eigen>
#include <vector>
#include <chrono>

#include <skye_controls/skye_geometric_controller.h>
#include <skye_controls/skye_paramsConfig.h>
#include <skye_controls/waypoint_controller.h>

#define TEACH_MODE 1
#define REPEAT_MODE 2
#define STANDBY_MODE 3
#define SPACE_TEACHING_MODE 1
#define TIME_TEACHING_MODE 2

/**
 * @brief The SkyeWaypoint struct wraps a single waypoint of Skye for easy parameters exchange
 */
struct SkyeWaypoint{
  /**
   * @brief waypoint_time : time at which the waypoint was visited
   */
  long double waypoint_time;
  /**
   * @brief waypoint_position_if : position coordinates of the waypoint in the inertial frame
   */
  Eigen::Vector3d waypoint_position_if;
  /**
   * @brief waypoint_velocity_if : velocities values of the waypoint in the inertial frame
   */
  Eigen::Vector3d waypoint_velocity_if;
  /**
   * @brief waypoint_angular_velocity_bf : angular velocity of the waypoint in the body frame
   */
  Eigen::Vector3d waypoint_angular_velocity_bf;
  /**
   * @brief waypoint_acceleration_bf : accelartion of the waypoint in the body frame
   */
  Eigen::Vector3d waypoint_acceleration_bf;
  /**
   * @brief waypoint_orientation_if : orientation quaternion of the waypoint in the inertial frame
   */
  Eigen::Quaterniond waypoint_orientation_if;
};

/**
 * @brief The SkyeAction struct represents one of the actions saved in the memory.
 * It is composed by an action ID and the trajectory itself, a vector of SkyeWaypoint
 */
struct SkyeAction{
  int action_id;
  std::vector<SkyeWaypoint> action_trajectory;
};

/**
 * @brief The SkyeTeachAndRepeat class is the Teach and Repeat Library, it provides
 * functionalities to change mode, assign new actions to be repeated, initialize and update
 * the teach and repeat object parameters. Then, during the execution, only one function should be called
 * i.e. ExecuteTeachAndRepeat.
 */
class SkyeTeachAndRepeat
{
public:
  /**
   * @brief SkyeTeachAndRepeat : the constructor just initializise a couple of variables to zero
   * or false booleans.
   */
  SkyeTeachAndRepeat();
  ~SkyeTeachAndRepeat();

  //Setters
  /**
   * @brief CheckModeChange : gets a new mode for the teach and repeat as a parameter and checks if it is ok.
   * @param new_mode : the desired new mode that still needs to be checked
   * @return : if the change was made
   */
  bool CheckModeChange(int new_mode);
  /**
   * @brief AssignNewActionToRepeat : assigns a new action to repeat if this exists
   * @param action_to_repeat : it's the action ID that should be executed
   * @return : wether the change was applyed successfully.
   */
  bool AssignNewActionToRepeat(int action_to_repeat);
  /**
   * @brief InitializeParameters : Initializes the parameters at the beginning
   * @param waypoints_change_threshold_position : position threshold at which waypoints should be changed
   * @param waypoints_change_threshold_orientation : orientation threshold at which waypoints should be changed
   * @param orientation_sample_threshold : orientation norm threshold at which new waypoints should be sampled
   * @param position_sample_threshold : position norm threshold at which new waypoints should be sampled
   * @param teaching_mode : choice between norms-based sampling or time-based sampling
   * @param inertia : Skye's Inertia
   * @param geometric_params: Struct for the geometric controller parameters
   */
  void InitializeParameters(double waypoints_change_threshold_position,
                            double waypoints_change_threshold_orientation,
                            double orientation_sample_threshold,
                            double position_sample_threshold,
                            int teaching_mode,
                            const Eigen::Matrix3d& inertia,
                            SkyeParameters geometric_params);
  /**
   * @brief UpdateControllerParameters : updates the controller gains.
   * @param k_x : position error control coefficient
   * @param k_v : velocity error control coefficient
   * @param k_if : force integrator error control coefficient
   * @param k_im : acceleration integrator error control coefficient
   * @param k_R : orientation error control coefficient
   * @param k_omega : angular velocity error control coefficient
   */
  void UpdateControllerParameters(double k_x,
                                  double k_v,
                                  double k_if,
                                  double k_im,
                                  double k_R,
                                  double k_omega);

  //Execution
  /**
   * @brief ExecuteTeachAndRepeat main and only function for the execution of the teach and repeat.
   * @param position_if : current position of Skye
   * @param velocity_if : current velocity of Skye
   * @param angular_velocity_bf : current angular velocity of Skye
   * @param acceleration_if : current acceleration of Skye
   * @param orientation_if : current orientation of Skye
   * @param control_force_bf : output control force generated during the repeat phase
   * @param control_moment_bf : output control moment generated during the repeat phase
   *
   * If in teach phase, the control outputs are not touched so to allow parallel tasks.
   */
  void ExecuteTeachAndRepeat(const Eigen::Vector3d& position_if,
                             const Eigen::Vector3d& velocity_if,
                             const Eigen::Vector3d& angular_velocity_bf,
                             const Eigen::Vector3d& acceleration_if,
                             const Eigen::Quaterniond& orientation_if,
                             Eigen::Vector3d* control_force_bf,
                             Eigen::Vector3d* control_moment_bf);



private:

  //Manipulation
  /**
   * @brief PackParameters is a utility function that is needed to prepare the action to be repeated for the repeat phase.
   */
  void PackParameters();
  /**
   * @brief TeachPhase performs the teaching phase
   * @param position_if : current position of Skye
   * @param velocity_if : current velocity of Skye
   * @param angular_velocity_bf : current angular velocity of Skye
   * @param acceleration_if : current acceleration of Skye
   * @param orientation_if : current orientation of Skye
   *
   * During this phase, it is firstly assesed if there are actions and, if not, a new one is created.
   * This is also true for the first waypoint and the teach phase is able to understand wether there is a
   * new action being taught or not.
   */
  void TeachPhase(const Eigen::Vector3d& position_if,
                  const Eigen::Vector3d& velocity_if,
                  const Eigen::Vector3d& angular_velocity_bf,
                  const Eigen::Vector3d& acceleration_if,
                  const Eigen::Quaterniond& orientation_if);

  /**
   * @brief RepeatPhase computes outputs to repeat the trajectory
   * @param position_if : current position of Skye
   * @param velocity_if : current velocity of Skye
   * @param angular_velocity_bf : current angular velocity of Skye
   * @param orientation_if : current orientation of Skye
   * @param control_force_bf : output control force generated during the repeat phase
   * @param control_moment_bf : output control moment generated during the repeat phase
   *
   * RepeatPhase takes care of operating the waypoint controller and the geometric controller properly.
   * The current state information is passed and this functions is in charge of computing
   * the controller outputs.
   */
  void RepeatPhase(const Eigen::Vector3d& position_if,
                   const Eigen::Vector3d& velocity_if,
                   const Eigen::Vector3d& angular_velocity_bf,
                   const Eigen::Quaterniond& orientation_if,
                   Eigen::Vector3d* control_force_bf,
                   Eigen::Vector3d* control_acceleration_bf);


  //Waypoint Controller
  /**
   * @brief waypoints_parameters_ : struct for the waypoint controller parameters
   */
  WaypointControllerParameters waypoints_parameters_;
  /**
   * @brief waypoints_controller_ : object of the waypoint controller
   */
  WaypointController waypoints_controller_;

  //Geometric Controller
  /**
   * @brief geometric_controller_parameters_ : struct for the geometric controller parameters
   */
  SkyeParameters geometric_controller_parameters_;
  /**
   * @brief geometric_controller_ : object of the geometric controller
   */
  SkyeGeometricController geometric_controller_;
  /**
   * @brief saved_data_ : vector of SkyeActions containing all the actions
   */
  std::vector<SkyeAction> saved_data_;
  /**
   * @brief node_mode_ : current mode for the node, either teach, repeat or something else.
   * 1 = teach
   * 2 = repeat
   * 3 = standby
   * Everything else = error messages
   */
  int node_mode_; //Teach or repeat
  /**
   * @brief teaching_mode_ : choice between norm-based theaching and time-based teaching
   * Time based mode currently not developed
   */
  int teaching_mode_; //1 for space, 2 for time
  /**
   * @brief action_selected_ : the action (aka trajectory) to repeat
   */
  int action_selected_;
  /**
   * @brief has_teaching_just_started_ : boolean that says if it is the first time the teaching is running
   * This is used to save a new action with a new first waypoint
   */
  bool has_teaching_just_started_;
  /**
   * @brief are_parameters_initialized_ : this is needed to trigger the parameter packing of the waypoint controller
   */
  bool are_parameters_initialized_;
  /**
   * @brief waypoints_change_threshold_position_ : it's a threshold that triggers the waypoint change.
   * It is the comparison on the position error norm.
   */
  double waypoints_change_threshold_position_;
  /**
   * @brief waypoints_change_threshold_orientation_ : it's a threshold that triggers the waypoint change.
   * It is the comparison on the orientation error norm.
   */
  double waypoints_change_threshold_orientation_;
  /**
   * @brief orientation_sample_threshold_ : this says how closely the new waypoints should be sampled given an orientation change
   */
  double orientation_sample_threshold_;
  /**
   * @brief position_sample_threshold_ : this says how closely the new waypoints should be sampled given a position change
   */
  double position_sample_threshold_;
  /**
   * @brief elapsed_time_ : elapsed time during one loop, used for the teach phase
   */
  long double elapsed_time_;
  /**
   * @brief inertia_ : Inertia of Skye
   */
  Eigen::Matrix3d inertia_;
  /**
   * @brief teach_starting_time_ : high res clock for the starting time of the theach phase
   */
  std::chrono::high_resolution_clock::time_point teach_starting_time_;
  /**
   * @brief repeat_starting_time_ : high res clock for the starting time of the repeat phase
   */
  std::chrono::high_resolution_clock::time_point repeat_starting_time_;
  /**
   * @brief current_time_ : high res clock for the current time of the each iteration
   */
  std::chrono::high_resolution_clock::time_point current_time_;
  /**
   * @brief time_difference_ : duration object to compute the time different w.r.t. the startin time
   */
  std::chrono::duration<double> time_difference_;
};

#endif // SKYE_TEACH_AND_REPEAT_H
