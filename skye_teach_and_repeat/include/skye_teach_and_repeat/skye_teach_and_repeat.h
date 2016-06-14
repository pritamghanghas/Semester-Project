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
#define SPACE_TEACHING_MODE 1
#define TIME_TEACHING_MODE 2

struct SkyeWaypoint{
  //TODO: add time
  long double waypoint_time;
  Eigen::Vector3d waypoint_position_if;
  Eigen::Vector3d waypoint_velocity_if;
  Eigen::Vector3d waypoint_angular_velocity_bf;
  Eigen::Vector3d waypoint_acceleration_bf;
  Eigen::Quaterniond waypoint_orientation_if;
};

struct SkyeAction{
  int action_id;
  std::vector<SkyeWaypoint> action_trajectory;
};


class SkyeTeachAndRepeat
{
public:
  SkyeTeachAndRepeat();
  ~SkyeTeachAndRepeat();

  //Getters
  int node_mode();

  //Setters
  bool CheckModeChange(int new_mode);
  bool AssignNewActionToRepeat(int action_to_repeat);
  void InitializeParameters(double waypoints_change_threshold_position,
                            double waypoints_change_threshold_orientation,
                            double orientation_sample_threshold,
                            double position_sample_threshold,
                            int teaching_mode,
                            const Eigen::Matrix3d& inertia,
                            SkyeParameters geometric_params);
  void UpdateControllerParameters(double k_x,
                                  double k_v,
                                  double k_if,
                                  double k_im,
                                  double k_R,
                                  double k_omega);

  //Manipulation
  void PackParameters();

  //Execution
  void ExecuteTeachAndRepeat(const Eigen::Vector3d& position_if,
                             const Eigen::Vector3d& velocity_if,
                             const Eigen::Vector3d& angular_velocity_bf,
                             const Eigen::Vector3d& acceleration_if,
                             const Eigen::Quaterniond& orientation_if,
                             Eigen::Vector3d* control_force_bf,
                             Eigen::Vector3d* control_acceleration_bf);

  void TeachPhase(const Eigen::Vector3d& position_if,
                  const Eigen::Vector3d& velocity_if,
                  const Eigen::Vector3d& angular_velocity_bf,
                  const Eigen::Vector3d& acceleration_if,
                  const Eigen::Quaterniond& orientation_if);

  void RepeatPhase(const Eigen::Vector3d& position_if,
                   const Eigen::Vector3d& velocity_if,
                   const Eigen::Vector3d& angular_velocity_bf,
                   const Eigen::Quaterniond& orientation_if,
                   Eigen::Vector3d* control_force_bf,
                   Eigen::Vector3d* control_acceleration_bf);


private:
  int node_mode_; //Teach or repeat
  int teaching_mode_; //1 for space, 2 for time
  bool has_teaching_just_started_;
  bool are_parameters_initialized_;
  bool already_printed_;
  double waypoints_change_threshold_position_;
  double waypoints_change_threshold_orientation_;
  double orientation_sample_threshold_;
  double position_sample_threshold_;

  std::chrono::high_resolution_clock::time_point teach_starting_time_;
  std::chrono::high_resolution_clock::time_point repeat_starting_time_;
  std::chrono::high_resolution_clock::time_point current_time_;

  std::chrono::duration<double> time_difference_;
  long double elapsed_time_, delay_;

  Eigen::Matrix3d inertia_;

  std::vector<SkyeAction> saved_data_;
  int action_selected_;

  //Waypoint Controller
  WaypointControllerParameters waypoints_parameters_;
  WaypointController waypoints_controller_;

  //Geometric Controller
  SkyeParameters geometric_controller_parameters_;
  SkyeGeometricController geometric_controller_;

};

#endif // SKYE_TEACH_AND_REPEAT_H
