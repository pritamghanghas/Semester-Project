#ifndef WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROLLER_H

#include <Eigen/Eigen>
#include <vector>

/**
 * @brief The WaypointControllerParameters struct provides a thin interface to pass parameters.
 *
 * It provides an easy way to pass waypoints, orientation and a threshold
 */
struct WaypointControllerParameters{
  std::vector<Eigen::Vector3d> input_positions;
  std::vector<Eigen::Vector3d> input_velocities;
  std::vector<Eigen::Vector3d> input_angular_velocities;
  std::vector<Eigen::Vector3d> input_accelerations;
  std::vector<Eigen::Quaterniond> input_orientations;
  double input_position_change_threshold;
  double input_orientation_change_threshold;
};

/**
 * @brief The WaypointPose struct is used to pack position and orientation together
 * position is an Eigen::Vector3d and orientation is an Eigen::Quaterniond
 */
struct WaypointPose {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d acceleration;
  Eigen::Quaterniond orientation;
};

/**
 * @brief The WaypointController class takes care of providing new goals whenever a certain threshold is passed.
 *
 * before computing a new goal the class instance shoud have the parameters initialized with InitParameters
 */
class WaypointController
{
public:
  WaypointController();
  ~WaypointController();
  /**
     * @brief InitParameters : saves parameters in class members
     * @param parameters : a WaypointControllerParameters struct with parameters in it.
     */
  bool InitParameters(WaypointControllerParameters parameters);
  /**
     * @brief ComputeGoal : computes a new goal given the actual position of skye
     * @param current_position_if : the current position vector expressed in the inertial frame
     * @param new_pose : the output vector passed as a new Waypose
     */
  void ComputeGoalPosition(const Eigen::Vector3d &current_position_if,
                           const Eigen::Quaterniond &current_orientation_if,
                           WaypointPose *new_pose);
private:
  /**
   * @brief position_change_threshold_ : threshold on the position error that triggers the goal change.
   */
  double position_change_threshold_;
  /**
   * @brief orientation_change_threshold_ : threshold on the orientation that allows the goal change.
   */
  double orientation_change_threshold_;
  /**
   * @brief positions_ : vector containing the positions of the waypoint controllers.
   */
  std::vector<Eigen::Vector3d> positions_;
  /**
   * @brief velocities_ : vector containing all the velocities of the waypoints
   */
  std::vector<Eigen::Vector3d> velocities_;
  /**
   * @brief angular_velocities_ : vector containing all the angular velocities of the waypoints.
   */
  std::vector<Eigen::Vector3d> angular_velocities_;
  /**
   * @brief accelerations_ : vector containing all the accelerations of the waypoints.
   */
  std::vector<Eigen::Vector3d> accelerations_;
  /**
   * @brief orientations_ : vector  containing all the orientations of the waypoints.
   */
  std::vector<Eigen::Quaterniond>  orientations_;
};

#endif // WAYPOINT_CONTROLLER_H
