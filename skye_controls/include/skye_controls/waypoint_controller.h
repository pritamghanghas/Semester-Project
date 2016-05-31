#ifndef WAYPOINT_CONTROLLER_H
#define WAYPOINT_CONTROLLER_H

#include <Eigen/Eigen>
#include <vector>
#include <iostream>// remove once debug is removed!!!

/**
 * @brief The WaypointControllerParameters struct provides a thin interface to pass parameters.
 *
 * It provides an easy way to pass waypoints, orientation and a threshold
 */
struct WaypointControllerParameters{
  std::vector<Eigen::Vector3d> input_positions;
  std::vector<Eigen::Vector3d> input_velocities;
  std::vector<Eigen::Vector3d> input_angular_velocities;
  std::vector<Eigen::Quaterniond> input_orientations;
  double input_goal_change_threshold;
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
  void InitParameters(WaypointControllerParameters parameters);
  /**
     * @brief ComputeGoal : computes a new goal given the actual position of skye
     * @param current_position_if : the current position vector expressed in the inertial frame
     * @param new_pose : the output vector passed as a new Waypose
     */
  void ComputeGoalPosition(const Eigen::Vector3d &current_position_if,
                           const Eigen::Quaterniond &current_orientation_if,
                           WaypointPose *new_pose);
private:
  double goal_change_threshold_;
  double orientation_change_threshold_;
  std::vector<Eigen::Vector3d> positions_;
  std::vector<Eigen::Vector3d> velocities_;
  std::vector<Eigen::Vector3d> angular_velocities_;
  std::vector<Eigen::Quaterniond>  orientations_;
  WaypointControllerParameters controller_parameters_;
};

#endif // WAYPOINT_CONTROLLER_H
