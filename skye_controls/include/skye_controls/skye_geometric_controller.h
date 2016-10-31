#ifndef SKYE_GEOMETRIC_CONTROLLER_H
#define SKYE_GEOMETRIC_CONTROLLER_H

#include <Eigen/Eigen>
#include <Eigen/Geometry>

/**
 * @brief The SkyeParameters struct this structure is used to pass every parameter from the outside,
 * making the initialization easier and scalable in case of future modifications
 */
struct SkyeParameters{
  /**
   * @brief input_number_of_actuators : the number of actuators skye has. It is used to compute the momentum saturation
   */
  int input_number_of_actuators;
  /**
   * @brief input_k_x : the position control gain
   */
  double input_k_x;
  /**
   * @brief input_k_v : the linear velocity control gain
   */
  double input_k_v;
  /**
   * @brief input_k_if : the force integrator control gain
   */
  double input_k_if;
  /**
   * @brief input_k_im : the momentum integrator control gain
   */
  double input_k_im;
  /**
   * @brief input_k_omega  : the angular velocity control gain
   */
  double input_k_omega;
  /**
   * @brief input_k_R : the attitude control gain
   */
  double input_k_R;
  /**
   * @brief input_mass : Skye's Mass
   */
  double input_mass;
  /**
   * @brief input_radius : Skye's radius
   */
  double input_radius;
  /**
   * @brief input_maximum_force_cog : unidirectional saturation value for the force
   */
  double input_maximum_force_cog;
  /**
   * @brief input_distance_integrator_treshold : threshold under which the position integrator is activated
   */
  double input_distance_integrator_treshold;
  /**
   * @brief input_attitude_integrator_treshold : threshold under which the attitude integrator is activated
   */
  double input_attitude_integrator_treshold;
  /**
   * @brief input_maximum_force_integrator : saturation value for the position control integrator
   */
  double input_maximum_force_integrator;
  /**
   * @brief input_maximum_momentum_integrator : saturation value for the attitude control integrator
   */
  double input_maximum_momentum_integrator;
  /**
   * @brief input_windup_force_threshold : threshold close to the setpoint when to perform the windup
   */
  double input_windup_force_threshold;
  /**
   * @brief input_windup_acceleration_threshold : threshold close to the setpoint when to perform the windup
   */
  double input_windup_acceleration_threshold;
  /**
   * @brief input_desired_position_if : desired position expressed in the inertial frame
   */
  Eigen::Vector3d input_desired_position_if;
  /**
   * @brief input_desired_velocity_if : desired linear velocity expressed in the inertial frame
   */
  Eigen::Vector3d input_desired_velocity_if;
  /**
   * @brief input_desired_acceleration_if : desired linear acceleration expressed in the inertial frame
   */
  Eigen::Vector3d input_desired_acceleration_if;
  /**
   * @brief input_desired_angular_velocity_bf : desired angular velocity expressed in the body frame
   */
  Eigen::Vector3d input_desired_angular_velocity_bf;
  /**
   * @brief input_desired_angular_acceleration_bf : desired angular acceleration expressed in the body frame
   */
  Eigen::Vector3d input_desired_angular_acceleration_bf;
  /**
   * @brief input_inertia : Skye's inertia
   */
  Eigen::Matrix3d input_inertia;
  /**
   * @brief input_R_des : desired rotation matrix expressed in the inertial frame
   */
  Eigen::Matrix3d input_R_des_if;
};

class SkyeGeometricController
{

public:
  /**
   * @brief SkyeGeometricController::SkyeGeometricController Just creates the object
   *
   */
  SkyeGeometricController();

  /**
   * @brief InitializeParams : function that initializes all the parameters needed by the controller
   * @param param : a SkyeParameters struct containing all the parameters
   */
  void InitializeParams(const SkyeParameters param);

  /**
   * @brief UpdateParameters : updates the parameters for control errors
   * @param poisiton_if_ : last known position of Skye in the inertial frame
   * @param velocity_if_ : last known linear velocity of Skye in the inertial frame
   * @param orientation_bf_ : last known orientation of Skye in the body frame
   * @param a_angular_velocity_bf_ : last known angular velocity of Skye in the body frame
   *
   * This must be called befor computing force and acceleration because it saves the updated parameters into the private variables
   * */
  void UpdateParameters(const Eigen::Vector3d &poisiton_if,
                        const Eigen::Vector3d &velocity_if,
                        const Eigen::Quaterniond &orientation_bf,
                        const Eigen::Vector3d &a_angular_velocity_bf);

  /**
   * @brief UpdateGains : Provides an interface to dynamically change control parameters
   * @param k_x : new position control gain
   * @param k_v : new linear velocity control gain
   * @param k_if : new force integrator control gain
   * @param k_im : new momentum integrator control gain
   * @param k_R : new attitude control gain
   * @param k_omega : new angular velocity control gain
   */
  void UpdateGains(double k_x,
                   double k_v,
                   double k_if,
                   double k_im,
                   double k_R,
                   double k_omega);
  /**
   * @brief ComputeForce : computes the force to control the position of Skye
   * @param[out] output_force_bf_ : the output force that is calculated
   */
  void ComputeForce(Eigen::Vector3d *output_force_bf);

  /**
   * @brief ComputeAcceleration : computes the momentum to control the orientation of Skye
   * @param output_acceleration_ : the output acceleration that is calculated
   */
  void ComputeAcceleration(Eigen::Vector3d *output_acceleration_bf);

  /**
   * @brief UpdateDesiredPose : updates the desired pose of Skye for new control iteration
   * @param desired_position : new desired position in inertial frame coordinates
   * @param desired_orientation : new desired orientation expressed as rotation of the inertial frame
   */
  void UpdateDesiredPose(const Eigen::Vector3d &desired_position_if,
                         const Eigen::Vector3d &desired_velocity_if,
                         const Eigen::Vector3d &desired_angular_velocity_bf,
                         const Eigen::Vector3d &desired_acceleration_if,
                         const Eigen::Quaterniond &desired_orientation_bf);

private:
  /**
   * @brief singularity_detected_ : boolean that shows wether there's been critical point
   */
  bool singularity_detected_;

  //Control coefficients - see geometric controller documentation
  /**
   * @brief k_x_ : the position control gain
   */
  double k_x_;
  /**
   * @brief k_v_ : the linear velocity control gain
   */
  double k_v_;
  /**
   * @brief k_if_ : the force integrator control gain
   */
  double k_if_;
  /**
   * @brief k_im_ : the momentum integrator control gain
   */
  double k_im_;

  /**
   * @brief k_omega_  : the angular velocity control gain
   */
  double k_omega_;

  /**
   * @brief k_R_ : the attitude control gain
   */
  double k_R_;
  /**
   * @brief normalized_k_R_ : attitude control coefficients normalized with the inertia
   */
  Eigen::Vector3d normalized_k_R_;
  /**
   * @brief normalized_k_omega_ : angular velocity control coefficients normalized with the inertia
   */
  Eigen::Vector3d normalized_k_omega_;


  //Skye's mass and radius and fixed parameters
  /**
   * @brief mass_ : Skye's Mass
   */
  double mass_;
  /**
   * @brief radius_ : Skye's radius
   */
  double radius_;
  /**
   * @brief number_of_actuators_ : the number of actuators skye has. It is used to compute the momentum saturation
   */
  int number_of_actuators_ ;


  // Saturation and thresholds
  /**
   * @brief maximum_force_cog_bf_ : unidirectional saturation value for the force
   */
  double maximum_force_cog_bf_;
  /**
   * @brief maximum_acceleration_cog_bf_ : threshold close to the setpoint when to perform the windup
   */
  double maximum_acceleration_cog_bf_;
  /**
   * @brief distance_integrator_treshold_ : threshold under which the position integrator is activated
   */
  double distance_integrator_treshold_;
  /**
   * @brief attitude_integrator_treshold_ : threshold under which the attitude integrator is activated
   */
  double attitude_integrator_treshold_;
  /**
   * @brief maximum_force_integrator_ : saturation value for the position control integrator
   */
  double maximum_force_integrator_;
  /**
   * @brief maximum_momentum_integrator_ : saturation value for the attitude control integrator
   */
  double maximum_acceleration_integrator_;

  // desired state data
  /**
   * @brief desired_position_if_ : desired position expressed in the inertial frame
   */
  Eigen::Vector3d desired_position_if_;
  /**
     * @brief desired_velocity_if_ : desired linear velocity expressed in the inertial frame
     */
  Eigen::Vector3d desired_velocity_if_;
  /**
     * @brief desired_acceleration_if_ : desired linear acceleration expressed in the inertial frame
     */
  Eigen::Vector3d desired_acceleration_if_;
  /**
     * @brief desired_angular_velocity_bf_ : desired angular velocity expressed in the body frame
     */
  Eigen::Vector3d desired_angular_velocity_bf_;
  /**
     * @brief desired_angular_acceleration_bf_ : desired angular acceleration expressed in the body frame
     */
  Eigen::Vector3d desired_angular_acceleration_bf_;

  // Current state data
  /**
   * @brief angular_velocity_ : current angular velocity
   */
  Eigen::Vector3d angular_velocity_;

  // Control errors
  /**
   * @brief position_error_bf_ : position error expressed in the body frame
   */
  Eigen::Vector3d position_error_bf_;
  /**
   * @brief position_error_if_ : position error expressed in the inertial frame
   */
  Eigen::Vector3d position_error_if_;
  /**
   * @brief velocity_error_bf_ : velocity error expressed in the body frame
   */
  Eigen::Vector3d velocity_error_bf_;
  /**
   * @brief velocity_error_if_ : velocity error expressed in the inertial frame
   */
  Eigen::Vector3d velocity_error_if_;
  /**
   * @brief attitude_error_bf_ : attitude error expressed in the body frame
   */
  Eigen::Vector3d attitude_error_bf_;
  /**
   * @brief angular_velocity_error_bf_ : angular velocity error expressed in the body frame
   */
  Eigen::Vector3d angular_velocity_error_bf_;
  /**
   * @brief integrated_position_error_ : position error integrated during time
   */
  Eigen::Vector3d integrated_position_error_;
  /**
   * @brief integrated_acceleration_error_ : orientation error integrated during time
   */
  Eigen::Vector3d integrated_orientation_error_;

  // Outputs
  /**
   * @brief resulting_force_ : final control output force saved for anti-windup back calculation
   */
  Eigen::Vector3d resulting_force_;
  /**
   * @brief resulting_acceleration_ : final control output acceleration saved for anti-windup back calculation
   */
  Eigen::Vector3d resulting_acceleration_;

  // Anti windup terms
  /**
   * @brief windup_force_ : windup force coming from anti-windup back calculation to be used for next step
   */
  Eigen::Vector3d windup_force_;
  /**
   * @brief windup_acceleration_ : windup acceleration coming from anti-windup back calculation to be used for next step
   */
  Eigen::Vector3d windup_acceleration_;
  /**
   * @brief windup_integrator_force_ : windup term for the integrator saturation
   *  coming from anti-windup back calculation to be used for next step
   */
  Eigen::Vector3d windup_integrator_force_;
  /**
   * @brief windup_integrator_acceleration_ : windup term for the integrator saturation
   *  coming from anti-windup back calculation to be used for next step
   */
  Eigen::Vector3d windup_integrator_acceleration_;
  /**
   * @brief unbounded_force_integrator_ : Integrator term unbounded to assess integrator saturation
   * and compute the antiwindup in the force computation
   */
  Eigen::Vector3d unbounded_force_integrator_;
  /**
   * @brief unbounded_acceleration_integrator_  : Integrator term unbounded to assess integrator saturation
   * and compute the antiwindup in the acceleration computation
   */
  Eigen::Vector3d unbounded_acceleration_integrator_;

  // Matrices
  /**
     * @brief inertia_ : Skye's inertia
     */
  Eigen::Matrix3d inertia_;
  /**
     * @brief R_des_if_ : desired rotation matrix expressed in the inertial frame
     */
  Eigen::Matrix3d R_des_if_;
  /**
   * @brief R_if_ : current rotation matrix in the inertial frame
   */
  Eigen::Matrix3d R_if_;
  /**
   * @brief R_temp_if_ : temporary rotation matrix used to solve the critical point issue
   */
  Eigen::Matrix3d R_temp_if_;



  /**
   * @brief VectorFromSkewMatrix : returns a vector from a skew matrix
   * @param skew_matrix : the input matrix
   * @param vector : the output vector
   */
  inline void VectorFromSkewMatrix(Eigen::Matrix3d &skew_matrix, Eigen::Vector3d *vector);

  /**
   * @brief SkyeGeometricController::ComputeNormalizedGains : normalizes the gains with the inertia of Skye in order to have inertia-independent calculations
   */
  inline void ComputeNormalizedGains();

  /**
   * @brief SaturateVector : small utility that saturates a vector component-wise given a threshold
   * @param a_vector_ : the vector to saturate
   * @param a_threshold_ : the threshold to saturate the vector
   */
  inline void SaturateVector(double a_threshold, Eigen::Vector3d *a_vector);


};

#endif // SKYE_GEOMETRIC_CONTROLLER_H
