/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SKYE_GAZEBO_PLUGINS_COMMON_H_
#define SKYE_GAZEBO_PLUGINS_COMMON_H_

#include <Eigen/Dense>
#include <gazebo/gazebo.hh>
#include <Eigen/LU>
#include <ros/console.h>
#include <string> 
#include <assert.h>

namespace gazebo {

// Default values
static const std::string kDefaultNamespace = "";
static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;
const bool SKYE_GAZEBO_PLUGINS_VERBOSE = true;

/**
 * \brief Obtains a parameter from sdf.
 * \param[in] sdf Pointer to the sdf object.
 * \param[in] name Name of the parameter.
 * \param[out] param Param Variable to write the parameter to.
 * \param[in] default_value Default value, if the parameter not available.
 * \param[in] verbose If true
 */
template<class T>
bool getSdfParam(sdf::ElementPtr sdf, 
                 const std::string& name, 
                 T& param, 
                 const T& default_value, 
                 const bool& verbose = true){

  if (sdf->HasElement(name)) {
    param = sdf->GetElement(name)->Get<T>();

    if(verbose)
      ROS_INFO_STREAM(name << ": " << param);

    return true;
  }
  else {
    param = default_value;
    if (verbose)
      ROS_INFO_STREAM("Used default value for " << name << ": " << param);
      //gzerr << "[skye_gazebo_plugins] Please specify a value for parameter \"" << name << "\".\n";
  }
  return false;
}

template<class T>
bool getSdfQuaternion(sdf::ElementPtr sdf, 
                      const std::string& name, 
                      T& param, 
                      const T& default_value, 
                      const bool& verbose = true) {

  if (sdf->HasElement(name)) {
    sdf::ElementPtr quat_elem = sdf->GetElement(name);
    // check there are w, x, y and z elements
    if(quat_elem->HasElement("w") && quat_elem->HasElement("x") && 
       quat_elem->HasElement("y") && quat_elem->HasElement("z")){

      param.w = quat_elem->GetElement("w")->Get<double>();
      param.x = quat_elem->GetElement("x")->Get<double>();
      param.y = quat_elem->GetElement("y")->Get<double>();
      param.z = quat_elem->GetElement("z")->Get<double>();

      if(verbose)
        ROS_INFO_STREAM(name << ": [" << param.w << ", "
                                      << param.x << ", "
                                      << param.y << ", "
                                      << param.z << "]");

      return true;
    }
  }
  
  if(verbose)
        ROS_INFO_STREAM(name << ": [w, x, y, z] not specified");

  return false;
}

bool getSdfMatrix(sdf::ElementPtr sdf, 
                  const std::string& matrix_name, 
                  Eigen::Ref<Eigen::MatrixXd> m, 
                  const bool& verbose = true);

class TransferFunction {
/**
 * This class can be used to apply a filter on a signal.
 * Short reveiw :
 * continous time system:
 *   dx(t) = A*x(t) + B*u(t)
 * discretized system (ZoH):
 *   x(k+1) = Ad * x(k) + Bd * u(k), where
 * Ad = exp(A * Ts), and if A is invertible B = inv(A) * (Ad - I) * B
*/

  public:
    TransferFunction(const int stateDimension, const int inputDimension);

    bool UpdateState(const Eigen::Ref<const Eigen::MatrixXd> &inputSignal,
                     Eigen::Ref<Eigen::MatrixXd> outputState);

    bool DiscretizeSS(const Eigen::Ref<const Eigen::MatrixXd> &A, 
                      const Eigen::Ref<const Eigen::MatrixXd> &B,
                      const double &sample_time);
  
      ~TransferFunction();

  protected:
    Eigen::MatrixXd   Ad_;
    Eigen::MatrixXd   Bd_;
    Eigen::MatrixXd   previousState_;
    bool valid_matrices_;

};

}

#endif /* SKYE_GAZEBO_PLUGINS_COMMON_H_ */