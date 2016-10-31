#include "skye_ros/common.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <ros/console.h>

namespace gazebo {

bool getSdfMatrix(sdf::ElementPtr sdf, 
                  const std::string& matrix_name, 
                  Eigen::Ref<Eigen::MatrixXd> m, 
                  const bool& verbose) {

  if(sdf->HasElement(matrix_name)) {
    sdf::ElementPtr matrix_elem = sdf->GetElement(matrix_name);
    // read elements with name: ij; set to 0 unspecified ones
    for(int i = 0; i < m.rows(); i++) {
      for(int j = 0; j < m.cols(); j++) {

        // matrix elements start from 11 and not from 00, add 1 to indeces 
        std::string element_name = "elem_" + std::to_string(i+1) + std::to_string(j+1);

        if(matrix_elem->HasElement(element_name)){
          m(i,j) = matrix_elem->GetElement(element_name)->Get<double>();
        }
        else{
          m(i,j) = 0.0; // unspecified parameter, set matrix element to 0
        }
      }
    }

    return true;
  }
  
  if(verbose)
        ROS_INFO_STREAM(matrix_name << ": not specified");

  return false;
}


TransferFunction::TransferFunction(const int stateDimension, const int inputDimension){
  Ad_ = Eigen::MatrixXd::Zero(stateDimension,stateDimension);
  Bd_ = Eigen::MatrixXd::Zero(stateDimension,inputDimension);
  previousState_ = Eigen::MatrixXd::Zero(stateDimension,inputDimension);
  valid_matrices_ = false;
}

bool TransferFunction::UpdateState(const Eigen::Ref<const Eigen::MatrixXd> &inputSignal,
                                   Eigen::Ref<Eigen::MatrixXd> outputState) {
  /*
  This method will apply the filter on the system.
  */  
  if(valid_matrices_){
    outputState = Ad_ * previousState_ + Bd_ * inputSignal;
  }
  else{
    // set output to 0
    ROS_ERROR("Error: TransferFunction, matrix A and B not valid");
    outputState.Zero(previousState_.rows(), previousState_.cols());
  } 

  previousState_ = outputState;

  return valid_matrices_;
}

bool TransferFunction::DiscretizeSS(const Eigen::Ref<const Eigen::MatrixXd> &A, 
                                    const Eigen::Ref<const Eigen::MatrixXd> &B,
                                    const double &sample_time) {

  // check A and B have same dimensions of Ad_ and Bd_ set in the constructor
  if(A.rows() != Ad_.rows() || A.cols() != Ad_.cols() ||
     B.rows() != Bd_.rows() || B.cols() != Bd_.cols()){

    ROS_ERROR_STREAM("Error: TransferFunction, A must be " << Ad_.rows() << "x" << Ad_.cols() <<
                     " and B must be " << Bd_.rows() << "x" << Bd_.cols());
    valid_matrices_ = false;
  }
  else
    switch(A.rows()) {

      case 1: // A must be invertible to discretize the system with below short forumala
              if(A(0,0) == 0.0) {
                ROS_FATAL("Matrix A of TrasferFunction must be invertible");
                valid_matrices_ = false;
              }           
              Ad_(0,0) = exp(sample_time * A(0,0));
              Bd_(0,0) = (1.0 / A(0,0)) * (Ad_(0,0) - 1.0) * B(0,0);
              valid_matrices_ = true;
              break;

      case 2:{
              Eigen::Matrix2d A_inv;
              bool invertible;
              double determinant;
              Eigen::Matrix2d fix_size_A = A; 
              fix_size_A.computeInverseAndDetWithCheck(A_inv, determinant, invertible);
              // A must be invertible to discretize the system with below short forumala
              if(!invertible) {
                ROS_FATAL("Matrix A of TrasferFunction must be invertible");
                valid_matrices_ = false;
              } 
              // compute exponential matrix e^(A * sample_time)
              Eigen::Matrix2d tmp = fix_size_A * sample_time;
              Ad_ = tmp.exp();
              Bd_ = A_inv * (Ad_ - Eigen::MatrixXd::Identity(Ad_.rows(),Ad_.cols())) * B;
              valid_matrices_ = true;
              break;
            }

      default: ROS_ERROR("Error: TransferFunction, stateDimension must be either 1 or 2");
               valid_matrices_ = false;
    } 

  return valid_matrices_;
}

TransferFunction::~TransferFunction() {}

}