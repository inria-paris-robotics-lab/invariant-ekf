/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   RobotState.h
 *  @author Ross Hartley
 *  @brief  Source file for RobotState
 *  @date   September 25, 2018
 **/

#include "inekf/RobotState.hpp"

namespace inekf {

using namespace std;

// Default constructor
RobotState::RobotState()
    : X_(Eigen::MatrixXd::Identity(5, 5)), Theta_(Eigen::MatrixXd::Zero(6, 1)),
      P_(Eigen::MatrixXd::Identity(15, 15)) {}
// Initialize with X
RobotState::RobotState(const Eigen::MatrixXd &X)
    : X_(X), Theta_(Eigen::MatrixXd::Zero(6, 1)) {
  P_ = Eigen::MatrixXd::Identity(3 * this->dimX() + this->dimTheta() - 6,
                                 3 * this->dimX() + this->dimTheta() - 6);
}
// Initialize with X and Theta
RobotState::RobotState(const Eigen::MatrixXd &X, const Eigen::VectorXd &Theta)
    : X_(X), Theta_(Theta) {
  P_ = Eigen::MatrixXd::Identity(3 * this->dimX() + this->dimTheta() - 6,
                                 3 * this->dimX() + this->dimTheta() - 6);
}
// Initialize with X, Theta and P
RobotState::RobotState(const Eigen::MatrixXd &X, const Eigen::VectorXd &Theta,
                       const Eigen::MatrixXd &P)
    : X_(X), Theta_(Theta), P_(P) {}
// TODO: error checking to make sure dimensions are correct and supported

Eigen::Ref<const Eigen::MatrixXd> RobotState::getX() const { return X_; }
Eigen::Ref<const Eigen::VectorXd> RobotState::getTheta() const {
  return Theta_;
}
Eigen::Ref<const Eigen::MatrixXd> RobotState::getP() const { return P_; }
Eigen::Ref<const Eigen::Matrix3d> RobotState::getRotation() const {
  return X_.block<3, 3>(0, 0);
}
Eigen::Ref<const Eigen::Vector3d> RobotState::getVelocity() const {
  return X_.block<3, 1>(0, 3);
}
Eigen::Ref<const Eigen::Vector3d> RobotState::getPosition() const {
  return X_.block<3, 1>(0, 4);
}
Eigen::Ref<const Eigen::Vector3d> RobotState::getGyroscopeBias() const {
  return Theta_.head(3);
}
Eigen::Ref<const Eigen::Vector3d> RobotState::getAccelerometerBias() const {
  return Theta_.tail(3);
}
long RobotState::dimX() const { return X_.cols(); }
long RobotState::dimTheta() const { return Theta_.rows(); }
long RobotState::dimP() const { return P_.cols(); }

void RobotState::setX(Eigen::Ref<const Eigen::MatrixXd> X) { X_ = X; }
void RobotState::setTheta(Eigen::Ref<const Eigen::VectorXd> Theta) {
  Theta_ = Theta;
}
void RobotState::setP(Eigen::Ref<const Eigen::MatrixXd> P) { P_ = P; }
void RobotState::setRotation(Eigen::Ref<const Eigen::Matrix3d> R) {
  X_.block<3, 3>(0, 0) = R;
}
void RobotState::setVelocity(Eigen::Ref<const Eigen::Vector3d> v) {
  X_.block<3, 1>(0, 3) = v;
}
void RobotState::setPosition(Eigen::Ref<const Eigen::Vector3d> p) {
  X_.block<3, 1>(0, 4) = p;
}
void RobotState::setGyroscopeBias(Eigen::Ref<const Eigen::Vector3d> bg) {
  Theta_.head(3) = bg;
}
void RobotState::setAccelerometerBias(Eigen::Ref<const Eigen::Vector3d> ba) {
  Theta_.tail(3) = ba;
}

void RobotState::copyDiagX(int n, Eigen::MatrixXd &BigX) {
  long dimX = this->dimX();
  for (int i = 0; i < n; ++i) {
    long startIndex = BigX.rows();
    BigX.conservativeResize(startIndex + dimX, startIndex + dimX);
    BigX.block(startIndex, 0, dimX, startIndex) =
        Eigen::MatrixXd::Zero(dimX, startIndex);
    BigX.block(0, startIndex, startIndex, dimX) =
        Eigen::MatrixXd::Zero(startIndex, dimX);
    BigX.block(startIndex, startIndex, dimX, dimX) = X_;
  }
  return;
}

ostream &operator<<(ostream &os, const RobotState &s) {
  os << "--------- Robot State -------------" << endl;
  os << "X:\n" << s.X_ << endl << endl;
  os << "Theta:\n" << s.Theta_ << endl << endl;
  os << "P:\n" << s.P_ << endl;
  os << "-----------------------------------";
  return os;
}

} // namespace inekf
