/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   NoiseParams.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Invariant EKF noise parameter class
 *  @date   September 25, 2018
 **/

#include "inekf/NoiseParams.hpp"

namespace inekf {

using namespace std;

// ------------ NoiseParams -------------
// Default Constructor
NoiseParams::NoiseParams() {
  setGyroscopeNoise(0.01);
  setAccelerometerNoise(0.1);
  setGyroscopeBiasNoise(0.00001);
  setAccelerometerBiasNoise(0.0001);
  setLandmarkNoise(0.1);
  setContactNoise(0.1);
}

void NoiseParams::setGyroscopeNoise(double standard_dev) {
  Qg_ = standard_dev * standard_dev * Eigen::Matrix3d::Identity();
}
void NoiseParams::setGyroscopeNoise(
    Eigen::Ref<const Eigen::Vector3d> standard_dev) {
  Qg_ << standard_dev(0) * standard_dev(0), 0, 0, 0,
      standard_dev(1) * standard_dev(1), 0, 0, 0,
      standard_dev(2) * standard_dev(2);
}
void NoiseParams::setGyroscopeNoise(Eigen::Ref<const Eigen::Matrix3d> cov) {
  Qg_ = cov;
}

void NoiseParams::setAccelerometerNoise(double standard_dev) {
  Qa_ = standard_dev * standard_dev * Eigen::Matrix3d::Identity();
}
void NoiseParams::setAccelerometerNoise(
    Eigen::Ref<const Eigen::Vector3d> standard_dev) {
  Qa_ << standard_dev(0) * standard_dev(0), 0, 0, 0,
      standard_dev(1) * standard_dev(1), 0, 0, 0,
      standard_dev(2) * standard_dev(2);
}
void NoiseParams::setAccelerometerNoise(Eigen::Ref<const Eigen::Matrix3d> cov) {
  Qa_ = cov;
}

void NoiseParams::setGyroscopeBiasNoise(double standard_dev) {
  Qbg_ = standard_dev * standard_dev * Eigen::Matrix3d::Identity();
}
void NoiseParams::setGyroscopeBiasNoise(
    Eigen::Ref<const Eigen::Vector3d> standard_dev) {
  Qbg_ << standard_dev(0) * standard_dev(0), 0, 0, 0,
      standard_dev(1) * standard_dev(1), 0, 0, 0,
      standard_dev(2) * standard_dev(2);
}
void NoiseParams::setGyroscopeBiasNoise(Eigen::Ref<const Eigen::Matrix3d> cov) {
  Qbg_ = cov;
}

void NoiseParams::setAccelerometerBiasNoise(double standard_dev) {
  Qba_ = standard_dev * standard_dev * Eigen::Matrix3d::Identity();
}
void NoiseParams::setAccelerometerBiasNoise(
    Eigen::Ref<const Eigen::Vector3d> standard_dev) {
  Qba_ << standard_dev(0) * standard_dev(0), 0, 0, 0,
      standard_dev(1) * standard_dev(1), 0, 0, 0,
      standard_dev(2) * standard_dev(2);
}
void NoiseParams::setAccelerometerBiasNoise(
    Eigen::Ref<const Eigen::Matrix3d> cov) {
  Qba_ = cov;
}

void NoiseParams::setLandmarkNoise(double standard_dev) {
  Ql_ = standard_dev * standard_dev * Eigen::Matrix3d::Identity();
}
void NoiseParams::setLandmarkNoise(
    Eigen::Ref<const Eigen::Vector3d> standard_dev) {
  Ql_ << standard_dev(0) * standard_dev(0), 0, 0, 0,
      standard_dev(1) * standard_dev(1), 0, 0, 0,
      standard_dev(2) * standard_dev(2);
}
void NoiseParams::setLandmarkNoise(Eigen::Ref<const Eigen::Matrix3d> cov) {
  Ql_ = cov;
}

void NoiseParams::setContactNoise(double standard_dev) {
  Qc_ = standard_dev * standard_dev * Eigen::Matrix3d::Identity();
}
void NoiseParams::setContactNoise(
    Eigen::Ref<const Eigen::Vector3d> standard_dev) {
  Qc_ << standard_dev(0) * standard_dev(0), 0, 0, 0,
      standard_dev(1) * standard_dev(1), 0, 0, 0,
      standard_dev(2) * standard_dev(2);
}
void NoiseParams::setContactNoise(Eigen::Ref<const Eigen::Matrix3d> cov) {
  Qc_ = cov;
}

Eigen::Ref<const Eigen::Matrix3d> NoiseParams::getGyroscopeCov() const {
  return Qg_;
}
Eigen::Ref<const Eigen::Matrix3d> NoiseParams::getAccelerometerCov() const {
  return Qa_;
}
Eigen::Ref<const Eigen::Matrix3d> NoiseParams::getGyroscopeBiasCov() const {
  return Qbg_;
}
Eigen::Ref<const Eigen::Matrix3d> NoiseParams::getAccelerometerBiasCov() const {
  return Qba_;
}
Eigen::Ref<const Eigen::Matrix3d> NoiseParams::getLandmarkCov() const {
  return Ql_;
}
Eigen::Ref<const Eigen::Matrix3d> NoiseParams::getContactCov() const {
  return Qc_;
}

std::ostream &operator<<(std::ostream &os, const NoiseParams &p) {
  os << "--------- Noise Params -------------" << endl;
  os << "Gyroscope Covariance:\n" << p.Qg_ << endl;
  os << "Accelerometer Covariance:\n" << p.Qa_ << endl;
  os << "Gyroscope Bias Covariance:\n" << p.Qbg_ << endl;
  os << "Accelerometer Bias Covariance:\n" << p.Qba_ << endl;
  os << "Landmark Covariance:\n" << p.Ql_ << endl;
  os << "Contact Covariance:\n" << p.Qc_ << endl;
  os << "-----------------------------------" << endl;
  return os;
}

} // namespace inekf
