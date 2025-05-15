/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   NoiseParams.hpp
 *  @author Ross Hartley
 *  @brief  Header file for Invariant EKF noise parameter class
 *  @date   September 25, 2018
 **/
#ifndef NOISEPARAMS_H
#define NOISEPARAMS_H
#include <Eigen/Dense>
#include <iostream>

namespace inekf {

class NoiseParams {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NoiseParams();

  void setGyroscopeNoise(double standard_dev);
  void setGyroscopeNoise(Eigen::Ref<const Eigen::Vector3d> standard_dev);
  void setGyroscopeNoise(Eigen::Ref<const Eigen::Matrix3d> cov);

  void setAccelerometerNoise(double standard_dev);
  void setAccelerometerNoise(Eigen::Ref<const Eigen::Vector3d> standard_dev);
  void setAccelerometerNoise(Eigen::Ref<const Eigen::Matrix3d> cov);

  void setGyroscopeBiasNoise(double standard_dev);
  void setGyroscopeBiasNoise(Eigen::Ref<const Eigen::Vector3d> standard_dev);
  void setGyroscopeBiasNoise(Eigen::Ref<const Eigen::Matrix3d> cov);

  void setAccelerometerBiasNoise(double standard_dev);
  void
  setAccelerometerBiasNoise(Eigen::Ref<const Eigen::Vector3d> standard_dev);
  void setAccelerometerBiasNoise(Eigen::Ref<const Eigen::Matrix3d> cov);

  void setLandmarkNoise(double standard_dev);
  void setLandmarkNoise(Eigen::Ref<const Eigen::Vector3d> standard_dev);
  void setLandmarkNoise(Eigen::Ref<const Eigen::Matrix3d> cov);

  void setContactNoise(double standard_dev);
  void setContactNoise(Eigen::Ref<const Eigen::Vector3d> standard_dev);
  void setContactNoise(Eigen::Ref<const Eigen::Matrix3d> cov);

  Eigen::Ref<const Eigen::Matrix3d> getGyroscopeCov() const;
  Eigen::Ref<const Eigen::Matrix3d> getAccelerometerCov() const;
  Eigen::Ref<const Eigen::Matrix3d> getGyroscopeBiasCov() const;
  Eigen::Ref<const Eigen::Matrix3d> getAccelerometerBiasCov() const;
  Eigen::Ref<const Eigen::Matrix3d> getLandmarkCov() const;
  Eigen::Ref<const Eigen::Matrix3d> getContactCov() const;

  friend std::ostream &operator<<(std::ostream &os, const NoiseParams &p);

private:
  Eigen::Matrix3d Qg_;
  Eigen::Matrix3d Qa_;
  Eigen::Matrix3d Qbg_;
  Eigen::Matrix3d Qba_;
  Eigen::Matrix3d Ql_;
  Eigen::Matrix3d Qc_;
};

} // namespace inekf
#endif
