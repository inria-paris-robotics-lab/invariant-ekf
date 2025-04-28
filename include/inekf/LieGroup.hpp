/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   LieGroup.hpp
 *  @author Ross Hartley
 *  @brief  Header file for various Lie Group functions
 *  @date   September 25, 2018
 **/

#ifndef LIEGROUP_H
#define LIEGROUP_H
#include <Eigen/Dense>

namespace inekf {

extern const double TOLERANCE;

Eigen::Matrix3d skew(Eigen::Ref<const Eigen::Vector3d> v);
Eigen::Matrix3d exp_SO3(Eigen::Ref<const Eigen::Vector3d> w);
Eigen::MatrixXd exp_SEK3(Eigen::Ref<const Eigen::VectorXd> v);
Eigen::MatrixXd adjoint_SEK3(Eigen::Ref<const Eigen::MatrixXd> X);

} // namespace inekf
#endif
