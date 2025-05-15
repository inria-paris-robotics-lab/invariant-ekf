///////////////////////////////////////////////////////////////////////////////
// BSD 2-Clause License
//
// Copyright (C) 2025, INRIA
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include <eigenpy/eigen-from-python.hpp>
#include <eigenpy/eigen-to-python.hpp>

#include "inekf/LieGroup.hpp"

namespace inekf {
namespace python {
namespace bp = boost::python;

void exposeLieGroup() {
  bp::scope().attr("TOLERANCE") = inekf::TOLERANCE;

  bp::def("skew", &inekf::skew,
          "Compute the skew-symmetric matrix of a 3D vector");

  bp::def("exp_SO3", &inekf::exp_SO3, "Compute the exponential map of SO(3)");

  bp::def("exp_SEK3", &inekf::exp_SEK3,
          "Compute the exponential map of SEK(3)");

  bp::def("adjoint_SEK3", &inekf::adjoint_SEK3,
          "Compute the adjoint representation of SEK(3)");
}
} // namespace python
} // namespace inekf
