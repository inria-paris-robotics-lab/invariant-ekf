/// @copyright Copyright (C) 2025 INRIA
#include "inekf/python.hpp"

namespace inekf::python {

/* FORWARD DECLARATIONS */
void exposeLieGroup();
void exposeNoiseParams();
void exposeRobotState();
void exposeInEKF();

/* PYTHON MODULE */
BOOST_PYTHON_MODULE(inekf_pywrap) {
  namespace bp = boost::python;
  bp::import("eigenpy");
  ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::MatrixXd);

  exposeLieGroup();
  exposeNoiseParams();
  exposeRobotState();
  exposeInEKF();
}

} // namespace inekf::python
