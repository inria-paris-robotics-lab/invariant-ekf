/// @copyright Copyright (C) 2025 INRIA
#include "inekf/python.hpp"

namespace inekf::python
{

  namespace bp = boost::python;

  /* FORWARD DECLARATIONS */
  void exposeLieGroup();
  void exposeNoiseParams();
  void exposeRobotState();
  void exposeInEKF();

  /* PYTHON MODULE */
  BOOST_PYTHON_MODULE(inekf_pywrap)
  {
    exposeLieGroup();
    exposeNoiseParams();
    exposeRobotState();
    exposeInEKF();
  }

} // namespace inekf::python
