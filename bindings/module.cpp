/// @copyright Copyright (C) 2025 INRIA
#include "inekf/python.hpp"

namespace inekf::python
{

  namespace bp = boost::python;

  /* FORWARD DECLARATIONS */
  void exposeLieGroup();

  /* PYTHON MODULE */
  BOOST_PYTHON_MODULE(inekf_pywrap)
  {
    bp::scope().attr("__version__") = INEKF_VERSION;
    exposeLieGroup();
  }

} // namespace inekf::python
