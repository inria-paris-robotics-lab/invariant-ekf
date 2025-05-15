#pragma once

#include <eigenpy/eigenpy.hpp>

namespace inekf {
/// \brief Python bindings
namespace python {

namespace bp = boost::python;

// User-defined literal to avoid writing bp::arg, bp::args, etc.
inline bp::arg operator""_a(const char *name, size_t) { return bp::arg(name); }

} // namespace python
} // namespace inekf
