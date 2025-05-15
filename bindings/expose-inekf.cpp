#include "inekf/InEKF.hpp"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <eigenpy/eigen-from-python.hpp>
#include <eigenpy/eigen-to-python.hpp>
#include <eigenpy/std-map.hpp>
#include <eigenpy/std-pair.hpp>
#include <eigenpy/std-vector.hpp>

namespace inekf {
namespace python {
namespace bp = boost::python;
using eigenpy::StdVectorPythonVisitor;
using eigenpy::python::StdMapPythonVisitor;

void exposeInEKF() {
  eigenpy::StdPairConverter<std::pair<int, bool>>::registration();
  StdVectorPythonVisitor<std::vector<std::pair<int, bool>>, true>::expose(
      "StdVec_StdPair_intbool");

  StdVectorPythonVisitor<
      std::vector<Kinematics, Eigen::aligned_allocator<Kinematics>>,
      true>::expose("StdVec_Kinematics");

  StdMapPythonVisitor<int, int, std::less<int>,
                      std::allocator<std::pair<const int, int>>,
                      true>::expose("StdMap_int");

  StdMapPythonVisitor<int, bool, std::less<int>,
                      std::allocator<std::pair<const int, bool>>,
                      true>::expose("StdMap_bool");

  bp::class_<Kinematics>("Kinematics")
      .def(bp::init<>())
      .def(bp::init<int, const Eigen::Vector3d &, const Eigen::Matrix3d &>())
      .def(bp::init<int, const Eigen::Vector3d &, const Eigen::Matrix3d &,
                    const Eigen::Vector3d &, const Eigen::Matrix3d &>())
      .def_readwrite("id", &Kinematics::id)
      .def_readwrite("position", &Kinematics::position)
      .def_readwrite("velocity", &Kinematics::velocity)
      .def_readwrite("covariance", &Kinematics::covariance)
      .def_readwrite("covariance_vel", &Kinematics::covariance_vel);

  bp::class_<InEKF>("InEKF")
      .def(bp::init<>())
      .def(bp::init<const NoiseParams &>())
      .def(bp::init<const RobotState &>())
      .def(bp::init<const RobotState &, const NoiseParams &>())

      .def("getState", &InEKF::getState, bp::args("self"),
           bp::return_internal_reference<>())
      .def("getNoiseParams", &InEKF::getNoiseParams, bp::args("self"),
           bp::return_internal_reference<>())
      .def("getPriorLandmarks", &InEKF::getPriorLandmarks, bp::args("self"),
           bp::return_internal_reference<>())
      .def("getEstimatedLandmarks", &InEKF::getEstimatedLandmarks)
      .def("getContacts", &InEKF::getContacts)
      .def("getEstimatedContactPositions", &InEKF::getEstimatedContactPositions)

      .def("setState", &InEKF::setState)
      .def("setNoiseParams", &InEKF::setNoiseParams)
      .def("setPriorLandmarks", &InEKF::setPriorLandmarks)
      .def("setContacts", &InEKF::setContacts)
      .def("setGravity", &InEKF::setGravity)

      .def("propagate", &InEKF::propagate)
      .def("correct", &InEKF::correct)
      .def("correctLandmarks", &InEKF::correctLandmarks)
      .def("correctKinematics", &InEKF::correctKinematics);

  bp::class_<Observation>(
      "Observation", bp::init<const Eigen::VectorXd &, const Eigen::VectorXd &,
                              const Eigen::MatrixXd &, const Eigen::MatrixXd &,
                              const Eigen::MatrixXd &>())
      .def("empty", &Observation::empty)

      .def_readwrite("Y", &Observation::Y)
      .def_readwrite("b", &Observation::b)
      .def_readwrite("H", &Observation::H)
      .def_readwrite("N", &Observation::N)
      .def_readwrite("PI", &Observation::PI);

  bp::class_<Landmark>("Landmark", bp::init<int, const Eigen::Vector3d &>())
      .def_readwrite("id", &Landmark::id)
      .def_readwrite("position", &Landmark::position);
}

} // namespace python
} // namespace inekf
