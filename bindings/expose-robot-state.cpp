#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include "inekf/RobotState.hpp"

namespace inekf
{
  namespace python
  {
    namespace bp = boost::python;

    void exposeRobotState()
    {
      bp::class_<RobotState>("RobotState")
        .def(bp::init<>())
        .def(bp::init<const Eigen::MatrixXd&>())
        .def(bp::init<const Eigen::MatrixXd&, const Eigen::VectorXd&>())
        .def(bp::init<const Eigen::MatrixXd&, const Eigen::VectorXd&, const Eigen::MatrixXd&>())

        .def("getX", &RobotState::getX)
        .def("getTheta", &RobotState::getTheta)
        .def("getP", &RobotState::getP)
        .def("getRotation", &RobotState::getRotation)
        .def("getVelocity", &RobotState::getVelocity)
        .def("getPosition", &RobotState::getPosition)
        .def("getGyroscopeBias", &RobotState::getGyroscopeBias)
        .def("getAccelerometerBias", &RobotState::getAccelerometerBias)
        .def("dimX", &RobotState::dimX)
        .def("dimTheta", &RobotState::dimTheta)
        .def("dimP", &RobotState::dimP)
        
        .def("setX", &RobotState::setX)
        .def("setP", &RobotState::setP)
        .def("setTheta", &RobotState::setTheta)
        .def("setRotation", &RobotState::setRotation)
        .def("setVelocity", &RobotState::setVelocity)
        .def("setPosition", &RobotState::setPosition)
        .def("setGyroscopeBias", &RobotState::setGyroscopeBias)
        .def("setAccelerometerBias", &RobotState::setAccelerometerBias)
        
        .def("copyDiagX", &RobotState::copyDiagX);
    }
  } // namespace python
} // namespace simple_mpc