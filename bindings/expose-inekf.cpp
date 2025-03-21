#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <eigenpy/std-vector.hpp>
#include "inekf/InEKF.hpp"

namespace inekf
{
  namespace python
  {
    namespace bp = boost::python;
    using eigenpy::StdVectorPythonVisitor;

    void exposeInEKF()
    {
      bp::class_<InEKF>("InEKF")
        .def(bp::init<>())
        .def(bp::init<NoiseParams>())
        .def(bp::init<RobotState>())
        .def(bp::init<RobotState, inekf::NoiseParams>())
        
        .def("getState", &InEKF::getState)
        .def("getNoiseParams", &InEKF::getNoiseParams)
        .def("getPriorLandmarks", &InEKF::getPriorLandmarks)
        .def("getEstimatedLandmarks", &InEKF::getEstimatedLandmarks)
        .def("getContacts", &InEKF::getContacts)
        .def("getEstimatedContactPositions", &InEKF::getEstimatedContactPositions)
        
        .def("setState", &InEKF::setState)
        .def("setNoiseParams", &InEKF::setNoiseParams)
        .def("setPriorLandmarks", &InEKF::setPriorLandmarks)
        .def("setContacts", &InEKF::setContacts)
        
        .def("Propagate", &InEKF::Propagate)
        .def("Correct", &InEKF::Correct)
        .def("CorrectLandmarks", &InEKF::CorrectLandmarks)
        .def("CorrectKinematics", &InEKF::CorrectKinematics);
    
      bp::class_<Observation>("Observation", 
         bp::init<Eigen::VectorXd&, Eigen::VectorXd&, Eigen::MatrixXd&, Eigen::MatrixXd&, Eigen::MatrixXd&>())
        .def("empty", &Observation::empty)
        
        .def_readwrite("Y", &Observation::Y)
        .def_readwrite("b", &Observation::b)
        .def_readwrite("H", &Observation::H)
        .def_readwrite("N", &Observation::N)
        .def_readwrite("PI", &Observation::PI);
      
      bp::class_<Kinematics>("Kinematics", bp::init<int, Eigen::Matrix4d, Eigen::Matrix<double,6,6>>())
        .def_readwrite("id", &Kinematics::id)
        .def_readwrite("pose", &Kinematics::pose)
        .def_readwrite("covariance", &Kinematics::covariance);
      
      using KinematicsVec = std::vector<Kinematics>;
      StdVectorPythonVisitor<KinematicsVec, true>::expose("StdVec_Kinematics", 
        eigenpy::details::overload_base_get_item_for_std_vector<KinematicsVec>());
    
      bp::class_<Landmark>("Landmark", bp::init<int, Eigen::Vector3d>())
        .def_readwrite("id", &Landmark::id)
        .def_readwrite("position", &Landmark::position);
    }

  } // namespace python
} // namespace simple_mpc