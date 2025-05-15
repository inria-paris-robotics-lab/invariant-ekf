#include "inekf/NoiseParams.hpp"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <eigenpy/eigen-from-python.hpp>
#include <eigenpy/eigen-to-python.hpp>

namespace inekf {
namespace python {
namespace bp = boost::python;

void exposeNoiseParams() {
  bp::class_<NoiseParams>("NoiseParams", bp::init<>())
      .def("setGyroscopeNoise",
           (void(NoiseParams::*)(double)) & NoiseParams::setGyroscopeNoise)
      .def("setGyroscopeNoise",
           (void(NoiseParams::*)(Eigen::Ref<const Eigen::Vector3d>)) &
               NoiseParams::setGyroscopeNoise)
      .def("setGyroscopeNoise",
           (void(NoiseParams::*)(Eigen::Ref<const Eigen::Matrix3d>)) &
               NoiseParams::setGyroscopeNoise)

      .def("setGyroscopeBiasNoise",
           (void(NoiseParams::*)(double)) & NoiseParams::setGyroscopeBiasNoise)
      .def("setGyroscopeBiasNoise",
           (void(NoiseParams::*)(Eigen::Ref<const Eigen::Vector3d>)) &
               NoiseParams::setGyroscopeBiasNoise)
      .def("setGyroscopeBiasNoise",
           (void(NoiseParams::*)(Eigen::Ref<const Eigen::Matrix3d>)) &
               NoiseParams::setGyroscopeBiasNoise)

      .def("setAccelerometerNoise",
           (void(NoiseParams::*)(double)) & NoiseParams::setAccelerometerNoise)
      .def("setAccelerometerNoise",
           (void(NoiseParams::*)(Eigen::Ref<const Eigen::Vector3d>)) &
               NoiseParams::setAccelerometerNoise)
      .def("setAccelerometerNoise",
           (void(NoiseParams::*)(Eigen::Ref<const Eigen::Matrix3d>)) &
               NoiseParams::setAccelerometerNoise)

      .def("setAccelerometerBiasNoise",
           (void(NoiseParams::*)(double)) &
               NoiseParams::setAccelerometerBiasNoise)
      .def("setAccelerometerBiasNoise",
           (void(NoiseParams::*)(Eigen::Ref<const Eigen::Vector3d>)) &
               NoiseParams::setAccelerometerBiasNoise)
      .def("setAccelerometerBiasNoise",
           (void(NoiseParams::*)(Eigen::Ref<const Eigen::Matrix3d>)) &
               NoiseParams::setAccelerometerBiasNoise)

      .def("setLandmarkNoise",
           (void(NoiseParams::*)(double)) & NoiseParams::setLandmarkNoise)
      .def("setLandmarkNoise",
           (void(NoiseParams::*)(Eigen::Ref<const Eigen::Vector3d>)) &
               NoiseParams::setLandmarkNoise)
      .def("setLandmarkNoise",
           (void(NoiseParams::*)(Eigen::Ref<const Eigen::Matrix3d>)) &
               NoiseParams::setLandmarkNoise)

      .def("setContactNoise",
           (void(NoiseParams::*)(double)) & NoiseParams::setContactNoise)
      .def("setContactNoise",
           (void(NoiseParams::*)(Eigen::Ref<const Eigen::Vector3d>)) &
               NoiseParams::setContactNoise)
      .def("setContactNoise",
           (void(NoiseParams::*)(Eigen::Ref<const Eigen::Matrix3d>)) &
               NoiseParams::setContactNoise)

      .def("getGyroscopeCov", &NoiseParams::getGyroscopeCov)
      .def("getAccelerometerCov", &NoiseParams::getAccelerometerCov)
      .def("getGyroscopeBiasCov", &NoiseParams::getGyroscopeBiasCov)
      .def("getAccelerometerBiasCov", &NoiseParams::getAccelerometerBiasCov)
      .def("getLandmarkCov", &NoiseParams::getLandmarkCov)
      .def("getContactCov", &NoiseParams::getContactCov);
}
} // namespace python
} // namespace inekf
