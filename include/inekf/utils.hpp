#include <Eigen/Core>

namespace inekf
{
  typedef std::vector<std::pair<double, Eigen::Matrix<double, 6, 1>>> vectorPairIntVector6d;
  typedef std::vector<std::pair<double, Eigen::Matrix<double, 6, 1>>>::const_iterator
      vectorPairIntVector6dIterator;
      
  inline double stod98(const std::string &s) { return atof(s.c_str()); }

  inline int stoi98(const std::string &s) { return atoi(s.c_str()); }
} // namespace inekf
