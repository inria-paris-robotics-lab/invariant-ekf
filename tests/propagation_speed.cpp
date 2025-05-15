/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   propagation_speed.cpp
 *  @author Ross Hartley
 *  @brief  Test to determine average propagation speed
 *  @date   September 25, 2018
 **/

#include <boost/test/unit_test.hpp>

#include "inekf/InEKF.hpp"
#include "inekf/utils.hpp"
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define DT_MIN 1e-6
#define DT_MAX 1
#define TOLERANCE 1e-6

BOOST_AUTO_TEST_SUITE(propagation_speed)

using namespace std;
using namespace inekf;
using namespace boost::posix_time;

BOOST_AUTO_TEST_CASE(propagation) {
  // Initialize filter
  Eigen::MatrixXd X = Eigen::MatrixXd::Identity(10, 10);
  RobotState state(X);
  InEKF filter(state);

  cout << "Robot's state is initialized to: \n";
  cout << filter.getState() << endl;

  ifstream infile("../data/propagation_speed_test_data.txt");
  string line;
  Eigen::Matrix<double, 6, 1> m, m_last;
  double t, t_last;
  m_last << 0, 0, 0, 0, 0, 0;
  t_last = 0;
  vectorPairIntVector6d measurements_vec;

  // Loop through data file and read in measurements line by line
  while (getline(infile, line)) {
    vector<string> measurement;
    boost::split(measurement, line, boost::is_any_of(" "));
    // Handle measurements
    if (measurement[0].compare("IMU") == 0) {
      t = stod98(measurement[1]);
      m << stod98(measurement[2]), stod98(measurement[3]),
          stod98(measurement[4]), stod98(measurement[5]),
          stod98(measurement[6]), stod98(measurement[7]);
      measurements_vec.push_back(
          pair<double, Eigen::Matrix<double, 6, 1>>(t, m));
    }
  }

  // Propagate all IMU data
  cout << "Propagating " << measurements_vec.size() << " IMU measurements...\n";
  int64_t max_duration = 0;
  int64_t sum_duration = 0;
  for (vectorPairIntVector6dIterator it = measurements_vec.begin();
       it != measurements_vec.end(); ++it) {
    // Propagate using IMU data
    t = it->first;
    m = it->second;

    double dt = t - t_last;
    if (dt > DT_MIN && dt < DT_MAX) {
      ptime start_time = second_clock::local_time();
      filter.propagate(m_last, dt);
      ptime end_time = second_clock::local_time();
      int64_t duration = (end_time - start_time).total_nanoseconds();
      // cout << "duration: " <<  duration << endl;
      sum_duration += duration;
      if (duration > max_duration)
        max_duration = duration;

      // cout << filter.getState() << endl;
    }
    // Store previous timestamp
    t_last = t;
    m_last = m;
  }
  cout << "final state " << filter.getState() << endl;
  cout << "max duration: " << max_duration << endl;
  cout << "average duration: "
       << double(sum_duration) / (double)measurements_vec.size() << endl;

  Eigen::MatrixXd X_ref(10, 10);
  X_ref.setIdentity();
  X_ref.topLeftCorner(3, 5) << 0.999956, -0.00543054, -0.00764294, 0.00700324,
      0.00289025, 0.00539822, 0.999976, -0.00424347, -0.0118462, -0.00696191,
      0.00766581, 0.00420202, 0.999962, -9.82926, -4.91541;

  BOOST_CHECK(X_ref.isApprox(filter.getState().getX(), TOLERANCE));
}

BOOST_AUTO_TEST_SUITE_END()
