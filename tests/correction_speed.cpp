/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   correction_speed.cpp
 *  @author Ross Hartley
 *  @brief  Test to determine average correction speed
 *  @date   September 25, 2018
 **/

#include <boost/test/unit_test.hpp>

#include "inekf/InEKF.hpp"
#include "inekf/utils.hpp"
#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#define DT_MIN 1e-6
#define DT_MAX 1
#define TOLERANCE 1e-6

BOOST_AUTO_TEST_SUITE(correction_speed)

using namespace std;
using namespace inekf;
using namespace boost::posix_time;

BOOST_AUTO_TEST_CASE(correction) {
  // Initialize filter
  InEKF filter;
  cout << "Robot's state is initialized to: \n";
  cout << filter.getState() << endl;

  ifstream infile("../data/correction_speed_test_data.txt");
  string line;
  Eigen::Matrix<double, 6, 1> m, m_last;
  double t, t_last;
  m_last << 0, 0, 0, 0, 0, 0;
  t_last = 0;
  vectorPairIntVector6d measurements_vec;
  vectorLandmarks measured_landmarks;

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

    } else if (measurement[0].compare("LANDMARK") == 0) {
      t = stod98(measurement[1]);
      for (size_t i = 2; i < measurement.size(); i += 4) {
        int id = (int)stod98(measurement[i]);
        Eigen::Vector3d p_bl;
        p_bl << stod98(measurement[i + 1]), stod98(measurement[i + 2]),
            stod98(measurement[i + 3]);
        Landmark landmark(id, p_bl);
        measured_landmarks.push_back(landmark);
      }
    }
  }

  // Propagate all IMU data
  cout << "Propagating " << measurements_vec.size() << " IMU measurements...\n";
  for (vectorPairIntVector6dIterator it = measurements_vec.begin();
       it != measurements_vec.end(); ++it) {
    // Propagate using IMU data
    t = it->first;
    m = it->second;
    double dt = t - t_last;
    if (dt > DT_MIN && dt < DT_MAX) {
      filter.propagate(m_last, dt);
    }
    // Store previous timestamp
    t_last = t;
    m_last = m;
  }
  cout << filter.getState() << endl;

  Eigen::MatrixXd X_ref(5, 5);
  X_ref << 0.989023, -0.00271245, -0.147738, -0.0804268, -0.0395996, 0.0229655,
      0.990504, 0.135555, -0.0776685, -0.0591434, 0.145967, -0.13746, 0.979693,
      -9.85893, -4.92681, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;

  BOOST_CHECK(X_ref.isApprox(filter.getState().getX(), TOLERANCE));

  // Correct using landmark data
  cout << "Correcting " << measured_landmarks.size()
       << " landmark measurements...\n";
  int64_t max_duration = 0;
  int64_t sum_duration = 0;
  for (vectorLandmarksIterator it = measured_landmarks.begin();
       it != measured_landmarks.end(); ++it) {
    vectorLandmarks landmarks;
    landmarks.push_back(*it);
    ptime start_time = second_clock::local_time();
    filter.correctLandmarks(landmarks);
    // cout << filter.getState() << endl;
    ptime end_time = second_clock::local_time();
    int64_t duration = (end_time - start_time).total_nanoseconds();
    // cout << "duration: " <<  duration << endl;
    sum_duration += duration;
    if (duration > max_duration)
      max_duration = duration;
  }
  cout << filter.getState() << endl;
  cout << "max duration: " << max_duration << endl;
  cout << "average duration: "
       << double(sum_duration) / ((double)measured_landmarks.size() / 3)
       << endl;

  Eigen::MatrixXd X_ref2(15, 15);
  X_ref2.setIdentity();
  X_ref2.topRows(3) << 0.989023, -0.00271245, -0.147738, -0.0804269, -0.0395997,
      -0.0443532, -0.039004, -0.0390407, -0.0448281, -0.0340716, -0.0290572,
      -0.0430822, -0.0344641, -0.0506805, -0.0378577, 0.0229655, 0.990504,
      0.135555, -0.0776685, -0.0591434, -0.0684089, -0.0722664, -0.038285,
      -0.0392135, -0.0597898, -0.0518103, -0.0549081, -0.0510135, -0.0560486,
      -0.0609962, 0.145967, -0.13746, 0.979693, -9.85893, -4.92681, -4.91634,
      -4.94546, -4.90262, -4.92101, -4.91384, -4.91953, -4.91349, -4.94641,
      -4.92917, -4.93037;

  BOOST_CHECK(X_ref2.isApprox(filter.getState().getX(), TOLERANCE));
}

BOOST_AUTO_TEST_SUITE_END()
