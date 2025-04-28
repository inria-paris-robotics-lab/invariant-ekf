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

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define DT_MIN 1e-6
#define DT_MAX 1
#define TOLERANCE 1e-6

BOOST_AUTO_TEST_SUITE(kinematics)

using namespace std;
using namespace inekf;

BOOST_AUTO_TEST_CASE(kinematics) {
  //  ---- Initialize invariant extended Kalman filter ----- //
  RobotState initial_state;

  // Initialize state mean
  Eigen::Matrix3d R0;
  Eigen::Vector3d v0, p0, bg0, ba0;
  R0 << 1, 0, 0, // initial orientation
      0, -1, 0,  // IMU frame is rotated 90deg about the x-axis
      0, 0, -1;
  v0 << 0, 0, 0;  // initial velocity
  p0 << 0, 0, 0;  // initial position
  bg0 << 0, 0, 0; // initial gyroscope bias
  ba0 << 0, 0, 0; // initial accelerometer bias
  initial_state.setRotation(R0);
  initial_state.setVelocity(v0);
  initial_state.setPosition(p0);
  initial_state.setGyroscopeBias(bg0);
  initial_state.setAccelerometerBias(ba0);

  // Initialize state covariance
  NoiseParams noise_params;
  noise_params.setGyroscopeNoise(0.01);
  noise_params.setAccelerometerNoise(0.1);
  noise_params.setGyroscopeBiasNoise(0.00001);
  noise_params.setAccelerometerBiasNoise(0.0001);
  noise_params.setContactNoise(0.01);

  // Initialize filter
  InEKF filter(initial_state, noise_params);
  cout << "Noise parameters are initialized to: \n";
  cout << filter.getNoiseParams() << endl;
  cout << "Robot's state is initialized to: \n";
  cout << filter.getState() << endl;

  // Open data file
  ifstream infile("../data/imu_kinematic_measurements.txt");
  string line;
  Eigen::Matrix<double, 6, 1> imu_measurement =
      Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> imu_measurement_prev =
      Eigen::Matrix<double, 6, 1>::Zero();
  double t = 0;
  double t_prev = 0;

  long nb_measures_kin = 0;
  long sum_kinematics = 0;
  long nb_measures_imu = 0;
  long sum_imu = 0;

  // ---- Loop through data file and read in measurements line by line ---- //
  while (getline(infile, line)) {
    vector<string> measurement;
    boost::split(measurement, line, boost::is_any_of(" "));
    // // Handle measurements
    if (measurement[0].compare("IMU") == 0) {
      // cout << "Received IMU Data, propagating state\n";
      assert((measurement.size() - 2) == 6);
      t = stod98(measurement[1]);
      // Read in IMU data
      imu_measurement << stod98(measurement[2]), stod98(measurement[3]),
          stod98(measurement[4]), stod98(measurement[5]),
          stod98(measurement[6]), stod98(measurement[7]);

      // Propagate using IMU data
      double dt = t - t_prev;
      std::chrono::steady_clock::time_point begin2;
      std::chrono::steady_clock::time_point end2;
      if (dt > DT_MIN && dt < DT_MAX) {
        begin2 = std::chrono::steady_clock::now();
        filter.propagate(imu_measurement_prev, dt);
        end2 = std::chrono::steady_clock::now();
      }
      nb_measures_imu += 1;
      sum_imu +=
          std::chrono::duration_cast<std::chrono::microseconds>(end2 - begin2)
              .count();

    } else if (measurement[0].compare("CONTACT") == 0) {
      // cout << "Received CONTACT Data, setting filter's contact state\n";
      assert((measurement.size() - 2) % 2 == 0);
      vector<pair<int, bool>> contacts;
      int id;
      bool indicator;
      t = stod98(measurement[1]);
      // Read in contact data
      for (size_t i = 2; i < measurement.size(); i += 2) {
        id = stoi98(measurement[i]);
        indicator = bool(stod98(measurement[i + 1]));
        contacts.push_back(pair<int, bool>(id, indicator));
      }
      // Set filter's contact state
      filter.setContacts(contacts);
    } else if (measurement[0].compare("KINEMATIC") == 0) {
      // cout << "Received KINEMATIC observation, correcting state\n";
      assert((measurement.size() - 2) % 44 == 0);
      int id;
      Eigen::Quaternion<double> q;
      Eigen::Vector3d position;
      Eigen::Matrix3d covariance;
      vectorKinematics measured_kinematics;
      t = stod98(measurement[1]);
      // Read in kinematic data
      for (size_t i = 2; i < measurement.size(); i += 44) {
        id = stoi98(measurement[i]);
        q = Eigen::Quaternion<double>(
            stod98(measurement[i + 1]), stod98(measurement[i + 2]),
            stod98(measurement[i + 3]), stod98(measurement[i + 4]));
        q.normalize();
        position << stod98(measurement[i + 5]), stod98(measurement[i + 6]),
            stod98(measurement[i + 7]);
        for (size_t j = 3; j < 6; ++j) {
          for (size_t k = 3; k < 6; ++k) {
            covariance((long)(j - 3), (long)(k - 3)) =
                stod98(measurement[i + 8 + j * 6 + k]);
          }
        }

        Kinematics frame(id, position, covariance);
        measured_kinematics.push_back(frame);
      }
      // Correct state using kinematic measurements
      std::chrono::steady_clock::time_point begin =
          std::chrono::steady_clock::now();
      filter.correctKinematics(measured_kinematics);
      std::chrono::steady_clock::time_point end =
          std::chrono::steady_clock::now();
      sum_kinematics +=
          std::chrono::duration_cast<std::chrono::microseconds>(end - begin)
              .count();
      nb_measures_kin += 1;
    }

    // Store previous timestamp
    t_prev = t;
    imu_measurement_prev = imu_measurement;
  }
  double mean_kinematics_time =
      (double)sum_kinematics / (double)nb_measures_kin;
  double mean_imu_time = (double)sum_imu / (double)nb_measures_imu;

  // Final state should be
  Eigen::MatrixXd Xref(6, 6);
  Xref << 0.996183, 0.0828516, -0.0274863, 0.148885, 2.39881, 2.37682,
      0.0851084, -0.991845, 0.094868, 0.012167, 0.0157504, 0.135066, -0.0194022,
      -0.0968452, -0.99511, 0.0350042, -0.111789, -0.910618, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;

  // Print final state
  cout << "final state " << filter.getState() << endl;
  BOOST_CHECK(Xref.isApprox(filter.getState().getX(), TOLERANCE));
  std::cout << "Mean correct time is " << mean_kinematics_time << std::endl;
  std::cout << "Mean propagate time is " << mean_imu_time << std::endl;
}

BOOST_AUTO_TEST_SUITE_END()
