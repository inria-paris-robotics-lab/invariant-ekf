from inekf import (
    RobotState,
    NoiseParams,
    InEKF,
    Kinematics
)
import numpy as np

# Create state object
initial_state = RobotState()
dt = 1e-3 # Integration step
foot_frame_name = [prefix + "_foot" for prefix in ["FL", "FR", "RL", "RR"]]

# Initial rotation
R0 = np.array([[1, 0, 0],
               [0, -1, 0],
               [0, 0, -1]]) 

v0 = np.zeros(3) # initial velocity
p0 = np.zeros(3) # initial position
bg0 = np.zeros(3) # initial gyroscope bias
ba0 = np.zeros(3) # initial accelerometer bias
g = np.array([0, 0, 9]) # Gravity

# Initialize state
initial_state.setRotation(R0)
initial_state.setVelocity(v0)
initial_state.setPosition(p0)
initial_state.setGyroscopeBias(bg0)
initial_state.setAccelerometerBias(ba0)

# Initialize covariance noise in propagation step of IEKF
noise_params = NoiseParams()
noise_params.setGyroscopeNoise(0.01) # Noise added to gyroscope measure (in rad.s-1)
noise_params.setAccelerometerNoise(0.1) # Noise added to accelerometer measure (in m.s-2)
noise_params.setGyroscopeBiasNoise(0.00001) # Noise added to gyroscope bias (in rad.s-1)
noise_params.setAccelerometerBiasNoise(0.0001)  # Noise added to accelerometer bias (in m.s-2)
noise_params.setContactNoise(0.01)  # Noise added to contact velocity (assumed to be 0) (in m.s-1)

# Initialize filter
filter = InEKF(initial_state, noise_params)
filter.setGravity(g)
print("Noise parameters ", filter.getNoiseParams())
print("Initial state", filter.getState())

# Create measures for contacts and IMU
imu_measure = np.zeros(6)
contact_list = []
kinematics_list = []
for i in range(4):
    # True/False if contact numbered i is on/off
    contact_list.append((i, True))
    
    # Contact translation in base frame
    translation = np.zeros(3)
    covariance = np.eye(3) * 0.0001

    # Contact velocity in base frame
    velocity = np.zeros(3)
    covariance_vel = np.eye(3) * 0.0001

    kinematics = Kinematics(i, translation, covariance, velocity, covariance_vel)
    kinematics_list.append(kinematics)

# Propagation step
filter.propagate(imu_measure, dt)

# Set active contacts 
filter.setContacts(contact_list)

# Correction step through kinematics measures
filter.correctKinematics(kinematics_list)
