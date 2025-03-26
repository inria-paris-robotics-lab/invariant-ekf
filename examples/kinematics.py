from inekf import (
    RobotState,
    NoiseParams,
    InEKF,
    Kinematics
)
import numpy as np

# Initialize state
initial_state = RobotState()

R0 = np.array([[1, 0, 0],
               [0, -1, 0],
               [0, 0, -1]]) # Initial rotation 

v0 = np.zeros(3) # initial velocity
p0 = np.zeros(3) # initial position
bg0 = np.zeros(3) # initial gyroscope bias
ba0 = np.zeros(3) # initial accelerometer bias
g = np.array([0, 0, 9])

initial_state.setRotation(R0)
initial_state.setVelocity(v0)
initial_state.setPosition(p0)
initial_state.setGyroscopeBias(bg0)
initial_state.setAccelerometerBias(ba0)

# Initialize state covariance
noise_params = NoiseParams()
noise_params.setGyroscopeNoise(0.01)
noise_params.setAccelerometerNoise(0.1)
noise_params.setGyroscopeBiasNoise(0.00001)
noise_params.setAccelerometerBiasNoise(0.0001)
noise_params.setContactNoise(0.01)

# Initialize filter
filter = InEKF(initial_state, noise_params)
filter.setGravity(g)
print("Noise parameters ", filter.getNoiseParams())
print("Initial state", filter.getState())

foot_frame_name = [prefix + "_foot" for prefix in ["FL", "FR", "RL", "RR"]]
contact_list = []
kinematics_list = []
for i in range(4):
    contact_list.append((i, True))
    pose_matrix = np.eye(4)
    covariance = np.zeros((6,6))

    kinematics = Kinematics()
    kinematics.id = i
    kinematics.pose = pose_matrix
    kinematics.covariance = covariance

    kinematics_list.append(kinematics)


filter.setContacts(contact_list)
filter.correctKinematics(kinematics_list)