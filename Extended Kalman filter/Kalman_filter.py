# Main function of the Python program.
#
##

import pandas as pd
import numpy as np



# Fill in Those functions:

def computeRmse(trueVector, estimateVector):
    # Compute Root Mean Squared Error (RMSE)
    # Compute Root Mean Squared Error (RMSE)
    sum_squared_error = 0.0
    num_samples = len(trueVector)

    for i in range(num_samples):
        error = trueVector[i] - estimateVector[i]
        sum_squared_error += error ** 2

    rmse = np.sqrt(sum_squared_error / num_samples)
    return rmse

def computeRadarJacobian(Xvector):
    # Extract values from the state vector
    px = Xvector[0]
    py = Xvector[1]

    # Compute intermediate values for Jacobian matrix
    rho = np.sqrt(px ** 2 + py ** 2)
    phi = np.arctan2(py, px)

    # Check for division by zero
    if rho < 0.0001:
        rho = 0.0001

    # Compute the Jacobian matrix
    Hj = np.array([[px / rho, py / rho, 0, 0],
                   [-py / (rho ** 2), px / (rho ** 2), 0, 0],
                   [py * (Xvector[2] * py - Xvector[3] * px) / (rho ** 2),
                    px * (Xvector[3] * px - Xvector[2] * py) / (rho ** 2), px / rho, py / rho]])

    return Hj

def computeCovMatrix(deltaT, sigma_aX, sigma_aY):
    cov_matrix = np.array([[sigma_aX ** 2 * deltaT ** 4 / 4, 0, sigma_aX ** 2 * deltaT ** 3 / 3, 0],
                           [0, sigma_aY ** 2 * deltaT ** 4 / 4, 0, sigma_aY ** 2 * deltaT ** 3 / 3],
                           [sigma_aX ** 2 * deltaT ** 3 / 3, 0, sigma_aX ** 2 * deltaT ** 2, 0],
                           [0, sigma_aY ** 2 * deltaT ** 3 / 3, 0, sigma_aY ** 2 * deltaT ** 2]])

    return cov_matrix

def computeFmatrix(deltaT):
    # Compute the state transition matrix F
    f_matrix = np.array([[1, 0, deltaT, 0],
                         [0, 1, 0, deltaT],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    return f_matrix

def cartesian_to_polar(x, y, vx, vy):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)  # Use arctan2 to handle all possible cases of x and y
    rhodot = (x * vx + y * vy) / np.sqrt(x**2 + y**2)
    return rho, phi, rhodot

def radar_to_polar(radar_vector):
    x_groundtruth = radar_vector[5]
    y_groundtruth = radar_vector[6]
    vx_groundtruth = radar_vector[7]
    vy_groundtruth = radar_vector[8]

    rho, phi, rhodot = cartesian_to_polar(x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth)

    return rho, phi, rhodot


def radar_to_cartesian(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return x, y
def main():
    # we print a heading and make it bigger using HTML formatting
    print("Hellow")
    my_cols = ["A", "B", "C", "D", "E", "f", "g", "h", "i", "j", "k"]
    data = pd.read_csv(r"C:\Users\Dell\PycharmProjects\KalmanFilter\Data.txt", names=my_cols, delim_whitespace=True, header=None)
    print(data.head())
    for i in range(10):
        measur = data.iloc[i, :].values
        print(measur[0])
    # define matrices:
    deltaT = 0.1
    useRadar = True
    p = np.eye(2)
    p_radar = np.eye(4)
    xEstimate = []  # Estimated state vector
    xTrue = [] # True state vector
    H_Lidar = np.array([[1, 0],
                        [0, 1]])
    R_lidar = np.array([[0.0225, 0.0], [0.0, 0.0225]])

    R_radar = np.array([[0.9, 0, 0], [0.0, 0.0009, 0], [0, 0, 0.09]])
    I = np.eye(2)
    I_radar = np.eye(4)
    F_matrix = computeFmatrix(deltaT)
    currentMeas = data.iloc[0, :].values
    X_state_current = np.array([currentMeas[4],currentMeas[5]])

    X_true_current = np.array([currentMeas[4],currentMeas[5]])
    firstMeasurment = data.iloc[0, :].values
    timeStamp = firstMeasurment[3]
    # fill in X_true and X_state. Put 0 for the velocities
    for i in range(1, 10):
        currentMeas = data.iloc[i, :].values
        X_true_current = np.array([currentMeas[4],currentMeas[5]])

        # compute the current dela t
        if (currentMeas[0] == 'L'):
            deltaT = (currentMeas[3] - timeStamp) / 1000000
            timeStamp = currentMeas[3]
            F = np.array([[1, deltaT],
                                [0, 1]])
            # perfrom predict
            X_state_current = np.dot(F, X_state_current)
            p = np.dot(np.dot(F, p), F.T)

            # pefrom measurment update
            currentMeas = data.iloc[i+2, :].values
            z = np.array([currentMeas[1], currentMeas[2]])
            y = z - np.dot(H_Lidar, X_state_current)
            S = np.dot(np.dot(H_Lidar, p), H_Lidar.T) + R_lidar
            K = np.dot(np.dot(p, H_Lidar.T), np.linalg.inv(S))
            X_state_current = X_state_current + np.dot(K, y)
            p = np.dot((I - np.dot(K, H_Lidar)), p)

        if (currentMeas[0] == 'R' and useRadar):

            # Convert radar vector to polar coordinates
            rho, phi, rhodot = currentMeas[1], currentMeas[2], currentMeas[3]

            X_vector_input_jacobian = np.array([currentMeas[5], currentMeas[6],currentMeas[7], currentMeas[8]])


            # perform predict
            deltaT = (currentMeas[4] - timeStamp) / 1000000
            F_matrix = computeFmatrix(deltaT)
            timeStamp = currentMeas[4]
            X_state_current = np.array([rho,phi,rhodot])
            p_radar = np.dot(np.dot(F_matrix, p_radar), F_matrix.T)

            # perform measurement update
            jacobian = computeRadarJacobian(X_vector_input_jacobian)
            currentMeas2 = data.iloc[i + 2, :].values
            z = np.array([currentMeas2[5], currentMeas2[6],currentMeas2[7], currentMeas2[8]])
            S = np.dot(np.dot(jacobian, p_radar), jacobian.T) + R_radar
            K = np.dot(np.dot(p_radar, jacobian.T), np.linalg.inv(S))
            # Measurement residual
            y = z - X_vector_input_jacobian
            X_state_current = X_state_current + np.dot(K.T, y)
            p_radar = np.dot((I_radar - np.dot(K, jacobian)), p_radar)

            # convert to cartesian
            X_cartesian,Y_cartesian = radar_to_cartesian(X_state_current[0], X_state_current[1])
            X_state_current = np.array([X_cartesian, Y_cartesian])
            X_true_current = np.array(radar_to_polar(currentMeas))
            X_cartesian, Y_cartesian = radar_to_cartesian(X_true_current[0], X_true_current[1])
            X_true_current = np.array([X_cartesian, Y_cartesian])
        xEstimate.append(X_state_current)
        xTrue.append(X_true_current)

    rmse = computeRmse(xEstimate, xTrue)
    print(" root mean square " ,rmse)
