"""Implementation of an Error State Kalman Filter (ESKF) using quaternions for orientation representation."""
import math
import numpy as np
import pandas as pd
df = pd.read_csv('IMU_data.csv')  # Assuming a CSV file with sensor data

"""Nominal State Methods"""
def omega_matrix(omega):
    wx, wy, wz = omega.flatten()
    Omega = np.block([[0, -wx, -wy, -wz],
                      [wx, 0, wz, -wy],
                      [wy, -wz, 0, wx],
                      [wz, wy, -wx, 0]])
    return Omega

def quaternion_derivative(q, omega):
    Omega = omega_matrix(omega)
    q_dot = 0.5 * q @ Omega
    return q_dot

def update_bias():
    wxw = 1e-5
    wyw = 2e-5
    wzw = 1.5e-5
    process_noise = np.array([[wxw], [wyw], [wzw]])
    return process_noise

def bias_derivative(process_noise):
    return process_noise

def propogate_nominal_state(q, wb, wm, wn):
    process_noise = update_bias()
    q_dot = quaternion_derivative(q, wm - wb - wn)
    wb_dot = bias_derivative(process_noise)
    X_dot = np.vstack((q_dot, wb_dot))
    return X_dot

"""Error State Methods"""
#READER, DONT WORRY ABOUT THIS (this is just to organize my thoughts)
#u = W_measured - W_bias -> u is a matrix with wx, wy, wz (I need to flatten this later in skew symmetric)
#skew_symmetric_matrix(input)

def skew_symmetric(phi):
    vx, vy, vz = phi.flatten()
    phi_skew = np.array([[0, -vz, vy],
                         [vz, 0, -vx],
                         [-vy, vx, 0]])
    return phi_skew

def compute_rotation_matrix(phi):
    phi_skew = skew_symmetric(phi)
    I = np.eye(3)
    R = I + (math.sin(theta)/theta)*phi_skew  + (1-math.cos(theta))/(theta**2)*(phi_skew**2)
    return R.T

def propogate_error_state(wm, wn, am, an, dt):
    Ua = am - an
    Uw = wm - wn
    Uk = np.vstack((Ua, Uw))
    phi = Uk * dt
    R_t = compute_rotation_matrix(phi)
    I = np.eye(3)
    zero = np.zeros((3,3))
    Fk = np.array([[R_t, -dt*I],
                   [zero, I]])
    
    


if __name__ == "__main__":
    q = np.array([[1.0], [0.0],[0.0],[0.0]]) #default quaternion

    while count in range(len(df)-1):
        count = 1

        """INITIALIZAITION OF VARIABLES"""
        #angular velocities
        wx = df['GyroX(rad/s)'][count]
        wy = df['GyroY(rad/s)'][count]
        wz = df['GyroZ(rad/s)'][count]
        wbx = np.zeros((3,1))
        wby = np.zeros((3,1))
        wbz = np.zeros((3,1))
        wnx = np.zeros((3,1))
        wny = np.zeros((3,1))
        wnz = np.zeros((3,1))

        #accelerations
        ax = df['AccX(m/s^2)'][count]
        ay = df['AccY(m/s^2)'][count]
        az = df['AccZ(m/s^2)'][count]
        abx = np.zeros((3,1))
        aby = np.zeros((3,1))
        abz = np.zeros((3,1))
        anx = np.zeros((3,1))
        an_y = np.zeros((3,1))
        anz = np.zeros((3,1))

        wm = np.array([[wx], [wy], [wz]]) #measured angular velocity
        wb = np.array([[wbx], [wby], [wbz]]) #bias for angular velocity
        wn = np.array([[wnx], [wny], [wnz]]) #noise for angular velocity

        am = np.array([[ax], [ay], [az]]) #measured acceleration
        ab = np.array([[abx], [aby], [abz]]) #bias for acceleration
        an = np.array([[anx], [an_y], [anz]]) #bias for acceleration

        X_dot = propogate_nominal_state(q, wb, wm, wn)

##########################################################################################
        #Ua = am - an
        #Uw = wm - wn
        #Uk = np.vstack((Ua, Uw))
        #phi = Uk * dt

        #Now i need
        delta_Xk = propogate_error_state(wm, wn, am, an, dt)