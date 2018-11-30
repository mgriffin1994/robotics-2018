"""Maths."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import numpy as np
import math
from math import sin, cos, pi

# TODO: Massage data into csv

# Beacon order:
# BLUE_YELLOW,    
#obs_matrix, act_matrix, model_params, action_table) YELLOW_BLUE,    
# BLUE_PINK,      
# PINK_BLUE,      
# PINK_YELLOW,    
# YELLOW_PINK,    
beacon_locs = np.array(
    [[1400, 950],
     [1400, -950],
     [0, 950],
     [0, -950],
     [-1400, 950],
     [-1400, -950]]
)

def distance(state, beacon_index):
    return np.linalg.norm(state[:2] - beacon_locs[beacon_index])

def angle(state, beacon_index):
    beacon = beacon_locs[beacon_index]
    angle = np.atan2(state[1] - beacon[1], state[0] - beacon[0]) - state[2]
    return (angle % (2*pi)) - pi

def f(x, params):
    return params[0] + x*params[1] + x**2*params[2] + x**3*params[3]

def f_grad(x, params):
    return params[1] + 2*x*params[2] + 3*x**2*params[3]

def A(action, action_table):
    # Interpolate?
    return action_table[action]

def sensor_model(state, noise, model_params):
    sensor_pred = np.zeros((12, 1))

    for beacon_index, _ in enumerate(beacon_locs):
        sensor_pred[beacon_index:beacon_index+2] = 
            np.array([f(distance(state, beacon_index), model_params), 
                      angle(state[2], beacon_index)]) + noise

    return sensor_pred

def rotate_matrix(theta):
    R = np.eye(3)
    R[0][0] = cos(theta)
    R[0][1] = -sin(theta)
    R[1][0] = sin(theta)
    R[1][1] = cos(theta)
    return R

def action_model(state, noise, action, action_table, timestep):
    R = rotate_matrix(state[2])
    velocitay = A(action, action_table)
    return state + R.dot(velocitay) * timestep + noise

def action_noise():
    return np.random.multivariate_normal(np.zeros((3, 1)), get_Q())

def sensor_noise(sigma_1, sigma_2):
    return np.random.multivariate_normal(np.zeros((12, 1)), get_R(sigma_1, sigma_2))

def gen_A(theta, action, action_table):
    velocitay = a(action, action_table)
    A = np.eye(3)
    A[0][2] = -sin(theta) * velocitay[0] - cos(theta) * velocitay[1]
    A[1][2] = cos(theta) * velocitay[0] - sin(theta) * velocitay[1]
    return A

def gen_H(state, model_params):
    H = np.zeros((12, 3))
    for beacon_index, (x, y) in enumerate(beacon_locs):
        d = distance(state, beacon_index)
        grad = f_grad(d, model_params)
        h = np.zeros((2, 3))
        h[0][0] = (state[0] - x)*grad/d
        h[0][1] = (state[1] - y)*grad/d
        h[1][0] = (y - state[1])/d**2
        h[1][1] = (state[0] - x)/d**2
        h[1][2] = -1
        H[beacon_index:beacon_index+2] = h

    return H

def get_Q():
    Q = np.eye(3) * 10
    Q[2][2] = .1
    return Q

def get_R(sigma_1, sigma_2):
    R = np.eye(12)
    for i in range(noise.shape[0]):
        R[i][i] = sigma_1**2 if i % 2 == 0 else sigma_2**2
    return R

def EKFS(obs_matrix, act_matrix, model_params, action_table, sigma_1, sigma_2):
    n = obs_matrix.shape[0]
    alpha = np.zeros((n, 3))
    beta =  np.zeros((n, 3))
    delta = np.zeros((n, 3))

    mu_prime = np.zeros((3, 1))
    sigma_prime = np.eye(3)
    Q = get_Q()
    R = get_R(sigma_1, sigma_2)

    #ForwardPass
    for t in range(n):
        action = act_matrix[t]
        obs = obs_matrix[t]
        H = gen_H(mu_prime, params)
        A = gen_A(mu_prime[2], action, action_table)

        # Time update
        mu_prime = action_model(mu_prime, action, action_table, t)
        sigma_prime = A * sigma_prime * A.transpose() + Q

        # Measurement update
        # TODO: Get rid of nan rows in observation, obs pred, and H
        # TODO: Check if error for inversion
        # TODO: Check if sigma_prime_t == sigma_t
        sensor_pred = (mu_prime, R, model_params)
        K = sigma_prime * H.transpose * np.linalg.inverse(H * sigma_prime * H.transpose() + R)
        mu_prime = mu_prime + K * (obs - sensor_pred)
        sigma_prime = (np.eye(3) - K * H) * sigma_prime

    mu_alpha = mu_prime
    sigma_alpha = sigma_prime

    alpha = mu_alpha, sigma_alpha

    mu_prime = np.zeros((3, 1))
    sigma_prime = np.eye(3) * np.Inf

    #BackwardPass 
    # TODO: Finish/check forward pass and change signs
    for t in range(n):
        pass

    # TODO: Is this delta?

    return alpha, beta, delta


def EM(obs_matrix, act_matrix, model_params, action_table):
    sigma_1, sigma_2 = 0, 0
    alpha, beta, delta = EKFS(obs_matrix, act_matrix, model_params, action_table, sigma_1, sigma_2):

    # E-step
    # M-step
    # KYS-step
