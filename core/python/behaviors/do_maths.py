"""Maths."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import numpy as np
from numpy.linalg import pinv
import math
from math import sin, cos, pi

# TODO: Massage data into csv

# Beacon order:
# BLUE_YELLOW,    
# YELLOW_BLUE,    
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
EPOCHS = 150
DATA_FILE = "beacon_data.npz"

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

def sensor_model(state, noise, model_params, beacons_seen):
    sensor_pred = np.zeros((2*len(beacons_seen), 1))
    for i, beacon_index in enumerate(beacons_seen):
        sensor_pred[i:i+2] = np.array([f(distance(state, beacon_index), model_params), 
                                       angle(state[2], beacon_index)]) + noise
    return sensor_pred

def rotate_matrix(theta):
    R = np.eye(3)
    R[0][0] = cos(theta)
    R[0][1] = -sin(theta)
    R[1][0] = sin(theta)
    R[1][1] = cos(theta)
    return R

def action_model(state, noise, action, action_table):
    timestep = 1/30.
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

def gen_H(state, model_params, beacons_seen):
    H = np.zeros((2*len(beacons_seen), 3))
    for i, (beacon_index, (x, y)) in enumerate(zip(beacons_seen, beacon_locs[beacons_seen])):
        d = distance(state, beacon_index)
        grad = f_grad(d, model_params)
        h = np.zeros((2, 3))
        h[0][0] = (state[0] - x)*grad/d
        h[0][1] = (state[1] - y)*grad/d
        h[1][0] = (y - state[1])/d**2
        h[1][1] = (state[0] - x)/d**2
        h[1][2] = -1
        H[i:i+2] = h
    return H

def get_Q():
    Q = np.eye(3) * 10
    Q[2][2] = .1
    return Q

def get_R(sigma_1, sigma_2, num_beacons_seen):
    R = np.eye(2*num_beacons_seen)
    for i in range(noise.shape[0]):
        R[i][i] = sigma_1**2 if i % 2 == 0 else sigma_2**2
    return R

def EKFS(obs_matrix, act_matrix, model_params, action_table, sigma_1, sigma_2):
    n = obs_matrix.shape[0]
    alpha = np.zeros((n+1, 3))
    beta =  np.zeros((n+1, 3))
    delta = np.zeros((n, 3))
    alpha_cov = np.zeros((n+1, 3, 3))
    beta_cov =  np.zeros((n+1, 3, 3))
    delta_cov = np.zeros((n, 3, 3))

    mu_prime = np.zeros((3, 1))
    sigma_prime = np.eye(3)
    Q = get_Q()

    alpha[0] = mu_prime
    alpha_cov[0] = sigma_prime

    #ForwardPass
    # TODO: Discard observations?
    for t in range(n):
        action = act_matrix[t]
        A = gen_A(mu_prime[2], action, action_table)

        # Time update
        mu = action_model(mu_prime, action, action_table)
        sigma = A.dot(sigma_prime).dot(A.T) + Q

        # TODO: Toss distance from observations
        obs = obs_matrix[t]
        beacons_seen = []
        for beacon_index, ob in enumerate(np.array_split(obs, 6)):
            if not np.isnan(ob[0]):
                beacons_seen.append(beacon_index)
            
        R = get_R(sigma_1, sigma_2, len(beacons_seen))
        pred = sensor_pred(mu, R, model_params, beacons_seen)
        H = gen_H(mu_prime, params, beacons_seen)

        # Measurement update
        if beacons_seen:
            K = sigma.dot(H.transpose.dot(pinv(H.dot(sigma).dot(H.T) + R)))
            mu_prime = mu + K.dot((obs - pred))
            sigma_prime = (np.eye(3) - K.dot(H)).dot(sigma)

        # Update alpha
        if not beacons_seen:
            alpha[t+1] = mu
            alpha_cov[t+1] = sigma
        else:
            alpha[t+1] = mu_prime
            alpha_cov[t+1] = sigma_prime

    mu_prime = np.zeros((3, 1))
    sigma_prime = np.eye(3) * np.Inf

    beta[n] = mu_prime
    beta_cov[n] = sigma_prime

    #BackwardPass 
    for t in range(n-1, -1, -1):
        action = act_matrix[t]
        obs = obs_matrix[t]

        # Time update
        mu = action_model(mu_prime, action, action_table)
        sigma = A.dot(sigma_prime.)dot(A.T) - Q

        obs = obs_matrix[t]
        beacons_seen = []
        for beacon_index, ob in enumerate(np.array_split(obs, 6)):
            if not np.isnan(ob[0]):
                beacons_seen.append(beacon_index)

        R = get_R(sigma_1, sigma_2, len(beacons_seen))
        pred = sensor_pred(mu, R, model_params, beacons_seen)
        H = gen_H(mu_prime, params, beacons_seen)

        # Measurement update
        if beacons_seen:
            K = sigma.dot(H.T).dot(pinv(H.dot(sigma).dot(H.T) + R))
            mu_prime = mu - K.dot((obs - pred))
            sigma_prime = (np.eye(3) + K.dot(H)).dot(sigma)

        # Update alpha
        if not beacons_seen:
            beta[t] = mu
            beta_cov[t] = sigma
        else:
            beta[t] = mu_prime
            beta_cov[t] = sigma_prime

        # Delta update
        delta[t] = mu
        delta_cov[t] = sigma

    return alpha, alpha_cov, beta, beta_cov, delta, delta_cov

def get_L(state):
    L = np.zeros((3, 6))
    L[0][0] = cos(state[2])
    L[0][1] = sin(state[2])
    L[0][2] = -sin(state[2])*(state[0]-state[3]) + cos(state[2])*(state[1]-state[4])
    L[0][3] = -cos(state[2])
    L[0][4] = -sin(state[2])
    L[1][0] = -sin(state[2])
    L[1][1] = cos(state[2])
    L[1][2] = -cos(state[2])*(state[0]-state[3]) - sin(state[2])*(state[1]-state[4])
    L[1][3] = sin(state[2])
    L[1][4] = -cos(state[2])
    L[2][2] = 1
    L[2][5] = -1
    return L

def D(state):
    R = get_R(-state[2])
    return R.dot(state[:3] - state[3:])

def sensor_train(obs_matrix, mu_gamma, sigma_gamma, ns):
    n = obs_matrix.shape[0]
    t = n
    #X = np.ones((t*ns, 4))
    X = []
    #y = np.zeros((t*ns, 1))
    #y2 = np.zeros((t*ns, 1))
    y = []
    y2 = []
    for i in range(n):
        valid_beacons = np.array(([obs_matrix[i, j:j+2] for j in range(6) if not np.isnan(obs_matrix[i, 2*j])]))
        num_beacons = len(valid_beacons)
        sample = np.random.multivariate_normal(mu_gamma[t], sigma_gamma[t], ns*num_beacons)
        Xi = np.ones((ns*num_beacons, 5))
        
        #sample ns*num_beacons x 3
        Xi[:,1] = np.array([distance(x) for x in sample.T])
        Xi[:,2] = Xi[:, 1]**2
        Xi[:,3] = Xi[:, 1]**2
        Xi[:,4] = np.array([angle(x) for x in sample.T])
     
        yi = np.zeros((ns*num_beacons, 1))
        y2i = np.zeros((ns*num_beacons, 1))
        
        yi = np.repeat(valid_beacons[:, 0], ns)
        y2i = np.repeat(valid_beacons[:, 1], ns)
        
        X.append(Xi)
        y.append(yi)
        y2.append(y2i)

    X = np.array(X)
    y = np.array(y)
    y2 = np.array(y2)

    model_params = pinv(X[:, :4]).dot(y)
    sigma_1 = np.sqrt(np.mean(np.pow(X[:, 4].dot(model_params) - y, 2)))
    sigma_2 = np.sqrt(np.mean(np.pow(X[:, 4] - y2, 2)))
    return model_params, sigma_1, sigma_2

def action_train(L, mu_zeta, sigma_zeta, m, act_matrix, action_table):
    for action in action_table:
        act_indices = np.nonzero(np.all(act_matrix == action, axis=1))[0]

        # Get all L, mus, sigmas, for timesteps
        Ls = L[act_indices]
        mus = mu_zeta[act_indices]
        sigmas = sigma_zeta[act_indices]
        ms = m[act_indices]

        # Get previous mu_a for this action
        mu_a = action_table[action]
        mu_a_prime = np.zeros_like(mu_a)
        Q = get_Q()

        # Calculate new mu_a
        for index, act in enumerate(act_matrix[act_indices]):
            L_t = Ls[index]
            sigma_t = sigmas[index]
            mu_t = mus[index]
            mu_a_prime += mu_a + 
                Q.dot(pinv(Q + L_t.dot(sigma_t.dot(L_t.T))).dot(L_t.dot(mu_t) + ms[index] - mu_a)
        mu_a_prime /= len(act_indices)

        # Change action table for this slot with mu_a
        action_table[action] = mu_a_prime

    return action_table

def get_action_table():
    a = [-1/2, -1/6, 0, 1/6, 1/2]
    b = [-3*math.pi/4, -math.pi/2, -math.pi/4, 0,
          math.pi/4, math.pi/2, 3*math.pi/4, math.pi]
    action_table = dict()
    actions = list(product(a, b))
    for v_a, v_b in actions:
        vt = v_a
        vx = cos(v_b) * math.sqrt(1 - vt**2)
        vy = sin(v_b) * math.sqrt(1 - vt**2)
        action_table[(vx, vy, vt)] = np.zeros((3, 1))
    return action_table

def get_model_params():
    # NOTE: Taken from paper for faster convergence
    return np.array([50, -0.01, 0, 0])

def EM(obs_matrix, act_matrix, num_epochs):
    n = obs_matrix.shape[0]
    # NOTE: Taken from paper for faster convergence
    sigma_1, sigma_2 = 10, 0.2
    ns = 1

    action_table = get_action_table()
    model_params = get_model_params()

    for _ in range(num_epochs):
        # E-step
        alpha, alpha_cov, beta, beta_cov, delta, delta_cov = 
            EKFS(obs_matrix, act_matrix, model_params, action_table, sigma_1, sigma_2):

        mu_gamma = np.zeros((n, 3))
        sigma_gamma = np.zeros((n, 3, 3))

        mu_zeta = np.zeros((n, 6))
        sigma_zeta = np.zeros((n, 6, 6))
        L = np.zeros((n, 3, 6))
        m = np.zeros((n, 3))

        for t in range(n):
            # Construct gammas
            mu_gamma[t] = pinv(pinv(alpha_cov[t]) + pinv(beta_cov[t])).dot(pinv(alpha_cov[t]).dot(alpha[t]) + pinv(beta_cov[t]).dot(beta[t]))
            sigma_gamma[t] = pinv(pinv(alpha_cov[t]) + pinv(beta_cov[t]))

            # Construct zetas
            mu_zeta[t] = np.concatenate(alpha[t], delta[t], axis=None)
            sigma_zeta[t][:3,:3] = alpha_cov[t]
            sigma_zeta[t][3:,3:] = delta_cov[t]

            # Linearize D
            L[t] = get_L(mu_zeta[t])
            m[t] = D(mu_zeta[t]) - L[t].dot(mu_zeta[t])

        # M-step
        # Sensor model
        model_params, sigma_1, sigma_2 = sensor_train(obs_matrix, mu_gamma, sigma_gamma, ns)

        # Action model
        action_table = action_train(L, mu_zeta, sigma_zeta, m, act_matrix, action_table)

    return action_table, model_params, sigma_1, sigma_2

def get_data(data_file):
    data = np.load(data_file)
    act_matrix = data['actions']
    obs_matrix = data['observations']
    return act_matrix, obs_matrix

def main():
    act_matrix, obs_matrix = get_data(NP_DATA_FILE)
    action_table, model_params, sigma_1, sigma_2 = EM(obs_matrix, act_matrix, EPOCHS)
