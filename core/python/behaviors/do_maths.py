"""Maths."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import numpy as np
from numpy.linalg import pinv
import math
from math import sin, cos, pi
from itertools import product
from scipy import spatial
import sys
from numpy.linalg import inv

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
NP_DATA_FILE = "/home/gguzman/nao/trunk/beacon_data.npz"
TREE = 0
ACT_TREE = 0

sys.setrecursionlimit(20000)

def distance(state, beacon_index):
    return np.linalg.norm(state[:2] - beacon_locs[beacon_index])

def angle(state, beacon_index):
    beacon = beacon_locs[beacon_index]
    angle = np.arctan2(state[1] - beacon[1], state[0] - beacon[0]) - state[2]
    return (angle % (2*pi)) - pi

def f(x, params):
    return params[0] + x*params[1] + x**2*params[2] + x**3*params[3]

def f_grad(x, params):
    return params[1] + 2*x*params[2] + 3*x**2*params[3]

def get_action(action, action_table):
    # Interpolate?
    return action_table[TREE.query(tuple(action))[1],3:]

def sensor_model(state, model_params, beacons_seen):
    sensor_pred = np.zeros((2*len(beacons_seen), ))
    for i, beacon_index in enumerate(beacons_seen):
        sensor_pred[i:i+2] = np.array([f(distance(state, beacon_index), model_params), 
                                       angle(state, beacon_index)])
    return sensor_pred

def rotate_matrix(theta):
    R = np.eye(3)
    R[0][0] = cos(theta)
    R[0][1] = -sin(theta)
    R[1][0] = sin(theta)
    R[1][1] = cos(theta)
    return R

def action_model(state, action, action_table):
    timestep = 1/30.
    R = rotate_matrix(state[2])
    velocitay = get_action(action, action_table)
    return state + R.dot(velocitay) * timestep

def action_noise(Q):
    return np.random.multivariate_normal(np.zeros((3)), Q)

def sensor_noise(R, num_beacons_seen):
    return np.random.multivariate_normal(np.zeros((2*num_beacons_seen)), R)

def gen_A(theta, action, action_table):
    velocitay = get_action(action, action_table)
    A = np.eye(3)
    A[0][2] = -sin(theta) * velocitay[0] - cos(theta) * velocitay[1]
    A[1][2] = cos(theta) * velocitay[0] - sin(theta) * velocitay[1]
    return A

def gen_H(state, model_params, beacons_seen):
    H = []
    for i, (beacon_index, (x, y)) in enumerate(zip(beacons_seen, beacon_locs[beacons_seen])):
        d = distance(state, beacon_index)
        grad = f_grad(d, model_params)
        h = np.zeros((2, 3))
        h[0][0] = (state[0] - x)*grad/d
        h[0][1] = (state[1] - y)*grad/d
        h[1][0] = (y - state[1])/d**2
        h[1][1] = (state[0] - x)/d**2
        h[1][2] = -1
        H.append(h)
    H = np.concatenate(H, axis=0)
    return H

def get_Q():
    Q = np.eye(3) * 10
    Q[2][2] = .1
    return Q

def get_R(sigma_1, sigma_2, num_beacons_seen):
    R = np.eye(2*num_beacons_seen)
    for i in range(R.shape[0]):
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

    mu_prime = np.zeros((3))
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

        obs = obs_matrix[t]
        beacons_seen = get_valid_beacons(obs)
        obs = obs[~np.isnan(obs)]

        # Measurement update
        if beacons_seen:
            R = get_R(sigma_1, sigma_2, len(beacons_seen))
            pred = sensor_model(mu, model_params, beacons_seen)
            pred[1] = (pred[1] % (2*pi)) - pi
            H = gen_H(mu, model_params, beacons_seen)

            K = sigma.dot(H.T.dot(inv(H.dot(sigma).dot(H.T) + R)))
            mu_prime = mu + K.dot((obs - pred))
            sigma_prime = (np.eye(3) - K.dot(H)).dot(sigma)

        mu[2] = (mu[2] % (2*pi)) - pi
        mu_prime[2] = (mu_prime[2] % (2*pi)) - pi

        # Update alpha
        if not beacons_seen:
            alpha[t+1] = mu
            alpha_cov[t+1] = sigma
        else:
            alpha[t+1] = mu_prime
            alpha_cov[t+1] = sigma_prime

    mu_prime = np.zeros((3))
    sigma_prime = np.eye(3) * 1e6

    beta[n] = mu_prime
    beta_cov[n] = sigma_prime

    #BackwardPass 
    for t in range(n-1, -1, -1):
        action = act_matrix[t]
        A = gen_A(mu_prime[2], action, action_table)

        # Time update
        mu = action_model(mu_prime, action, action_table)
        sigma = A.dot(sigma_prime).dot(A.T) - Q

        obs = obs_matrix[t]
        beacons_seen = get_valid_beacons(obs)
        obs = obs[~np.isnan(obs)]

        # Measurement update
        # TODO: Check signs for backward pass
        if beacons_seen:
            R = get_R(sigma_1, sigma_2, len(beacons_seen))
            pred = sensor_model(mu, model_params, beacons_seen)
            pred[1] = (pred[1] % (2*pi)) - pi
            H = gen_H(mu, model_params, beacons_seen)

            K = sigma.dot(H.T).dot(inv(H.dot(sigma).dot(H.T) + R))
            mu_prime = mu - K.dot((obs - pred))
            sigma_prime = (np.eye(3) - K.dot(H)).dot(sigma)

        mu[2] = (mu[2] % (2*pi)) - pi
        mu_prime[2] = (mu_prime[2] % (2*pi)) - pi

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
    R = rotate_matrix(-state[2])
    return R.dot(state[:3] - state[3:])

def get_valid_beacons(obs):
    valid_beacons = []
    for beacon_index, ob in enumerate(np.array_split(obs, 6)):
        if not np.isnan(ob[0]):
            valid_beacons.append(beacon_index)
    return valid_beacons

def sensor_train(obs_matrix, mu_gamma, sigma_gamma, ns):
    n = obs_matrix.shape[0]
    X = []
    y = []
    y2 = []
    for i in range(n):
        valid_beacons = get_valid_beacons(obs_matrix[i])
        num_beacons = len(valid_beacons)
        sample = np.random.multivariate_normal(mu_gamma[i], sigma_gamma[i], ns*num_beacons)
        Xi = np.ones((ns*num_beacons, 5))

        if num_beacons == 0:
            continue
        
        out = obs_matrix[i].reshape(6, 2)[valid_beacons]

        out = np.repeat(out, ns, axis=0)
        valid_beacons = np.repeat(valid_beacons, ns)

        #sample ns*num_beacons x 3
        Xi[:,1] = np.array([distance(x, idx) for x, idx in zip(sample, valid_beacons)])
        Xi[:,2] = Xi[:, 1]**2
        Xi[:,3] = Xi[:, 1]**3
        Xi[:,4] = np.array([angle(x, idx) for x, idx in zip(sample, valid_beacons)])

        yi = out[:, 0]
        y2i = out[:, 1]
        
        X.append(Xi)
        y.append(yi)
        y2.append(y2i)

    y = np.concatenate(y, axis=0)
    y2 = np.concatenate(y2, axis=0)
    X = np.concatenate(X, axis=0)
    model_params = pinv(X[:, :4]).dot(y)
    sigma_1 = np.sqrt(np.mean(np.power(X[:, :4].dot(model_params) - y, 2)))
    sigma_2 = np.sqrt(np.mean(np.power(X[:, 4] - y2, 2)))
    return model_params, sigma_1, sigma_2

def action_train(L, mu_zeta, sigma_zeta, m, act_matrix, action_table):
    for i, action in enumerate(action_table):
        act_indices = ACT_TREE.query_ball_point(action[:3], 1e-6)

        if len(act_indices) == 0:
            continue

        # Get all L, mus, sigmas, for timesteps
        Ls = L[act_indices]
        mus = mu_zeta[act_indices]
        sigmas = sigma_zeta[act_indices]
        ms = m[act_indices]

        # Get previous mu_a for this action
        mu_a = action_table[i][3:]
        mu_a_prime = np.zeros_like(mu_a)
        Q = get_Q()

        # Calculate new mu_a
        for index, act in enumerate(act_matrix[act_indices]):
            L_t = Ls[index]
            sigma_t = sigmas[index]
            mu_t = mus[index]
            mu_a_prime += mu_a + Q.dot(pinv(Q + L_t.dot(sigma_t.dot(L_t.T))).dot(L_t.dot(mu_t) + ms[index] - mu_a))
        mu_a_prime /= len(act_indices)

        # Change action table for this slot with mu_a
        action_table[i][3:] = mu_a_prime

    return action_table

def get_action_table():
    a = [-1/2, -1/6, 0, 1/6, 1/2]
    b = [-3*math.pi/4, -math.pi/2, -math.pi/4, 0,
          math.pi/4, math.pi/2, 3*math.pi/4, math.pi]
    action_table = np.zeros((40, 6))
    actions = list(product(a, b))
    for i, (v_a, v_b) in enumerate(actions):
        vt = v_a
        vx = cos(v_b) * math.sqrt(1 - vt**2)
        vy = sin(v_b) * math.sqrt(1 - vt**2)
        action_table[i] = np.array([vx, vy, vt, 0, 0, 0])
    return action_table

def get_model_params():
    # NOTE: Taken from paper for faster convergence
    return np.array([50, -0.01, 0, 0])

def EM(obs_matrix, act_matrix, num_epochs):
    global TREE, ACT_TREE
    n = obs_matrix.shape[0]
    # NOTE: Taken from paper for faster convergence
    sigma_1, sigma_2 = 10, 0.2
    ns = 1

    action_table = get_action_table()
    model_params = get_model_params()

    TREE = spatial.KDTree(action_table[:,:3])
    ACT_TREE = spatial.KDTree(act_matrix)

    for i in range(num_epochs):
        print("Iteration ", i, " of EM")
        # E-step
        print("E-step")
        alpha, alpha_cov, beta, beta_cov, delta, delta_cov = EKFS(obs_matrix, act_matrix, model_params, action_table, sigma_1, sigma_2)

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
            mu_zeta[t] = np.concatenate((alpha[t], delta[t]), axis=None)
            sigma_zeta[t][:3,:3] = alpha_cov[t]
            sigma_zeta[t][3:,3:] = delta_cov[t]

            # Linearize D
            L[t] = get_L(mu_zeta[t])
            m[t] = D(mu_zeta[t]) - L[t].dot(mu_zeta[t])

        # M-step
        # Sensor model
        print("M-step")
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
    #print(NP_DATA_FILE)
    act_matrix, obs_matrix = get_data(NP_DATA_FILE)
    #print("Action Matrix: ", act_matrix.shape)
    #print(act_matrix)
    #print()
    #print("Observation Matrix: ", obs_matrix.shape)
    #print(obs_matrix)
    #print()
    action_table, model_params, sigma_1, sigma_2 = EM(obs_matrix, act_matrix, EPOCHS)
    #print("Running EM...")
    print(action_table, model_params, sigma_1, sigma_2)

if __name__ == "__main__":
    main()
