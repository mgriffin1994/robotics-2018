"""Interpolate."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import

import pprint as pp
import numpy as np
from numpy.linalg import pinv
import math
from math import sin, cos, pi
from itertools import product
from scipy import spatial
import sys
from numpy.linalg import inv
from scipy.stats import multivariate_normal
from scipy.interpolate import griddata
from mpl_toolkits import mplot3d

# Workaround for not having TKinter
import matplotlib.pyplot as plt

desired_actions = np.array(
  [[1.0, 0, 0],
   [-1.0, 0, 0],
   [0, 1.0, 0],
   [0, -1.0, 0]]
)

TREE = 0

def load_data():
    action_table = np.load("action_table.npz")
    model_params = np.load("model_params.npz")
    sigmas = np.load("sigmas.npz")
    return action_table, model_params, sigmas

def get_action(action, action_table):
    # TODO: Convert action from mm/s to state reference frame
    return action_table[TREE.query(tuple(action))[1],:3]

def interpolate(actions, action_table):
    points = action_table['learned']
    values = action_table['desired']

    extent = (min(points[:,0]), max(points[:,0]), min(points[:,1]), max(points[:,1]))

    grid_x, grid_y, grid_t  = np.mgrid[extent[0]:extent[1]:50j, extent[2]:extent[3]:50j, -pi:pi:50j]

    grid_z0 = griddata(points, values, (grid_x, grid_y, grid_t), method='nearest')
    grid_z1 = griddata(points, values, (grid_x, grid_y, grid_t), method='linear')

    grid_x = grid_x.reshape(-1)
    grid_y = grid_y.reshape(-1)
    grid_t = grid_t.reshape(-1)

    action_table = np.concatenate((values, points), axis=1)
    query = np.vstack((grid_x, grid_y, grid_t)).T

    grid_x = grid_x.reshape((50, 50, 50))
    grid_y = grid_y.reshape((50, 50, 50))
    grid_t = grid_t.reshape((50, 50, 50))

    interpolated_actions = get_action(query, action_table)
    interpolated_actions = interpolated_actions.reshape((50, 50, 50, 3))

    for i in range(0, 50, 5):
        plt.subplot(221)
        plt.imshow(interpolated_actions[:,:,i,:], extent=extent, origin='lower')
        plt.plot(points[:,0], points[:,1], 'k.', ms=1)
        plt.title('Original (Theta: {0:.2f})'.format(grid_t[0,0,i]))
        plt.subplot(222)
        plt.imshow(grid_z0[:,:,i,:], extent=extent, origin='lower')
        plt.title('Nearest (Theta: {0:.2f})'.format(grid_t[0,0,i]))
        plt.subplot(223)
        plt.imshow(grid_z1[:,:,i,:], extent=extent, origin='lower')
        plt.title('Linear (Theta: {0:.2f})'.format(grid_t[0,0,i]))
        plt.gcf().set_size_inches(6, 6)
        plt.savefig("real_theta_{}.png".format(i))
        plt.close()

    #fig = plt.figure()
    #ax = plt.axes(projection='3d')

    #plt.subplot(221, projection='3d')
    #ax.plot3D(interpolated_actions[:,0], interpolated_actions[:,1], interpolated_actions[:,2])
    #ax.scatter3D(points[:,0], points[:,1], points[:2])
    #plt.title('Original')
    #plt.subplot(222, projection='3d')
    #ax.plot3D(grid_x, grid_y, grid_t, c=grid_z0)
    #ax.scatter3D(points[:,0], points[:,1], points[:2])
    #plt.title('Nearest')
    #plt.subplot(223, projection='3d')
    #ax.plot3D(grid_x, grid_y, grid_t, c=grid_z1)
    #ax.scatter3D(points[:,0], points[:,1], points[:2])
    #plt.title('Linear')
    #plt.show()
    #plt.close()

def main():
    global TREE
    action_table, model_params, sigmas = load_data()

    TREE = spatial.KDTree(action_table['learned'])

    learned_actions = interpolate(desired_actions, action_table)

if __name__ == "__main__":
    main()
