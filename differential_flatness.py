import numpy as np
from numpy.linalg import norm

def trajectory_to_state(trajectory, quadrotor):
    '''
    trajectory: 4 x at-least 3 x, y, z, heading trajectory at time t
    quadrotor: Drake QuadrotorPlant object
    '''
    vec3 = (3, 1)
    g = np.array([[0., 0., quadrotor.g()]]).T
    z_B = (trajectory[:3, 2].reshape(vec3) + g) / norm(trajectory[:3, 2])
    x_C = np.array([np.cos(trajectory[3,0]), np.sin(trajectory[3,0]), 0.]).reshape(vec3)
    y_B = np.cross(z_B, x_C) / norm(np.cross(z_B, x_C))
    x_B = np.cross(y_B, z_B)
    R_WB = np.hstack([x_B, y_B, z_B])
    
    adot = trajectory([:3,3]).reshape(vec3)
    h_omega = 1/norm(trajectory[:3, 2]) * (adot - )
    