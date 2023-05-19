import numpy as np
from numpy.linalg import norm
from scipy.spatial.transform import Rotation as Rot

def trajectory_to_state(trajectory, quadrotor):
    '''
    trajectory: 4 x 4: x, y, z, heading trajectory at time t
    quadrotor: Drake QuadrotorPlant object
    returns: 12 x 1 state vector (position, Euler angles, velocity, angular rates)
    '''
    vec3 = (3, 1)
    g = np.array([[0., 0., quadrotor.g()]]).T
    z_W = np.array([[0., 0., 1.]]).T
    t = trajectory[:3, 2].reshape(vec3) + g
    
    # Calculate rotation
    z_B = t / norm(t)
    x_C = np.array([np.cos(trajectory[3,0]), np.sin(trajectory[3,0]), 0.]).reshape(vec3)
    y_B = np.cross(z_B.reshape(-1), x_C.reshape(-1)).reshape(vec3) / norm(np.cross(z_B.reshape(-1), x_C.reshape(-1)))
    x_B = np.cross(y_B.reshape(-1), z_B.reshape(-1)).reshape(vec3)
    R_WB = np.hstack([x_B, y_B, z_B])
    
    # Calculate angular rates
    adot = trajectory[:3,3].reshape(vec3)
    u1 = quadrotor.m() * norm(t)
    h_omega = (quadrotor.m() / u1) * (adot - (z_B.T@adot)*z_B)
    psidot = trajectory[3,1]
    p = -h_omega.T @ y_B
    q =  h_omega.T @ x_B
    r = psidot * z_W.T @ z_B

    state = np.vstack([
        trajectory[:3,0].reshape(vec3),
        Rot.from_matrix(R_WB).as_euler('xyz').reshape(vec3),
        trajectory[:3,1].reshape(vec3),
        np.array([p, q, r]).reshape(vec3)
    ])

    return state