import numpy as np
from numpy.linalg import norm, inv
from scipy.spatial.transform import Rotation as Rot
from differential_flatness import trajectory_to_state

def update(trajectory, state, quadrotor, K_p, K_v, K_R, K_omega):
    # K_p = np.eye(3); K_v = np.eye(3)
    # K_R = .1*np.eye(3)
    # K_omega = .1*np.eye(3)

    vec3 = (3, 1)
    pos_traj = trajectory[:3,0].reshape(vec3)
    vel_traj = trajectory[:3,1].reshape(vec3)
    acc_traj = trajectory[:3,2].reshape(vec3)
    psi_traj = trajectory[3,0]
    omega_BW_traj = trajectory_to_state(trajectory, quadrotor)[9:,:]

    pos = state[:3,:]
    R_BW = Rot.from_euler('xyz', state[3:6].reshape(-1)).as_matrix()
    vel = state[6:9,:]
    omega_BW = state[9:,:]

    e_p = pos - pos_traj
    e_v = vel - vel_traj

    z_W = np.array([[0., 0., 1.]]).T
    F_des = -K_p@e_p - K_v@e_v + quadrotor.m()*quadrotor.g()*z_W + quadrotor.m()*acc_traj
    z_B = R_BW[:,2].reshape(vec3)
    u1 = F_des.T @ z_B
    z_B_des = F_des / norm(F_des)

    x_C_des = np.array([np.cos(psi_traj), np.sin(psi_traj), 0.]).reshape(vec3)
    y_B_des = np.cross(z_B_des.reshape(-1), x_C_des.reshape(-1)).reshape(vec3) / norm(np.cross(z_B_des.reshape(-1), x_C_des.reshape(-1)))
    x_B_des = np.cross(y_B_des.reshape(-1), z_B_des.reshape(-1)).reshape(vec3)
    y_B = R_BW[:,1].reshape(vec3)
    x_B = R_BW[:,0].reshape(vec3)
    sim_xy = x_B.T @ x_B_des + y_B.T @ y_B_des
    sim_neg_xy = x_B.T @ -x_B_des + y_B.T @ -y_B_des
    # if sim_xy < sim_neg_xy:
    if True:
        R_des = np.hstack([x_B_des, y_B_des, z_B_des])
    # else:
    #     R_des = np.hstack([-x_B_des, -y_B_des, z_B_des])

    e_R  = .5 * vee(R_des.T @ R_BW - R_BW.T @ R_des)
    e_omega = omega_BW - omega_BW_traj

    u = np.zeros((4,1))
    u[0,0] = u1
    u[1:,:] = -K_R @e_R - K_omega@e_omega

    kF = quadrotor.force_constant()
    kM = quadrotor.moment_constant()
    L = quadrotor.length()
    ctrl_alloc = np.array([
        [kF, kF, kF, kF],
        [0., kF*L, 0., -kF*L],
        [-kF*L, 0., kF*L, 0.],
        [kM, -kM, kM, -kM]
    ])
    motors = inv(ctrl_alloc) @ u
    return motors

def vee(mat):
    x = .5 * (mat[2,1] - mat[1,2])
    y = .5 * (mat[0,2] - mat[2,0])
    z = .5 * (mat[1,0] - mat[0,1])
    return np.array([x, y, z]).reshape((3,1)) 

# def vee(vec):
#     x, y, z = vec.reshape(-1)
#     return np.array([
#         [0., -z, y],
#         [z, 0., -x],
#         [-y, x, 0.]
#     ])