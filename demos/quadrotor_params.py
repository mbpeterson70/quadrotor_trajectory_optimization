import numpy as np

# Quadrotor Physical Parameters
m_kg = 1.0
L_m = 0.2
Jxx = 0.017
Jyy = Jxx
Jzz = 0.033
J = [[Jxx, 0, 0],
     [0, Jyy, 0],
     [0, 0, Jzz]]
kF = 1.0
kM = 1.0

# Controller Gains
K_p = 100*np.eye(3)
K_v = 100*np.eye(3)
K_R = 10*np.eye(3)
K_omega = 10*np.eye(3)

# Visualization Params
keyframes_pts_per_edge = 20
keyframes_half_width = .2