import numpy as np
from numpy.linalg import norm
from scipy.spatial.transform import Rotation as Rot

def trajectory_to_state(trajectory, quadrotor):
    '''
    trajectory: 4 x at-least 4 x, y, z, heading trajectory at time t
    quadrotor: Drake QuadrotorPlant object
    '''
    vec3 = (3, 1)
    g = np.array([[0., 0., quadrotor.g()]]).T
    z_W = np.array([[0., 0., 1.]]).T
    t = trajectory[:3, 2].reshape(vec3) + g
    z_B = t / norm(t)
    # if np.allclose(trajectory[:3,2], np.zeros(3)):
    #     z_B = g
    # else:
    #     z_B = () / norm(trajectory[:3, 2])
    x_C = np.array([np.cos(trajectory[3,0]), np.sin(trajectory[3,0]), 0.]).reshape(vec3)
    y_B = np.cross(z_B.reshape(-1), x_C.reshape(-1)).reshape(vec3) / norm(np.cross(z_B.reshape(-1), x_C.reshape(-1)))
    x_B = np.cross(y_B.reshape(-1), z_B.reshape(-1)).reshape(vec3)
    R_WB = np.hstack([x_B, y_B, z_B])
    
    adot = trajectory[:3,3].reshape(vec3)
    u1 = quadrotor.m() * norm(t)
    h_omega = (quadrotor.m() / u1) * (adot - (z_B.T@adot)*z_B)
    psidot = trajectory[3,1]

    p = -h_omega.T @ y_B
    q =  h_omega.T @ x_B
    r = psidot * z_W.T @ z_B
    # if p > 1: print('p ', end='')
    # if q > 1: print('q ', end='')
    # if r > 1: print('r ', end='')
    # print()

    state = np.vstack([
        trajectory[:3,0].reshape(vec3),
        Rot.from_matrix(R_WB).as_euler('xyz').reshape(vec3),
        trajectory[:3,1].reshape(vec3),
        np.array([p, q, r]).reshape(vec3)
    ])

    return state
    
# #!/usr/bin/python3
# import math

# import matplotlib.pyplot as plt
# import numpy as np
# from IPython.display import HTML, display
# from pydrake.all import (
#     AddMultibodyPlantSceneGraph,
#     ControllabilityMatrix,
#     DiagramBuilder,
#     Linearize,
#     LinearQuadraticRegulator,
#     MeshcatVisualizer,
#     Parser,
#     Saturation,
#     SceneGraph,
#     Simulator,
#     StartMeshcat,
#     WrapToSystem,
# )
# from pydrake.examples import (
#     AcrobotGeometry,
#     AcrobotInput,
#     AcrobotPlant,
#     AcrobotState,
#     QuadrotorGeometry,
#     QuadrotorPlant,
#     StabilizingLQRController,
# )
# from pydrake.symbolic import Variable
# from pydrake.systems.primitives import SymbolicVectorSystem
# from pydrake.solvers import MathematicalProgram, Solve

# import quadrotor_params as p

# from trajectory_generator import TrajectoryGenerator
# import waypoint_library

# running_as_notebook = True
# meshcat = StartMeshcat()
# builder = DiagramBuilder()

# plant = builder.AddSystem(QuadrotorPlant(
#     m_arg=p.m_kg,
#     L_arg=p.L_m,
#     I_arg=p.J,
#     kF_arg=p.kF,
#     kM_arg=p.kM
# ))

# traj = np.zeros((4,5))
# traj[:,2] = np.ones(4)
# trajectory_to_state(traj, plant)