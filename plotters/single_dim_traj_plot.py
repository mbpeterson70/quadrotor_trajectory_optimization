from pydrake.all import (
    MathematicalProgram,
    SnoptSolver,
)
import numpy as np
import matplotlib.pyplot as plt

from min_snap_traj_opt.trajectory_generator import TrajectoryGenerator

xs = np.array([
    0.0,
    2.0,
    5.0,
    4.0,
    -1.0,
    -2.0,
    0.0
])
waypoints = np.zeros((3, len(xs)))
waypoints[0,:] = xs
traj_gen = TrajectoryGenerator(waypoints, np.zeros(waypoints.shape[1]))
traj_gen.solve()
dt = .01
t = np.arange(0.0, traj_gen.tf, dt)
xders = np.zeros((traj_gen.kr+1, t.shape[0]))
for i, ti in enumerate(t):
    xders[:,i] = traj_gen.eval_full_trajectory(ti)[0,:]


fig, axs = plt.subplots(traj_gen.kr+1, 1)

axs[0].plot(t, xders[0,:])
axs[0].plot(np.arange(len(xs)), xs, marker="s", linestyle='None')
axs[0].legend(['Trajectory', 'Keyframes'])

for i in range(1, traj_gen.kr+1):
    axs[i].plot(t, xders[i,:])

for i in range(traj_gen.kr):
    axs[i].tick_params(axis='x', which='both', bottom=False, labelbottom=False)

axs[0].set_ylabel('position')
axs[1].set_ylabel('velocity')
axs[2].set_ylabel('acceleration')
axs[3].set_ylabel('jerk')
axs[4].set_ylabel('snap')
axs[4].set_xlabel('time (s)')

plt.show()