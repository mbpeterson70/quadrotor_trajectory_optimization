from pydrake.all import (
    MathematicalProgram,
    SnoptSolver,
)
import numpy as np
import matplotlib.pyplot as plt

class TrajectoryGenerator():

    def __init__(self, waypoints, headings):
        assert waypoints.shape[0] == 3
        self.headings = headings.reshape((1,-1))
        assert waypoints.shape[1] == self.headings.shape[1]
        self.waypoints = waypoints

        # numeric parameters
        self.kr = 4 # R3 trajectory order
        self.kpsi = 2 # heading trajectory order
        self.m = waypoints.shape[1] - 1 # number of time intervals
        self.ts = 1.0 # unscaled length of time intervals


    def solve(self):
        prog_x, c_x = self._setup_solver(self.kr, self.waypoints[0,:])
        prog_y, c_y = self._setup_solver(self.kr, self.waypoints[1,:])
        prog_z, c_z = self._setup_solver(self.kr, self.waypoints[2,:])
        prog_psi, c_psi = self._setup_solver(self.kpsi, self.headings.reshape(-1))

        cs = [c_x, c_y, c_z, c_psi]
        progs = [prog_x, prog_y, prog_z, prog_psi]
        self.sigma_x, self.sigma_y, self.sigma_z, self.sigma_psi = None, None, None, None
        # sigmas = [self.sigma_x, self.sigma_y, self.sigma_z, self.sigma_psi]
        self.sigmas = [None, None, None, None]

        for i, (c, prog) in enumerate(zip(cs, progs)):
            print(i)
            # solve mathematical program
            solver = SnoptSolver()
            result = solver.Solve(prog)

            # be sure that the solution is optimal
            assert result.is_success()

            # retrieve optimal solution
            self.sigmas[i] = result.GetSolution(c)

    def _setup_solver(self, k, keyframes):
        # initialize optimization
        prog = MathematicalProgram()

        # optimization variables
        c = prog.NewContinuousVariables(k, self.m, "sigma")

        # Zero initial and final velocity
        prog.AddConstraint(c[0,0] == 0.0)
        prog.AddConstraint(np.sum([(1/np.math.factorial(i))*self.ts**i*c[i,self.m-1] for i in range(k)]) == 0.0)

        # Dynamics/Keyframe constraints
        for j in range(self.m):
            eval_derivs = np.sum([(1/np.math.factorial(i))*self.ts**i*c[i-1,j] for i in range(1,k+1)])
            prog.AddConstraint(float(keyframes[j+1]) == float(keyframes[j]) + eval_derivs)

        for j in range(self.m-1):
            for i in range(k-1):
        #         prog.AddConstraint(sigma[(i+1)*n+k] == np.sum([(1/np.math.factorial(j))*ts**j*sigma[i*n+k+j] for j in range(n-k)]))
                prog.AddConstraint(c[i, j+1] == np.sum([(1/np.math.factorial(i2))*self.ts**i2*c[i+i2, j] for i2 in range(k-i)]))

        prog.AddCost(c[k-1,:] @ np.eye(self.m) @ c[k-1,:])

        return prog, c
    
    def eval_trajectory(self, t, n):
        '''
        Evaluates the nth derivative of the trajectory at time t
        '''
        interval = int(t / self.ts)
        curr_ts = t % self.ts

        if n == 0:
            if np.allclose(curr_ts, 0.0):
                return self.waypoints[:,interval]
            else:
                derivs = np.zeros(3)
                for i in range(derivs.shape[0]):
                    derivs[i] = np.sum([(1/np.math.factorial(j))*curr_ts**j*self.sigmas[i][j-1, interval] for j in range(1,self.kr+1)]) 
                return self.waypoints[:,interval] + derivs
        else:
            while interval >= self.sigmas[0].shape[1]:
                interval -= 1
                curr_ts += self.ts
                # print('too long')
            # self.sigmas[0][0, interval]
            # if interval == 11:
            #     import ipdb; ipdb.set_trace()
            traj = np.zeros(3)
            for i in range(3):
                traj[i] = np.sum([(1/np.math.factorial(j))*curr_ts**j*self.sigmas[i][j+n-1, interval] for j in range(self.kr-(n-1))])
            return traj

