import numpy as np
                    
def constant_rotation_headings(num_waypoints, psi_f=2*np.pi):
    headings = np.linspace(0.0, psi_f, 2*num_waypoints-3).tolist()
    for i in range(num_waypoints-3):
        del headings[-3 - i]
    headings = np.array(headings)
    return headings