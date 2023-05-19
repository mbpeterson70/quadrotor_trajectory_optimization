import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ControllabilityMatrix,
    DiagramBuilder,
    Linearize,
    LinearQuadraticRegulator,
    MeshcatVisualizer,
    Parser,
    Saturation,
    SceneGraph,
    Simulator,
    StartMeshcat,
    WrapToSystem,
    PointCloud,
    MeshcatPointCloudVisualizer,
    BaseField,
    Fields,
    Rgba
)

def waypoints_to_point_cloud(waypoints, pts_per_edge, half_width):
    num_wp = waypoints.shape[1]
    edges_per_shape = 12

    waypoint_pt_cloud = PointCloud(new_size=num_wp*pts_per_edge*edges_per_shape, fields=Fields(BaseField.kXYZs))
    wp_array = waypoint_pt_cloud.mutable_xyzs()

    for i in range(waypoints.shape[1]):
        wp_offset = i*pts_per_edge*edges_per_shape
        j = 0
        for ii in [-half_width, half_width]:
            for jj in [-half_width, half_width]:
                for kk in np.linspace(-half_width, half_width, pts_per_edge):
                    wp_array[:,wp_offset+j] = waypoints[:,i] + np.array([ii, jj, kk])
                    j += 1
        for ii in [-half_width, half_width]:
            for jj in np.linspace(-half_width, half_width, pts_per_edge):
                for kk in [-half_width, half_width]:
                    wp_array[:,wp_offset+j] = waypoints[:,i] + np.array([ii, jj, kk])
                    j += 1
        for ii in np.linspace(-half_width, half_width, pts_per_edge):
            for jj in [-half_width, half_width]:
                for kk in [-half_width, half_width]:
                    wp_array[:,wp_offset+j] = waypoints[:,i] + np.array([ii, jj, kk])
                    j += 1
    return waypoint_pt_cloud
                    
def constant_rotation_headings(num_waypoints, psi_f=2*np.pi):
    headings = np.linspace(0.0, psi_f, 2*num_waypoints-3).tolist()
    for i in range(num_waypoints-3):
        del headings[-3 - i]
    headings = np.array(headings)
    return headings