#!/usr/bin/python3
import math

import matplotlib.pyplot as plt
import numpy as np
from IPython.display import HTML, display
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
)
from pydrake.examples import (
    AcrobotGeometry,
    AcrobotInput,
    AcrobotPlant,
    AcrobotState,
    QuadrotorGeometry,
    QuadrotorPlant,
    StabilizingLQRController,
)
from pydrake.symbolic import Variable
from pydrake.systems.primitives import SymbolicVectorSystem
from pydrake.solvers import MathematicalProgram, Solve

import quadrotor_params as p

from trajectory_generator import TrajectoryGenerator
import waypoint_library
from differential_flatness import trajectory_to_state
import trajectory_controller





# waypoints = waypoint_library.waypoints_straight
waypoints = waypoint_library.waypoints_short

headings = np.linspace(0.0, 2*np.pi, 2*waypoints.shape[1]-3).tolist()
for i in range(waypoints.shape[1]-3):
    del headings[-3 - i]
headings = np.array(headings)
headings = np.zeros(headings.shape)

traj_gen = TrajectoryGenerator(waypoints, headings)
traj_gen.solve()

# t = np.linspace(0, traj_gen.tf, 1000)
dt = .001
t = np.arange(0.0, traj_gen.tf, dt)




running_as_notebook = True
meshcat = StartMeshcat()
builder = DiagramBuilder()

plant = builder.AddSystem(QuadrotorPlant(
    m_arg=p.m_kg,
    L_arg=p.L_m,
    I_arg=p.J,
    kF_arg=p.kF,
    kM_arg=p.kM
))

props = [Variable(f'p{i}') for i in range(4)]
controller = builder.AddSystem(SymbolicVectorSystem(state=props, dynamics=np.zeros(4), output=props))
builder.Connect(controller.get_output_port(0), plant.get_input_port(0))

# Set up visualization in MeshCat
scene_graph = builder.AddSystem(SceneGraph())
QuadrotorGeometry.AddToBuilder(
    builder, plant.get_output_port(0), scene_graph
)
meshcat.Delete()
meshcat.ResetRenderMode()
meshcat.SetProperty("/Background", "visible", False)
MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
# end setup for visualization

diagram = builder.Build()

# Set up a simulator to run this diagram
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0 if running_as_notebook else 0.0)
context = simulator.get_mutable_context()
# print(context.GetInputPort())





# Simulate
for i in range(5):
    context.SetTime(0.0)
    simulator.Initialize()

    controller_context = diagram.GetMutableSubsystemContext(controller, context)
    controller_state = controller_context.get_mutable_continuous_state_vector() #.SetFromVector(np.array([0.25, 0.25, 0.25, 0.25])*9.8)
    
    for ti in t[0:]:
        simulator.AdvanceTo(ti)
        traj_matrix = traj_gen.eval_full_trajectory(ti)
        # traj_state = trajectory_to_state(traj_matrix, plant)
        state = context.get_continuous_state_vector().CopyToVector().reshape((16,1))[:12,:]
        # state = np.array().reshape((12,1))
        print(state)
        controller_state.SetFromVector(trajectory_controller.update(traj_matrix, state, plant))

        # context.SetContinuousState(x_part)