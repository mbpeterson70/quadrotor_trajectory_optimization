#!/usr/bin/python3
import math

import matplotlib.pyplot as plt
import mpld3
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
import meshcat
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

class Quadrotor():

    def __init__(self):
        running_as_notebook = True
        self.meshcat = StartMeshcat()
        builder = DiagramBuilder()

        plant = builder.AddSystem(QuadrotorPlant(
            m_arg=p.m_kg,
            L_arg=p.L_m,
            I_arg=p.J,
            kF_arg=p.kF,
            kM_arg=p.kM
        ))

        # print(plant.get_input_port(0))
        # print(plant.num_input_ports())
        # print(plant.get_input_port(0).GetFullDescription())
        # c = plant.GetMutableSubsystemContext()
        # print(plant.get_input_port(0).FixValue(c, np.array([1., 1., 1., 1.])))
        # exit()

        # controller = builder.AddSystem(StabilizingLQRController(plant, [0, 0, 1]))
        # builder.Connect(controller.get_output_port(0), plant.get_input_port(0))
        # builder.Connect(plant.get_output_port(0), controller.get_input_port(0))
        props = [Variable(f'p{i}') for i in range(4)]
        controller = builder.AddSystem(SymbolicVectorSystem(state=props, dynamics=np.zeros(4), output=props))
        builder.Connect(controller.get_output_port(0), plant.get_input_port(0))
        # builder.Connect(plant.get_output_port(0), controller.get_input_port(0))

        # Set up visualization in MeshCat
        scene_graph = builder.AddSystem(SceneGraph())
        QuadrotorGeometry.AddToBuilder(
            builder, plant.get_output_port(0), scene_graph
        )
        self.meshcat.Delete()
        self.meshcat.ResetRenderMode()
        self.meshcat.SetProperty("/Background", "visible", False)
        MeshcatVisualizer.AddToBuilder(builder, scene_graph, self.meshcat)
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
            controller_context = diagram.GetMutableSubsystemContext(controller, context)
            controller_context.get_mutable_continuous_state_vector().SetFromVector(np.array([0.25, 0.25, 0.25, 0.25])*9.8)
            # context.SetContinuousState(
            #     0.5
            #     * np.random.randn(
            #         12,
            #     )
            # )
            simulator.Initialize()
            simulator.AdvanceTo(4.0 if running_as_notebook else 0.1)


if __name__ == '__main__':
    Quadrotor()
    import time
    # time.sleep(100)