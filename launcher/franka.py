"""Franka Emika Panda robot interface for Isaac Sim."""

import sys
import numpy as np

import carb
from omni.isaac.core.utils import prims, stage, viewports, rotations, nucleus
from pxr import Gf
from launcher import SIMULATION
import omni.graph.core as og
import usdrt.Sdf


class FrankaArm:
    """Franka Emika Panda robot interface for Isaac Sim."""

    def __init__(self) -> None:
        self._franka_stage_path = "/Franka"
        self._franka_usd_path = "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        self._background_stage_path = "/background"
        self._background_usd_path = "/Isaac/Environments/Simple_Room/simple_room.usd"
        self._assets_root_path = nucleus.get_assets_root_path()

        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            SIMULATION.close()
            sys.exit()

        self._simulation_context = None

    def setup(self):
        """Setup the Franka Emika Panda robot in Isaac Sim."""

        viewports.set_camera_view(
            eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5])
        )
        stage.add_reference_to_stage(
            self._assets_root_path + self._background_usd_path,
            self._background_stage_path,
        )
        prims.create_prim(
            self._franka_stage_path,
            "Xform",
            position=np.array([0, -0.64, 0]),
            orientation=rotations.gf_rotation_to_np_array(
                Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)
            ),
            usd_path=self._assets_root_path + self._franka_usd_path,
        )
        SIMULATION.update()

    def declare_action_graph(self):
        """Declare the action graph for the Franka Emika Panda robot with ROS component nodes."""
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                        (
                            "ReadSimTime",
                            "omni.isaac.core_nodes.IsaacReadSimulationTime",
                        ),
                        ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                        (
                            "PublishJointState",
                            "omni.isaac.ros2_bridge.ROS2PublishJointState",
                        ),
                        (
                            "SubscribeJointState",
                            "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
                        ),
                        (
                            "ArticulationController",
                            "omni.isaac.core_nodes.IsaacArticulationController",
                        ),
                        ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        (
                            "OnImpulseEvent.outputs:execOut",
                            "PublishJointState.inputs:execIn",
                        ),
                        (
                            "OnImpulseEvent.outputs:execOut",
                            "SubscribeJointState.inputs:execIn",
                        ),
                        (
                            "OnImpulseEvent.outputs:execOut",
                            "PublishClock.inputs:execIn",
                        ),
                        (
                            "OnImpulseEvent.outputs:execOut",
                            "ArticulationController.inputs:execIn",
                        ),
                        ("Context.outputs:context", "PublishJointState.inputs:context"),
                        (
                            "Context.outputs:context",
                            "SubscribeJointState.inputs:context",
                        ),
                        ("Context.outputs:context", "PublishClock.inputs:context"),
                        (
                            "ReadSimTime.outputs:simulationTime",
                            "PublishJointState.inputs:timeStamp",
                        ),
                        (
                            "ReadSimTime.outputs:simulationTime",
                            "PublishClock.inputs:timeStamp",
                        ),
                        (
                            "SubscribeJointState.outputs:jointNames",
                            "ArticulationController.inputs:jointNames",
                        ),
                        (
                            "SubscribeJointState.outputs:positionCommand",
                            "ArticulationController.inputs:positionCommand",
                        ),
                        (
                            "SubscribeJointState.outputs:velocityCommand",
                            "ArticulationController.inputs:velocityCommand",
                        ),
                        (
                            "SubscribeJointState.outputs:effortCommand",
                            "ArticulationController.inputs:effortCommand",
                        ),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        # Setting the /Franka target prim to Articulation Controller node
                        (
                            "ArticulationController.inputs:robotPath",
                            self._franka_stage_path,
                        ),
                        ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                        (
                            "SubscribeJointState.inputs:topicName",
                            "isaac_joint_commands",
                        ),
                        (
                            "PublishJointState.inputs:targetPrim",
                            [usdrt.Sdf.Path(self._franka_stage_path)],
                        ),
                        (
                            "PublishTF.inputs:targetPrims",
                            [usdrt.Sdf.Path(self._franka_stage_path)],
                        ),
                    ],
                },
            )
        except Exception as e:  # pylint: disable=broad-except
            print(e)
