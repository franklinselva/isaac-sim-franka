"""Franka Emika Panda robot interface for Isaac Sim."""

import sys
import numpy as np

import carb
from omni.isaac.core.utils import prims, stage, viewports, rotations, nucleus
from pxr import Gf, UsdGeom
from launcher import ROS_DOMAIN_ID, SIMULATION
import omni.graph.core as og
import usdrt.Sdf


class FrankaArm:
    """Franka Emika Panda robot interface for Isaac Sim."""

    def __init__(self) -> None:
        self._franka_stage_path = "/Franka"
        self._franka_usd_path = "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        self._camera_prim_path = f"{self._franka_stage_path}/panda_hand/geometry/realsense/realsense_camera"
        self._background_stage_path = "/background"
        self._background_usd_path = "/Isaac/Environments/Simple_Room/simple_room.usd"
        self.realsense_viewport_name = "realsense_viewport"
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

    def setup_camera(self):
        """Setup realsense camera for the Franka Emika Panda robot in Isaac Sim."""

        # Fix camera settings since the defaults in the realsense model are inaccurate
        realsense_prim = UsdGeom.Camera(
            stage.get_current_stage().GetPrimAtPath(self._camera_prim_path)
        )
        realsense_prim.GetHorizontalApertureAttr().Set(20.955)
        realsense_prim.GetVerticalApertureAttr().Set(15.7)
        realsense_prim.GetFocalLengthAttr().Set(18.8)
        realsense_prim.GetFocusDistanceAttr().Set(400)

        prims.set_targets(
            prim=stage.get_current_stage().GetPrimAtPath("/ActionGraph/setCamera"),
            attribute="inputs:cameraPrim",
            target_prim_paths=[self._camera_prim_path],
        )

    def declare_action_graph(self):
        """Declare the action graph for the Franka Emika Panda robot with ROS component nodes."""
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                            ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                        ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                        (
                            "SubscribeJointState",
                            "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
                        ),
                        (
                            "ArticulationController",
                            "omni.isaac.core_nodes.IsaacArticulationController",
                        ),
                        ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                        (
                            "getRenderProduct",
                            "omni.isaac.core_nodes.IsaacGetViewportRenderProduct",
                        ),
                        ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                        ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("cameraHelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                        ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                        ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                        (
                            "OnImpulseEvent.outputs:execOut",
                            "ArticulationController.inputs:execIn",
                        ),
                        ("Context.outputs:context", "PublishJointState.inputs:context"),
                        ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                        ("Context.outputs:context", "PublishClock.inputs:context"),
                        (
                            "ReadSimTime.outputs:simulationTime",
                            "PublishJointState.inputs:timeStamp",
                        ),
                        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
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
                        ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                        ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                        ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                        ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                        (
                            "getRenderProduct.outputs:renderProductPath",
                            "setCamera.inputs:renderProductPath",
                        ),
                        ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                        ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                        ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                        ("Context.outputs:context", "cameraHelperRgb.inputs:context"),
                        ("Context.outputs:context", "cameraHelperInfo.inputs:context"),
                        ("Context.outputs:context", "cameraHelperDepth.inputs:context"),
                        (
                            "getRenderProduct.outputs:renderProductPath",
                            "cameraHelperRgb.inputs:renderProductPath",
                        ),
                        (
                            "getRenderProduct.outputs:renderProductPath",
                            "cameraHelperInfo.inputs:renderProductPath",
                        ),
                        (
                            "getRenderProduct.outputs:renderProductPath",
                            "cameraHelperDepth.inputs:renderProductPath",
                        ),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        (
                            "ArticulationController.inputs:robotPath",
                            self._franka_stage_path,
                        ),
                        (
                            "PublishJointState.inputs:targetPrim",
                            [usdrt.Sdf.Path(self._franka_stage_path)],
                        ),
                        (
                            "PublishTF.inputs:targetPrims",
                            [usdrt.Sdf.Path(self._franka_stage_path)],
                        ),
                        ("Context.inputs:domain_id", ROS_DOMAIN_ID),
                        # Setting the /Franka target prim to Articulation Controller node
                        ("ArticulationController.inputs:usePath", True),
                        ("ArticulationController.inputs:robotPath", self._franka_stage_path),
                        ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                        ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
                        ("createViewport.inputs:name", self.realsense_viewport_name),
                        ("createViewport.inputs:viewportId", 1),
                        ("cameraHelperRgb.inputs:frameId", "sim_camera"),
                        ("cameraHelperRgb.inputs:topicName", "rgb"),
                        ("cameraHelperRgb.inputs:type", "rgb"),
                        ("cameraHelperInfo.inputs:frameId", "sim_camera"),
                        ("cameraHelperInfo.inputs:topicName", "camera_info"),
                        ("cameraHelperInfo.inputs:type", "camera_info"),
                        ("cameraHelperDepth.inputs:frameId", "sim_camera"),
                        ("cameraHelperDepth.inputs:topicName", "depth"),
                        ("cameraHelperDepth.inputs:type", "depth"),
                    ],
                },
            )
        except Exception as e:  # pylint: disable=broad-except
            print(e)
