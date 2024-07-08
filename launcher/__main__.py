"""Franka ROS2 Bridge Sample"""

from launcher import SIMULATION, SIMULATION_CONTEXT
from .franka import FrankaArm

import omni.graph.core as og
from omni.isaac.core.utils import (
    extensions,
)


def enable_extensions():
    """Enable extensions for the experiment"""
    extensions.enable_extension("omni.isaac.ros2_bridge")


def main():
    """Main function"""

    franka_arm = FrankaArm()

    # Setup steps to be executed in order
    setup_steps = [
        enable_extensions,
        franka_arm.setup,
        franka_arm.declare_action_graph,
        franka_arm.setup_camera,
    ]

    for step in setup_steps:
        step()
        SIMULATION.update()

    # need to initialize physics getting any articulation..etc
    SIMULATION_CONTEXT.initialize_physics()

    SIMULATION_CONTEXT.play()

    while SIMULATION.is_running():
        # Run with a fixed step size
        SIMULATION_CONTEXT.step(render=True)

        # Tick the Publish/Subscribe JointState and Publish Clock nodes each frame
        og.Controller.set(
            og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"),
            True,
        )

    input("Press Enter to close the application")
    SIMULATION_CONTEXT.stop()
    SIMULATION.close()


if __name__ == "__main__":
    main()
