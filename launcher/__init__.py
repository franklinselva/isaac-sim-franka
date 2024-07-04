"""Launcher module for Isaac Sim."""

from isaacsim import SimulationApp

# Simulation app should be created first and once to load all the shared libraries and initialize the simulation
CONFIG = {"renderer": "RayTracedLighting", "headless": False}

SIMULATION = SimulationApp(CONFIG)


from omni.isaac.core import SimulationContext  # pylint: disable=wrong-import-position

SIMULATION_CONTEXT = SimulationContext(stage_units_in_meters=1.0)
