#!/usr/bin/env python3

"""
Example showing how to build a world and use it with pyrobosim,
additionally starting up a ROS interface.
"""
import os
from pyrobosim.utils import search_graph
import rclpy
import threading
import numpy as np

from pyrobosim.core import Robot, World, WorldYamlLoader
from pyrobosim.navigation import ConstantVelocityExecutor, RRTPlanner
from pyrobosim.gui import start_gui
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
from WarehousePathing import Graph 
import pyrobosim


data_folder = get_data_folder()


def create_world():
    """Create a test world"""
    world = World()
    world.set_metadata(
        locations=os.path.join("/home/slane/pyrobosim/World/Location.yaml"),
    )
    Width = 20
    Height = 20



    # Set the location and object metadata
    # Add rooms
    #graph = world.graph_node_from_entity(Graph(Height, Width))
    r1coords = [(-20, 0),
                (0, 0),
                (0, 20),
                (-20, 20)]
    

    world.add_room(
        name="Warehouse", footprint=r1coords, wall_width=0.1, color=[0, 0, 0]
    )

    world.add_location(
        category="shelf", parent="Warehouse", pose=Pose(x=-3, y=3, yaw=-np.pi / 1.0), color = [0, 0, 0]
    )

    world.add_location(
        category="Nav_point", parent="Warehouse", pose=Pose(x=-5.5, y=1, yaw=-np.pi / 1.0), color = [0, 0, 0]
    )

    world.add_location(
        category="shelf", parent="Warehouse", pose=Pose(x=-8, y=3, yaw=-np.pi / 1.0), color = [0, 0, 0]
    )
    world.add_location(
        category="shelf", parent="Warehouse", pose=Pose(x=-3, y=8, yaw=-np.pi / 1.0), color = [0, 0, 0]
    )
    world.add_location(
        category="shelf", parent="Warehouse", pose=Pose(x=-8, y=8, yaw=-np.pi / 1.0), color = [0, 0, 0]
    )

            


    planner_config = {
        "world": world,
        "bidirectional": True,
        "rrt_connect": False,
        "rrt_star": True,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 0.5,
        "rewire_radius": 1.5,
        "compress_path": False,
    }            
    path_planner = RRTPlanner(**planner_config)
    robot = Robot(
        name="Taxi_bot",
        radius=0.1,
        color = [1, 0, 0],
        initial_battery_level = float("inf"),
        path_executor=ConstantVelocityExecutor(),
        path_planner=path_planner,
    )
    world.add_robot(robot, loc="Warehouse", pose=Pose(x=-0.25, y=0.25, yaw=-np.pi / 1.0))
    return world


def create_ros_node():
    """Initializes ROS node"""
    rclpy.init()
    # Set the world
    world = create_world()

    return world


if __name__ == "__main__":
    world = create_ros_node()

    # Start GUI in main thread
    start_gui(world)
