#!/usr/bin/env python3

"""
Example showing how to build a world and use it with pyrobosim,
additionally starting up a ROS interface.
"""
import os
import random
from pyrobosim.planning.actions import TaskPlan
from pyrobosim.utils.search_graph import Node
from pyrobosim.utils.search_graph import SearchGraph
from pyrobosim.utils import search_graph
import rclpy
import threading
import numpy as np
from pyrobosim.planning.actions import TaskAction
from pyrobosim.core import Robot, World, WorldYamlLoader
from pyrobosim.navigation import ConstantVelocityExecutor, PRMPlanner
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
        objects=os.path.join("/home/slane/pyrobosim/World/Object.yaml"),
    )

    Width = 10
    Height = 10

    # Set the location and object metadata
    # Add rooms
    #graph = world.graph_node_from_entity(Graph(Height, Width))
    r1coords = [(-Width-4, 0), (2, 0), (2, Height), (-Width-4, Height)]
                
    Warehouse = world.add_room(name="Warehouse", footprint=r1coords, wall_width=0.1, color=[0, 0, 0])
    g = pyrobosim.utils.search_graph.SearchGraph(color=[1, 0, 0], color_alpha=0, use_planner=False)

    dif = 3
    
    Shelves = []
    Nodes = []
    
    Nodes.append([0, -0.5, -0.5, pyrobosim.utils.search_graph.Node(pose=Pose(x=-0.5, y=0.5), parent=None, cost=0.0)])
    g.add_node(Nodes[-1][3])
    
    #item spawn
    Shelves.append(world.add_location(
                name = "Item spawn", category="shelf", parent="Warehouse", pose=Pose(x=1, y=2, yaw=-np.pi / 1.0), color = [0, 0, 0]
            ))
    
    #gen shelves
    count = 0
    for j in range(2, Width, dif):
        for i in range(2, Height, dif):
            Shelves.append(world.add_location(
                category="shelf", parent="Warehouse", pose=Pose(x=-i, y=j, yaw=-np.pi / 1.0), color = [0, 0, 0]
            ))
            count +=1
            Nodes.append([count, -i-dif/2, j-dif/2, pyrobosim.utils.search_graph.Node(pose=Pose(x=-i-dif/2, y=j-dif/2), parent=None, cost=0.0)])
            g.add_node(Nodes[-1][3])


    Nodes.append([count + 1, -Width-2.5, Height-3.5, pyrobosim.utils.search_graph.Node(pose=Pose(x=-Width-2.5, y=Height-3.5), parent=None, cost=0.0)])
    g.add_node(Nodes[-1][3])
    #delivery / taxi_kill
    Shelves.append(world.add_location(
                name = "Delivery", category="shelf", parent="Warehouse", pose=Pose(x=-Width-1, y=Height-2, yaw=-np.pi / 1.0), color = [0, 0, 0]
            ))
    
    return world, Shelves, Nodes, g

def create_robot(world):
    planner_config = {
        "world": world,
        "compress_path": True,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 2,
        "max_nodes": 250,
        "compress_path": True,
    }          
    
    path_planner = PRMPlanner(**planner_config)
    robot = Robot(name="Taxi_bot", radius=0.1, color = [1, 0, 0], initial_battery_level = float("inf"), path_executor=ConstantVelocityExecutor(), path_planner=path_planner,)
    
    return robot

def custom_robot_loc_to_loc(path):
    actions = []
    for i in range(len(path)-1):
        actions.append(TaskAction(
            "navigate", 
            source_location=path[i],  # Start coordinates
            target_location=path[i+1],   # End coordinates
        ),)
    plan = TaskPlan(actions=actions)
    return plan

def robot_item_pickup(object):
    actions = []
    actions.append(TaskAction("detect", object=object),)
    actions.append(TaskAction("pick", object=object),)
    plan = TaskPlan(actions=actions)
    return plan

def robot_item_place(object):
    actions = []
    actions.append(TaskAction("place", object=object),)
    plan = TaskPlan(actions=actions)
    return plan

def execute_task(Nav_item_plan, item_pick, Nav_goal_plan, item_place, Nav_start_plan):
    result, num_completed = robot.execute_plan(Nav_item_plan)
    result, num_completed = robot.execute_plan(item_pick)
    result, num_completed = robot.execute_plan(Nav_goal_plan)
    result, num_completed = robot.execute_plan(item_place)
    result, num_completed = robot.execute_plan(Nav_start_plan)
     
def spawn_object(item_id, location):
    object = world.add_object(category=item_id, parent=location, pose=Pose(x=1, y=2, yaw=0.0))
    return object


def create_ros_node():
    """Initializes ROS node"""
    rclpy.init()
    # Set the world
    world, Shelves, Nodes, g = create_world()
    return world, Shelves, Nodes, g


if __name__ == "__main__":
    world, Shelves, Nodes, g = create_ros_node()

    robot = create_robot(world)
    world.add_robot(robot, loc="Warehouse", pose=Pose(x=-0.5, y=1, yaw=-np.pi / 1.0))
    object = spawn_object("banana", Shelves[0])

    path = [Shelves[0], g.nearest(pose = Pose(-0.5, 1)), g.nearest(pose = Pose(-2, 1)), g.nearest(pose = Pose(-4, 1)), g.nearest(pose = Pose(-6, 1)), Shelves[2]]

    Nav_item_plan = custom_robot_loc_to_loc([g.nearest(pose = Pose(-0.5, 1)), Shelves[0]])
    item_pick = robot_item_pickup("banana")
    Nav_goal_plan = custom_robot_loc_to_loc(path)
    item_place = robot_item_place("banana")
    Nav_start_plan = custom_robot_loc_to_loc(path[::-1])
    
    #execute_task(Nav_item_plan, item_pick, Nav_goal_plan, item_place, Nav_start_plan)

    # Start GUI in main thread
    start_gui(world)