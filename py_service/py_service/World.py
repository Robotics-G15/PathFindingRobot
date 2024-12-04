import os
import random
import cmd
from pyrobosim_ros.ros_interface import WorldROSWrapper
from pyrobosim.planning.actions import TaskPlan, TaskAction
import rclpy
import numpy as np
from rclpy.node import Node
from pyrobosim.core import Robot, World, WorldYamlLoader
from pyrobosim.navigation import ConstantVelocityExecutor, RRTPlanner
from pyrobosim.gui import start_gui
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose
import threading
class WorldCreater:

    def create_world(self, location_file = None, object_file = None):
        """Create a test world"""
        world = World()
        if location_file is None:
            location_file = "/home/chaos08/pyrobosim_ws/src/py_service/py_service/Location.yaml"
        if object_file is None:
            object_file = "/home/chaos08/pyrobosim_ws/src/py_service/py_service/Object.yaml"
        world.set_metadata(
            locations=location_file,
            objects=object_file
        )

        Width = 10
        Height = 10

        r1coords = [(-Width-4, 0), (2, 0), (2, Height), (-Width-4, Height)]
        Warehouse = world.add_room(name="Warehouse", footprint=r1coords, wall_width=0.1, color=[0, 0, 0])

        dif = 3

        Shelves = []

        # Define shelves without associating graph nodes
        Shelves.append(world.add_location(
            name="Item_spawn", category="shelf", parent="Warehouse", pose=Pose(x=float(1), y=float(2), yaw=-np.pi / 1.0), color=[0, 0, 0]
        ))
        for j in range(2, Width, dif):
            for i in range(2, Height, dif):
                Shelves.append(world.add_location(
                    category="shelf", parent="Warehouse", pose=Pose(x=float(-i), y=float(j), yaw=-np.pi / 1.0), color=[0, 0, 0]
                ))
        
        Shelves.append(world.add_location(
            name="Delivery", category="shelf", parent="Warehouse", pose=Pose(x=float(-Width-1), y=float(Height-2), yaw=-np.pi / 1.0), color=[0, 0, 0]
        ))

        return world, Shelves

    def spawn_object(self, world, item_id, location):
        object = world.add_object(category=item_id, parent=location, pose=Pose(x=1.0, y=2.0, yaw=0.0))
        return object

    def create_robot(self, world, name):
        planner_config = {
            "world": world,
            "bidirectional": True,
            "rrt_connect": True,
            "rrt_star": True,
            "collision_check_step_dist": 0.025,
            "max_connection_dist": 0.25,
            "max_nodes_sampled": 1000,
            "max_time": 2.0,
            "rewire_radius": 1.0,
            "compress_path": True
        }

        
        path_planner = RRTPlanner(**planner_config)
        robot = Robot(name=name, radius=0.1, color=[1, 0, 0], initial_battery_level=float("inf"), path_executor=ConstantVelocityExecutor(), path_planner=path_planner)
        
        return robot

    # def navigate(self, item, path):
    #     actions = [
    #     TaskAction(
    #         "navigate",
    #         source_location="Warehouse",
    #         target_location=path[0],
    #     ),]
    #     actions.append(TaskAction("detect", object=item),)
    #     actions.append(TaskAction("pick", object=item),)

    #     for i in range(len(path)-1):
    #         actions.append(
    #             TaskAction(
    #         "navigate",
    #         source_location=path[i],
    #         target_location=path[i+1],
    #     ),)
    #     actions.append(TaskAction("place", object=item),)
    #     plan = TaskPlan(actions=actions)

    #     #result, num_completed = robots[0].execute_plan(plan)
    #     return plan

    # def navigate_return(self, item, path):
    #     actions = []
    #     path = path[:-1]
    #     for i in range(len(path)-1):
    #         actions.append(
    #             TaskAction(
    #         "navigate",
    #         source_location=path[i],
    #         target_location=path[i+1],
    #     ),)
    #     plan = TaskPlan(actions=actions)

    #     #result, num_completed = robots[0].execute_plan(plan)
    #     return plan


def main():
    rclpy.init() 
    creator = WorldCreater()
    world, _ = creator.create_world()
    for i in range(4):
        robot = creator.create_robot(world, f"TaxiBot{i}")
        world.add_robot(robot, loc="Warehouse", pose=Pose(x=-0.5, y=float(i), yaw=-np.pi / 1.0))
    
    node = WorldROSWrapper(world)
    ros_thread = threading.Thread(target=lambda: node.start(wait_for_gui=True))
    ros_thread.start()
    start_gui(node.world)


    """
    Implement ROS: 1: Item spawn, 2: Navigate to goal using path, 3: Navigate to item_spawn / start location

    ros2 run py_service taxi "banana", [item_spawn, shelf1, shelf2, shelf5, shelf8, Delivery]
    
    """

    

if __name__ == "__main__":
    main()
'''#!/usr/bin/env python3

"""
Example showing how to build a world and use it with pyrobosim,
additionally starting up a ROS interface.
"""
import os
import random
from pyrobosim.planning.actions import TaskPlan
from pyrobosim.utils import search_graph
import rclpy
import threading
import numpy as np
from pyrobosim.planning.actions import TaskAction
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
        objects=os.path.join("/home/slane/pyrobosim/World/Object.yaml"),
    )
    Width = 10
    Height = 10



    # Set the location and object metadata
    # Add rooms
    #graph = world.graph_node_from_entity(Graph(Height, Width))
    r1coords = [(-Width, 0),
                (2, 0),
                (2, Width),
                (-Width, Height)]

    r2coords = [(2, 0),
                (5, 0),
                (5, 5),
                (2, 5)]
    
    r3coords = [(2, Height-5),
                (5, Height-5),
                (5, Height),
                (2, Height)]
                
    Warehouse = world.add_room(
                name="Warehouse", footprint=r1coords, wall_width=0.1, color=[0, 0, 0]
            )
    
    
    
    dif = 3
    Nav_points = []
    Shelves = []

    Nav_points.append(world.add_location(
                category="Nav_point", parent="Warehouse", pose=Pose(x=-0.5, y=0.5, yaw=-np.pi / 1.0), color = [0, 0, 0]
            ))
    
    Nav_points.append(world.add_location(
                category="Nav_point", parent="Warehouse", pose=Pose(x=-0.5, y=Height-3.5, yaw=-np.pi / 1.0), color = [0, 0, 0]
            ))
    
    Shelves.append(world.add_location(
                category="shelf", parent="Warehouse", pose=Pose(x=1, y=0.5, yaw=-np.pi / 1.0), color = [0, 0, 0]
            ))
    

    for i in range(2, Width, dif):
        for j in range(2, Height, dif):
            Shelves.append(world.add_location(
                category="shelf", parent="Warehouse", pose=Pose(x=-i, y=j, yaw=-np.pi / 1.0), color = [0, 0, 0]
            ))
            Nav_points.append(world.add_location(
                category="Nav_point", parent="Warehouse", pose=Pose(x=-i-dif/2, y=j-dif/2, yaw=-np.pi / 1.0), color = [0, 0, 0]
            ))
        

    planner_config = {
        "world": world,
        "bidirectional": True,
        "rrt_connect": False,
        "rrt_star": True,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 1.5,
        "rewire_radius": 1.5,
        "compress_path": False,
    }            
    path_planner = RRTPlanner(**planner_config)
    robots = []    
    
    #for i in range(Width//dif):

    robots.append(Robot(name="Taxi_bot", radius=0.1, color = [1, 0, 0], initial_battery_level = float("inf"), path_executor=ConstantVelocityExecutor(), path_planner=path_planner,))
        #robots[i].name = f"Taxi_bot{i}"
    world.add_robot(robots[0], loc="Warehouse", pose=Pose(x=-0.5, y=1, yaw=-np.pi / 1.0))
    world.add_object(category="banana", parent=Shelves[0], pose=Pose(x=1, y=0.5, yaw=0.0))

    
    actions = [
    TaskAction(
        "navigate",
        source_location="Warehouse",
        target_location="shelf0",
    ),
    TaskAction("detect", object="banana"),
    TaskAction("pick", object="banana"),
    TaskAction(
        "navigate",
        source_location="shelf0",
        target_location="shelf8",
    ),
    TaskAction("place", object="banana"),
    ]

    plan = TaskPlan(actions=actions)
    result, num_completed = robots[0].execute_plan(plan)


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
    start_gui(world)'''

'''import os
import random
import cmd
from pyrobosim_ros.ros_interface import WorldROSWrapper
from pyrobosim.planning.actions import TaskPlan, TaskAction
import rclpy
import numpy as np
from rclpy.node import Node
from pyrobosim.core import Robot, World, WorldYamlLoader
from pyrobosim.navigation import ConstantVelocityExecutor, RRTPlanner
from pyrobosim.gui import start_gui
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

class WorldNode(Node):
    data_folder = get_data_folder()
    def __init__(self):
        super().__init__('wode')
        print('recieved')


    def create_world(self):
        """Create a test world"""
        global world
        world = World()
        world.set_metadata(
            locations=os.path.join("/home/chaos08/pyrobosim_ws/src/py_service/py_service/Location.yaml"),
            objects=os.path.join("/home/chaos08/pyrobosim_ws/src/py_service/py_service/Object.yaml"),
        )

        Width = 10
        Height = 10

        r1coords = [(-Width-4, 0), (2, 0), (2, Height), (-Width-4, Height)]
        Warehouse = world.add_room(name="Warehouse", footprint=r1coords, wall_width=0.1, color=[0, 0, 0])

        dif = 3

        Shelves = []

        # Define shelves without associating graph nodes
        Shelves.append(world.add_location(
            name="Item_spawn", category="shelf", parent="Warehouse", pose=Pose(x=1, y=2, yaw=-np.pi / 1.0), color=[0, 0, 0]
        ))
        for j in range(2, Width, dif):
            for i in range(2, Height, dif):
                Shelves.append(world.add_location(
                    category="shelf", parent="Warehouse", pose=Pose(x=-i, y=j, yaw=-np.pi / 1.0), color=[0, 0, 0]
                ))
        
        Shelves.append(world.add_location(
            name="Delivery", category="shelf", parent="Warehouse", pose=Pose(x=-Width-1, y=Height-2, yaw=-np.pi / 1.0), color=[0, 0, 0]
        ))

        return world, Shelves

    def spawn_object(self, world, item_id, location):
        object = world.add_object(category=item_id, parent=location, pose=Pose(x=1, y=2, yaw=0.0))
        return object

    def create_robot(self, world, name):
        planner_config = {
            "world": world,
            "bidirectional": True,
            "rrt_connect": True,
            "rrt_star": True,
            "collision_check_step_dist": 0.025,
            "max_connection_dist": 0.25,
            "max_nodes_sampled": 1000,
            "max_time": 2.0,
            "rewire_radius": 1.0,
            "compress_path": True
        }

        
        path_planner = RRTPlanner(**planner_config)
        robot = Robot(name=name, radius=0.1, color=[1, 0, 0], initial_battery_level=float("inf"), path_executor=ConstantVelocityExecutor(), path_planner=path_planner)
        
        return robot

    def navigate(self, item, path):
        actions = [
        TaskAction(
            "navigate",
            source_location="Warehouse",
            target_location=path[0],
        ),]
        actions.append(TaskAction("detect", object=item),)
        actions.append(TaskAction("pick", object=item),)

        for i in range(len(path)-1):
            actions.append(
                TaskAction(
            "navigate",
            source_location=path[i],
            target_location=path[i+1],
        ),)
        actions.append(TaskAction("place", object=item),)
        plan = TaskPlan(actions=actions)

        #result, num_completed = robots[0].execute_plan(plan)
        return plan

    def navigate_return(self, item, path):
        actions = []
        path = path[:-1]
        for i in range(len(path)-1):
            actions.append(
                TaskAction(
            "navigate",
            source_location=path[i],
            target_location=path[i+1],
        ),)
        plan = TaskPlan(actions=actions)

        #result, num_completed = robots[0].execute_plan(plan)
        return plan


def main():
    rclpy.init() 
    wode = WorldNode()
    world, Shelves = wode.create_world()
    for i in range(4):
        robot = wode.create_robot(world, f"Taxi-Bot{i}")
        world.add_robot(robot, loc="Warehouse", pose=Pose(x=-0.5, y=i, yaw=-np.pi / 1.0))
        #os.system(f'ros2 run py_service taxi {i}')
    node = rclpy.create_node('pyrobosim_world_node')
    #rclpy.spin(wode)

    """


    Implement ROS: 1: Item spawn, 2: Navigate to goal using path, 3: Navigate to item_spawn / start location

    ros2 run py_service taxi "banana", [item_spawn, shelf1, shelf2, shelf5, shelf8, Delivery]
    
    
    """
    
    world_ros_wrapper = WorldROSWrapper(world)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(world_ros_wrapper) 
    executor.add_node(wode)

    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    start_gui(world)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        world_ros_wrapper.destroy_node()
        rclpy.shutdown() 

if __name__ == "__main__":
    main()'''
