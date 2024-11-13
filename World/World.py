import os
import numpy as np
from pyrobosim.core import World
from pyrobosim.gui.main import start_gui
from pyrobosim.core.yaml_utils import WorldYamlLoader
from pyrobosim.utils.general import get_data_folder
from pyrobosim.utils.pose import Pose

# Then, you can add the other entities
data_folder = get_data_folder()

def create_world_from_yaml(world_file):
    return WorldYamlLoader().from_file(os.path.join(data_folder, world_file))

if __name__ == "__main__":
    world = World()
    world = create_world_from_yaml("/home/slane/pyrobosim/World/World_data.yaml")

    # Start ROS node in separate thread

    # Start GUI in main thread
    start_gui(world)
