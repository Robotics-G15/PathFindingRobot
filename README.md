### Setting Up ###
- Please create a ros2 workspace then in the src file clone our repository and pyrobosim's.
- When in pyrobosim/setup/ please type ``nano setup_pyrobosim.bash`` to edit the ROS_WORKSPACE path to your's
- Then take the team's new_services and py_service from PathFindingRobot to src.
- In your workspace enter ``colcon build``

### To source pyrobosim and build then run our code, run this: ###
```source pyrobosim/setup/source_pyrobosim.bash
source ./install/setup.bash
colcon build --packages-select py_service
ros2 run py_service launch```
