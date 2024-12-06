import launch
from launch import LaunchDescription
from launch_ros.actions import Node
# link has diffculty opening on uni wifi, takes a while when opening elsewhere be patient
#https://answers.ros.org/question/201989/can-i-debug-node-in-gdb-without-new-xterm/

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_service',
            executable='World',
            name='World_node',
            output='screen',
        ),
        Node(
            package='py_service',
            executable='server',
            name='taxi_server_node',
            output='screen',
            prefix='xterm -hold -e'
        ),
        Node(
            package='py_service',
            executable='client',
            name='client_node',
            output='screen',
            arguments=['banana,cow,frame', '1,2,3'],
            prefix='xterm -hold -e'
        ),
        Node(
            package='py_service',
            executable='delivery',
            name='delivery_node',
            output='screen',
            arguments=['banana', '1'],
            prefix='xterm -hold -e'
        ),
        Node(
            package='py_service',
            executable='taxi',
            name='taxi_node',
            output='screen',
            prefix='xterm -hold -e'
        ),
        Node(
            package='py_service',
            executable='hive',
            name='hive_node',
            output='screen',
            prefix='xterm -hold -e'
        ),
        Node(
            package='py_service',
            executable='claw',
            name='claw_node',
            output='screen',
            prefix='xterm -hold -e'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
