from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnShutdown

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='projecte_final_pkg',
            executable='deteccio_exe', # El nom que posarem al setup.py
            name='deteccio'
        ),
        Node(
            package='projecte_final_pkg',
            executable='moviment_exe',
            name='moviment'
        ),
        Node(
            package='projecte_final_pkg',
            executable='cartograf_exe',
            name='cartograf'
        ),
    ])
