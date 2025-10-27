import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file for elevation_mapping with Terrasense integration.
    NOTE: This assumes Gazebo simulation is already running!
    
    First run: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    Then run this file.
    """

    # Package directories
    elevation_mapping_share_dir = get_package_share_directory('elevation_mapping')
    config_dir = os.path.join(elevation_mapping_share_dir, 'config')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Configuration file path
    config_file_path = os.path.join(config_dir, "robots/waffle_robot_terrasense.yaml")
    
    # Prepare parameters list
    list_params = [config_file_path, {'use_sim_time': use_sim_time}]
        
    return launch.LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Terrasense node - publishes terrain classification
        launch_ros.actions.Node(
            package='terra_sense',
            executable='terra_sense_node',
            name='terra_sense_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'terrain_topic': '/terrain_class',
            }]
        ),
        
        # Elevation mapping node with Terrasense integration
        launch_ros.actions.Node(
            package='elevation_mapping',
            executable='elevation_mapping',
            name='elevation_mapping',
            output='screen',
            parameters=list_params,
        ),
        
        # RViz for visualization
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=[
                '--display-config', 
                os.path.join(elevation_mapping_share_dir, 'rviz2', 'custom_rviz2.rviz')
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
