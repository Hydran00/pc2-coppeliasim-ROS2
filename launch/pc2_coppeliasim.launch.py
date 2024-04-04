# Import necessary Python modules
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('input_topic', default_value='/cloud_raw'),
        DeclareLaunchArgument('output_topic', default_value='/cloud_out'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('near_clip', default_value='0.01'),
        DeclareLaunchArgument('far_clip', default_value='3.5'),
        DeclareLaunchArgument('view_angle', default_value='57.0'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('open_rviz', default_value='true'),

        # Define the float32multiarray_to_pointcloud2 node
        Node(
            package='pc2_coppeliasim',
            executable='raw_to_pc2',
            name='raw_to_pc2',
            output='screen',
            parameters=[
                {'frame_id': LaunchConfiguration('frame_id')},
                {'input_topic': LaunchConfiguration('input_topic')},
                {'output_topic': LaunchConfiguration('output_topic')},
                {'near_clip': LaunchConfiguration('near_clip')},
                {'far_clip': LaunchConfiguration('far_clip')},
                {'view_angle': LaunchConfiguration('view_angle')},
                {'height': LaunchConfiguration('height')},
                {'width': LaunchConfiguration('width')}
            ],
            respawn=True
        ),

        # Conditionally launch RViz based on the 'open_rviz' argument
        #Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz',
        #    arguments=['-d', LaunchConfiguration('rviz_config')],
        #    condition=IfCondition(LaunchConfiguration('open_rviz'))
        #)
    ])
