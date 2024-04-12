import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


launch_args = [
    # prefixes for multiple sensors
    DeclareLaunchArgument('prefixes', default_value=["1_"]),
    DeclareLaunchArgument('input_topic', default_value='/cloud_raw'),
    DeclareLaunchArgument('output_topic', default_value='/cloud_out'),
    DeclareLaunchArgument('frame_id', default_value='map'),
    # CoppeliaSim camera parameters
    DeclareLaunchArgument('near_clip', default_value='0.001'),
    DeclareLaunchArgument('far_clip', default_value='2.0'),
    DeclareLaunchArgument('view_angle', default_value='57.0'),
    DeclareLaunchArgument('height', default_value='360'),
    DeclareLaunchArgument('width', default_value='360'),
    DeclareLaunchArgument('open_rviz', default_value='true'),
    DeclareLaunchArgument('noise', default_value='true'),
    DeclareLaunchArgument('color', default_value='true')
]

def launch_setup(context):

    prefixes = LaunchConfiguration('prefixes').perform(context)
    prefix1 = prefixes[:2]

    use_sim_time=True
    R = 0
    G = 255
    B = 0

    converter = Node(
        package='pc2_coppeliasim',
        executable='raw_to_pc2',
        name='raw2pc_1',
        output='screen',
        parameters=[
            {'frame_id': prefix1+"camera"},
            {'input_topic': '/cloud_raw_1'},
            {'output_topic': '/cloud_out_1'},
            {'near_clip': LaunchConfiguration('near_clip')},
            {'far_clip': LaunchConfiguration('far_clip')},
            {'view_angle': LaunchConfiguration('view_angle')},
            {'height': LaunchConfiguration('height')},
            {'width': LaunchConfiguration('width')},
            {'noise': LaunchConfiguration('noise')},
            {'color': LaunchConfiguration('color')},
            {'R': R},
            {'G': G},
            {'B': B},
            {'use_sim_time': use_sim_time}
        ],
        respawn=True
    )
        # Visualization
    rviz_config = os.path.join(get_package_share_directory('pc2_coppeliasim'), 'rviz_config', 'rviz.rviz')
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    return [converter, rviz]

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld