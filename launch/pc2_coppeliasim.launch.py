# Import necessary Python modules
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


launch_args = [
    # prefixes for multiple sensors
    DeclareLaunchArgument('prefixes', default_value=['1_', '2_']),
    DeclareLaunchArgument('input_topic', default_value='/cloud_raw'),
    DeclareLaunchArgument('output_topic', default_value='/cloud_out'),
    DeclareLaunchArgument('frame_id', default_value='map'),
    # CoppeliaSim camera parameters
    DeclareLaunchArgument('near_clip', default_value='0.01'),
    DeclareLaunchArgument('far_clip', default_value='3.5'),
    DeclareLaunchArgument('view_angle', default_value='57.0'),
    DeclareLaunchArgument('height', default_value='480'),
    DeclareLaunchArgument('width', default_value='480'),
    DeclareLaunchArgument('open_rviz', default_value='true')
]

def launch_setup(context):

    prefixes = LaunchConfiguration('prefixes').perform(context)
    prefix1 = prefixes[0]
    prefix2 = prefixes[1]

    converter1 = Node(
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
            {'width': LaunchConfiguration('width')}
        ],
        respawn=True
    )
    # converter2 = Node(
    #     package='pc2_coppeliasim',
    #     executable='raw_to_pc2',
    #     name='raw2pc_2',
    #     output='screen',
    #     parameters=[
    #         {'frame_id': prefix2+"camera"},
    #         {'input_topic':'/cloud_raw_2'},
    #         {'output_topic': '/cloud_out_2'},
    #         {'near_clip': LaunchConfiguration('near_clip')},
    #         {'far_clip': LaunchConfiguration('far_clip')},
    #         {'view_angle': LaunchConfiguration('view_angle')},
    #         {'height': LaunchConfiguration('height')},
    #         {'width': LaunchConfiguration('width')}
    #     ],
    #     respawn=True
    # )

    # return [converter1, converter2]
    return [converter1]

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld