import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.actions import OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration as Lc
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch.event_handlers import OnProcessExit

def to_bool(value: str):
    if isinstance(value, bool):
        return value
    if not isinstance(value, str):
        raise ValueError('String to bool, invalid type ' + str(value))

    valid = {'true':True, '1':True,
             'false':False, '0':False}
    
    if value.lower() in valid:
        return valid[value]
    
    raise ValueError('String to bool, invalid value: %s' % value)

def launch_setup(context, *args, **kwargs): 
    debug = Lc('debug').perform(context)
    namespace = Lc('namespace').perform(context)
    x = Lc('x').perform(context)
    y = Lc('y').perform(context)
    z = Lc('z').perform(context)
    roll = Lc('roll').perform(context)
    pitch = Lc('pitch').perform(context)
    yaw = Lc('yaw').perform(context)
    use_sim_time = Lc('use_sim_time').perform(context)
    use_ros2_control = Lc('use_ros2_control').perform(context)
    package_name='shippy'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory('shippy'), 'config', 'gazebo_params.yaml')
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
                )]), launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )
    
    args=(
    '-x %s -y %s -z %s -R %s -P %s -Y %s -entity %s -topic robot_description' 
    %(x, y, z, roll, pitch, yaw, namespace)).split()

    urdf_spawner = Node(
        name = 'urdf_spawner',
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        parameters=[{'use_sim_time': to_bool(use_sim_time)}],
        arguments=args
    )
    diff_drive_spawner = Node(
        name = 'diff_drive_spawner',
        package='controller_manager',
        executable='spawner.py',
        arguments=['diff_cont']
    )
    joint_broad_spawner = Node(
        name = 'joint_broad_spawner',
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_broad']
    )
    set_contoller_manager_use_sim_time = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/controller_manager', 'use_sim_time', use_sim_time],
        output='screen')
    delayed_set_contoller_manager_use_sim_time = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawner,
            on_exit=[set_contoller_manager_use_sim_time],
        )
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=set_contoller_manager_use_sim_time,
            on_exit=[diff_drive_spawner],
        )
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=set_contoller_manager_use_sim_time,
            on_exit=[joint_broad_spawner],
        )
    )
    # return LaunchDescription([
    #     rsp,
    #     gazebo,
    #     urdf_spawner,
    # ])
    # Create a list containing all actions to be included in the launch description
    actions_list = [
        gazebo,
        rsp,
        urdf_spawner,
    ]
    if use_ros2_control == 'true':
        actions_list.extend([
            delayed_set_contoller_manager_use_sim_time,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner
        ])

    # Return the list of actions as part of a LaunchDescription
    return actions_list




def generate_launch_description():
    # TODO Try LaunchContext ?
    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='0'),

        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='0.5'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        DeclareLaunchArgument('mode', default_value='default'),
        DeclareLaunchArgument('namespace', default_value='shippy'),
        DeclareLaunchArgument('use_ned_frame', default_value='false'),
        DeclareLaunchArgument('write_file_on_disk', default_value='false'),

        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_ros2_control', default_value='true'),
        OpaqueFunction(function = launch_setup)
    ])