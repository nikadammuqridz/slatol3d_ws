import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_slatol_description = get_package_share_directory('slatol3d_description')
    pkg_slatol_bringup = get_package_share_directory('slatol3d_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_slatol_description, 'urdf', 'slatol.urdf.xacro')
    robot_desc = Command(['xacro ', xacro_file])

    # 1. Start Gazebo (Fortress)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2. Spawn Robot (Higher Z to allow boom drop)
    spawn_entity = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'slatol', '-z', '0.5', '-y', '0.0'],
        output='screen'
    )

    # 3. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[{'robot_description': robot_desc}]
    )

    # 4. Spawners
    load_jsb = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster"], output="screen"
    )

    load_jtc = Node(
        package="controller_manager", executable="spawner",
        arguments=["slatol_position_controller"], output="screen"
    )

    # 5. UI Node
    node_ui = Node(
        package='slatol3d_bringup', executable='slatol_ui',
        output='screen'
    )

    # 6. Planner Node
    node_planner = Node(
        package='slatol3d_bringup', executable='slatol_planner',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_jsb, load_jtc],
            )
        ),
        node_ui,
        node_planner,
    ])