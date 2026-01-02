import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_slatol3d_description = get_package_share_directory('slatol3d_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_slatol3d_description, 'urdf', 'slatol.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # Gazebo Sim (Clean Args)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Spawn Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'slatol', '-z', '0.65'],
        output='screen'
    )

    # Bridge (With Sensors)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/slatol/contact@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
            '/slatol/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/slatol/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    # UI & Planner
    ui_node = Node(package='slatol3d_bringup', executable='slatol_ui', output='screen')
    planner_node = Node(package='slatol3d_bringup', executable='slatol_planner', output='screen')

    # Controllers
    joint_state_broadcaster = Node(
        package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]
    )
    robot_controller = Node(
        package="controller_manager", executable="spawner", arguments=["slatol_position_controller"]
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        bridge,
        ui_node,
        planner_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster, robot_controller],
            )
        ),
    ])