import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

def generate_launch_description():

    # --- 1. Setup Paths (Fortress-compatible) ---
    pkg_slatol_bringup = get_package_share_directory('slatol3d_bringup')
    pkg_slatol_description_share = FindPackageShare('slatol3d_description').find('slatol3d_description')
    
    world_file_path = os.path.join(pkg_slatol_bringup, 'worlds', 'slatol.world')
    xacro_file = os.path.join(pkg_slatol_description_share, 'urdf', 'slatol.urdf.xacro')
    controller_config_file = os.path.join(
        pkg_slatol_description_share,
        'config',
        'slatol_controllers.yaml'
    )
    
    # === FIX: Define robot_description for RSP ===
    robot_description_content = Command([
        'xacro ',
        xacro_file,
    ])

    # === FIX: Set LD_LIBRARY_PATH for Gazebo to find ROS 2 Control plugin (.so file) ===
    # This exposes the necessary shared library paths to the Gazebo process.
    library_path_env = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH', 
        value=[
            os.environ.get('LD_LIBRARY_PATH', ''), 
            ':', 
            os.path.join(FindPackageShare('ros2_controllers').find('ros2_controllers'), 'lib'), 
            ':',
            os.path.join(FindPackageShare('controller_manager').find('controller_manager'), 'lib')
        ]
    )
    
    # === 2. Start Gazebo Fortress (Ignition) ===
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('ros_gz_sim').find('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_file_path}.items()
    )

    # === 3. Robot State Publisher (RSP) ===
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # === 4. Controller Manager Node (CM) - Core of ROS 2 Control ===
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config_file],
        output='screen',
        name='controller_manager',
    )

    # === 5. Spawn Robot Entity ===
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'slatol', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.5'],
        output='screen'
    )

    # === 6. Spawner for Joint State Broadcaster (JSB) and JPC ===
    spawner_delay = TimerAction(
        period=5.0, # Wait 5 seconds for Controller Manager services to stabilize
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['slatol_position_controller', '-c', '/controller_manager'],
                output='screen',
            ),
        ]
    )
    
    # === 7. STABLE Joint State Publisher (Replaces crash-prone GUI) ===
    # This is a robust non-GUI publisher needed for the robot state publisher chain.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    return LaunchDescription([
        library_path_env, # Critical fix for libgazebo_ros2_control.so
        gazebo,
        robot_state_publisher_node,
        controller_manager_node,
        spawn_entity,
        joint_state_publisher, 
        spawner_delay,
    ])