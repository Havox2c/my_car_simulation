import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'my_car_simulation'

    pkg_share = get_package_share_directory(package_name)
    
    # Lưu ý: Kiểm tra tên file này xem có đúng trong thư mục urdf/ không
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Đường dẫn đến file world
    world = os.path.join(pkg_share, 'worlds', 'track.world')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={'world': world}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'ackermann_car'],
        output='screen'
    )

    # Đã sửa: Bỏ bớt argument bị lỗi cut-off từ terminal
    spawn_joint_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Đã sửa: Bỏ bớt argument bị lỗi cut-off từ terminal
    spawn_ackermann_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        TimerAction(period=3.0, actions=[spawn_joint_broadcaster]),
        TimerAction(period=5.0, actions=[spawn_ackermann_controller]),
    ])
