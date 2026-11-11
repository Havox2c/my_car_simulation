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
    # Đường dẫn đến file cấu hình controller (truyền vào xacro)
    controller_config = os.path.join(pkg_share, 'config', 'car_controller.yaml')

    import re
    robot_description_raw = xacro.process_file(
        xacro_file,
        mappings={'controller_config_path': controller_config}
    ).toxml()
    robot_description_raw = re.sub(r'<!--.*?-->', '', robot_description_raw, flags=re.DOTALL)

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

    # Vị trí spawn: Chính giữa LÀN BÊN PHẢI (y = -0.27m), trước vạch xuất phát (x = 0.3m)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'ackermann_car',
            '-x', '0.3',
            '-y', '-0.27',
            '-z', '0.05',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        output='screen'
    )

    spawn_joint_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30'
        ],
        output='screen'
    )

    spawn_ackermann_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'ackermann_steering_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30'
        ],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        TimerAction(period=5.0, actions=[spawn_entity]),
        TimerAction(period=12.0, actions=[spawn_joint_broadcaster]),
        TimerAction(period=14.0, actions=[spawn_ackermann_controller]),
    ])
