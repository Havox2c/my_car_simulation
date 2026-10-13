import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'my_car_simulation'

    # 1. Đường dẫn tới file mô hình robot Xacro
    pkg_share = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # 2. Biên dịch file mô hình robot Xacro thành XML
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Đường dẫn tới file thế giới (world)
    world = os.path.join(pkg_share, 'worlds', 'track.world')

    # 3. Cấu hình Node Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # 4. Nhúng file launch gốc của Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world}.items()
    )

    # 5. Node đưa mô hình robot vào không gian Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'ackermann_car'],
        output='screen'
    )

    # 6. Spawner nạp Joint State Broadcaster
    # Tăng delay lên 15s để chờ Gazebo + spawn robot hoàn tất trên máy ảo (VM)
    # --controller-manager-timeout 30: chờ thêm tối đa 30s nếu controller_manager chưa sẵn
    joint_state_broadcaster_spawner = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '30',
                ],
                output='screen',
            )
        ]
    )
    
    # 7. Spawner nạp Ackermann Steering Controller
    # Chờ sau joint_state_broadcaster 2 giây để đảm bảo thứ tự đúng
    ackermann_steering_controller_spawner = TimerAction(
        period=17.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'ackermann_steering_controller',
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '30',
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        ackermann_steering_controller_spawner
    ])
