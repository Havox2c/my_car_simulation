import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name = 'my_car_simulation'

    # 1. Đường dẫn tới file mô hình robot Xacro
    pkg_share = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # Đường dẫn đến file cấu hình controller (truyền vào xacro)
    controller_config = os.path.join(pkg_share, 'config', 'car_controller.yaml')

    # 2. Biên dịch file mô hình robot Xacro thành XML (truyền đường dẫn config)
    # Loại bỏ toàn bộ XML comment để tránh lỗi parse rule '--param robot_description:=...' trong gazebo_ros2_control
    import re
    robot_description_raw = xacro.process_file(
        xacro_file,
        mappings={'controller_config_path': controller_config}
    ).toxml()
    robot_description_raw = re.sub(r'<!--.*?-->', '', robot_description_raw, flags=re.DOTALL)

    # Đường dẫn tới file thế giới (world)
    world = os.path.join(pkg_share, 'worlds', 'track.world')

    # Thêm thư mục models vào GAZEBO_MODEL_PATH để Gazebo tìm được race_track
    models_path = os.path.join(pkg_share, 'models')
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')]
    )

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

    # 5. Node đưa mô hình robot vào không gian Gazebo (delay 5s chờ Gazebo sẵn sàng)
    # Vị trí spawn: Chính giữa LÀN BÊN PHẢI (y = -0.27m), trước vạch xuất phát (x = 0.3m)
    # Đường đua rộng 1.1m từ y = -0.55m đến y = +0.55m (vạch tim đường vàng tại y = 0.0m)
    # Làn bên phải nằm từ y = -0.50m đến y = 0.0m => Tâm làn bên phải: y = -0.27m
    # Vạch xuất phát nằm tại x = 0.50m => Xe đứng trước vạch xuất phát tại x = 0.30m
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'ackermann_car',
                    '-x', '0.3',     # Đứng trước vạch xuất phát 0.2m
                    '-y', '-0.27',   # Chính giữa làn bên phải (y âm)
                    '-z', '0.05',    # Nâng nhẹ khỏi mặt đường
                    '-R', '0.0',     # Roll = 0
                    '-P', '0.0',     # Pitch = 0
                    '-Y', '0.0',     # Yaw = 0 (hướng mũi xe dọc theo đường đua)
                ],
                output='screen'
            )
        ]
    )

    # 6. Spawner nạp Joint State Broadcaster
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

    # 8. NMPC Controller Node (delay 20s - sau khi tất cả controllers đã sẵn sàng)
    # Đọc trạng thái từ /odom, /scan, /imu; xuất lệnh tới /cmd_vel
    nmpc_node = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='my_car_simulation',
                executable='nmpc_node.py',
                name='microcontroller_nmpc_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    # 9. Sensor Monitor Node - Hiển thị bảng dữ liệu IMU/EKF/LiDAR ra terminal (delay 20s)
    # In trạng thái: vị trí EKF, Yaw, vận tốc, yaw_rate IMU, khoảng cách vật cản
    sensor_monitor_node = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='my_car_simulation',
                executable='sensor_monitor_node.py',
                name='sensor_monitor_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    # 10. Image View - Xem tầm nhìn camera tích hợp nhận diện làn đường (delay 20s)
    # Hiển thị trực tiếp camera cùng overlay phát hiện vạch kẻ đường, tâm làn và sai số
    image_view_node = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='image_view',
                executable='image_view',
                name='camera_view',
                output='screen',
                remappings=[('image', '/camera/lane_debug')],
            )
        ]
    )

    return LaunchDescription([
        gazebo_model_path,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        ackermann_steering_controller_spawner,
        # === CÁC NODE ĐIỀU KHIỂN VÀ GIÁM SÁT (khởi động sau khi controllers sẵn sàng) ===
        nmpc_node,
        sensor_monitor_node,
        image_view_node,
    ])
