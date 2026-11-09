from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 坐标换算 + 位置估算节点
    chassis_odom_node = Node(
        package='lio_naver',
        executable='chassis_odom',
        name='chassis_odom',
        output='screen',
        parameters=[{
            'input_odom_topic': '/aft_mapped_to_init',
            'output_odom_topic': '/chassis/odom',
            'chassis_frame_id': 'chassis_init',
            'base_frame_id': 'base_link',
            'publish_rate': 50.0,
            'publish_tf': True,

            # 启动锚定：开机瞬间底盘中心 = (0,0,0)，yaw = 0
            'anchor_on_start': True,
            'anchor_samples': 10,

            # 安装外参：底盘系在 body(LiDAR/IMU) 系中的姿态(deg)与位置(m)
            'mount_roll_deg': 0.0,
            'mount_pitch_deg': 0.0,
            'mount_yaw_deg': 58.0,
            # 位置：假设雷达即底盘中心（平移到 0 时任何运动输出都不受影响）
            'mount_x': 0.0,
            'mount_y': 0.0,
            'mount_z': 0.0,

            'convert_twist': True,
            'copy_covariance': True,
        }]
    )

    # 精确坐标输出节点 (x, y, z, roll, pitch, yaw)
    chassis_printer_node = Node(
        package='lio_naver',
        executable='chassis_printer',
        name='chassis_printer',
        output='screen',
        parameters=[{
            'odom_topic': '/chassis/odom',
            'print_rate': 10.0,
            'show_orientation': True,
            'save_csv': True,
            'csv_path': '/home/qcver/laser/chassis_position_log.csv',
        }]
    )

    return LaunchDescription([
        chassis_odom_node,
        chassis_printer_node,
    ])
