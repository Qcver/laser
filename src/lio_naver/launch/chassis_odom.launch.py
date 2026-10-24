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
            #
            # ===== mount_yaw_deg 标定记录（2026-10-01，均用 chassis_position_log.csv）=====
            # ① 原始（参数全 0）直行 2 m：Δ = (-1.7270, +1.0786, -0.0134)，方向相对 +Y 偏 -58.01°
            #    => mount_yaw_deg = atan2(-Δx, Δy) = +58.01°，数值仿真复核后输出 (0, +2.036, 0)
            # ② 改建后直行 3 m 复核：Δ = (+0.0197, +3.0480, -0.0178)，方向 +0.37° → 残余 ≤ 0.4°
            #    残余与"推车时车头自转 1.03°"造成的路径弯曲同量级（弦向偏 ψ/2 ≈ 0.5°），
            #    故不再追调；若要标到 <0.1°，需做 10 m 直线且用导轨/直线基准约束住车头
            # 尺度核对：① 2.036/2.0 = +1.8%，② 3.048/3.0 = +1.6% → 疑有 ~1.7% 系统性偏大，
            #    属 Point-LIO/IMU 层面问题（mount_* 修不了），建议 10 m 测试确认
            # mount_roll_deg：两次 Δz/L 都给出 ε ≈ -0.35%（-0.38% / -0.33%）→ 系统性、符号一致；
            #    但可能与地面坡度混淆（地面斜 0.35% 也是这个量），故暂保持 0
            #    （区分办法：原地转 90° 再直行——安装滚转会给出同样的 Δz/L，地面坡度不会）
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
