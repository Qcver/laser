from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ================= 全局系底盘里程计 =================
    # 输出：底盘中心在 map 中的位姿（frame_id=map, child_frame_id=base_link）
    #   T_out = A * anchor * (T_WB * T_BC)
    #     T_BC 的平移量由“雷达中心在底盘系中的实测坐标” lidar_x/y/z 推出，
    #     输出点因此从“雷达原点”变成“底盘中心”；
    #     A 是全局偏置：开机瞬间底盘中心 = (map_x, map_y, map_z)、航向 = map_yaw_deg。
    global_chassis_odom_node = Node(
        package='lio_naver',
        executable='global_chassis_odom',
        name='global_chassis_odom',
        output='screen',
        parameters=[{
            # ---- 输入 / 输出 ----
            'input_odom_topic': '/aft_mapped_to_init',
            'output_odom_topic': '/global/chassis_odom',
            'map_frame_id': 'map',
            'base_frame_id': 'base_link',
            'camera_init_frame_id': 'camera_init',
            'publish_rate': 50.0,
            'publish_tf': True,                     # map -> base_link（动态）
            'publish_map_to_camera_init_tf': True,  # map -> camera_init（静态常量，接通 Point-LIO 的 TF 树）
            'publish_base_link_to_body_tf': False,  # 只当 Point-LIO 不再发 TF 时才开，否则 body 会有两个父系

            # ---- 启动锚定（车必须静止，样本数按 Point-LIO 输出频率折算）----
            'anchor_on_start': True,
            'anchor_samples': 10,

            # ---- 安装姿态（2026-10-01 直行标定：车头与 body x 轴差约 58°）----
            'mount_roll_deg': 0.0,
            'mount_pitch_deg': 0.0,
            'mount_yaw_deg': 58.0,

            # ---- 雷达【点云坐标系原点 O】在底盘系中的实测坐标（米，x 右 / y 前 / z 上）----
            #      注意：这里量的是手册《坐标系》一节图上标的那个 O 点，不是 IMU 原点！
            #      两者的差由下面的 trim_* 补上。
            #      t_BC 必须连同安装角一起旋转：t_BC(body) = -R_BC · d_C
            #      R_BC = Rz(-90°+58°) => t_BC = (-0.264, -0.162, -0.047)，|t_BC| = 0.313 m
            #      （别直接取 t_BC=(-d_前, +d_右, -d_上)，那是不含安装角的结果，错的）
            #      启动日志里会打印这组数，可直接核对
            'lidar_x': 0.138,
            'lidar_y': 0.277,
            'lidar_z': 0.047,

            # ---- body 系微调：把参考点从「雷达点云原点 O」挪到「IMU 原点」----
            # 依据手册（《Mid-360 用户手册》坐标系 / IMU 数据 两节）：
            #   · 图上的 O = 点云坐标系原点，点云输出 (0,0,0) 就在这里
            #   · “在点云坐标系下 IMU 芯片的位置为 (x=11.0, y=23.29, z=-44.12) mm”
            #   · “IMU 方向与点云坐标系相同” → 两坐标系只差一个平移，不需旋转
            # 而 Point-LIO 的 body 原点 = IMU 原点（输出的 (0,0,0) 就在这里），
            # 代码里 t_BC = -R_BC·d_C + trim（trim 就在 body 系里相加），
            # 所以 trim 直接取手册那三个数的【负值】：
            #   trim = -p_IMU(body系) = (-0.011, -0.02329, +0.04412)
            # 等价校核：也可把 lidar_* 直接写成 IMU 原点在底盘系中的坐标 (0.135, 0.303, 0.003)，
            #           两种写法算出的 t_BC 完全相同 = (-0.2748, -0.1851, -0.0029)，|t_BC| = 0.331 m
            'trim_x': -0.011,
            'trim_y': -0.02329,
            'trim_z': 0.04412,

            # ---- 全局偏置：开机时底盘中心在 map 中的位姿 ----
            'map_x': 0.663,
            'map_y': 0.875,
            'map_z': 0.15,
            'map_yaw_deg': 0.0,

            # ---- 速度换算 ----
            'convert_twist': True,
            'compensate_lever_arm': True,   # v_center = v_body + ω × (R_WB·t_BC)
            'copy_covariance': True,
        }]
    )

    # ================= 打印 / 记录（专用节点，话题写死 /global/chassis_odom）=================
    # 输出：map 系下“底盘中心相对全局原点”的位姿 x,y,z[m] + roll,pitch,yaw[deg]
    # CSV 与终端同频（默认 10 Hz），行数一致、不会重复膨胀
    global_chassis_printer_node = Node(
        package='lio_naver',
        executable='global_chassis_printer',
        name='global_chassis_printer',
        output='screen',
        parameters=[{
            'print_rate': 10.0,
            'save_csv': True,
            'csv_path': '/home/qcver/laser/global_chassis_log.csv',
        }]
    )

    return LaunchDescription([
        global_chassis_odom_node,
        global_chassis_printer_node,
    ])
