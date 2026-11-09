// global_chassis_odom.cpp
//
// 作用：把 Point-LIO 的 /aft_mapped_to_init 换算成
//       “底盘中心在地图系 map 中的位姿”，并输出速度与 TF。
//
//   底盘坐标系 C：x 右 / y 前 / z 上（右手系）
//   body 坐标系 B：x 前 / y 左 / z 上（Point-LIO 输出，假设值）
//
// 相比 chassis_odom 多三级处理：
//   1 平移外参：由“雷达中心在底盘系中的实测坐标” lidar_x/y/z 推出
//        t_BC = -R_BC * d_C + trim            (t_BC 在 body 系中表达)
//      把输出参考点从雷达原点搬到**底盘中心**
//   2 全局偏置：锚定之后再左乘常量 A = ( Rz(map_yaw_deg), (map_x,map_y,map_z) )
//        T_out = A * anchor * T_WC
//      使开机瞬间底盘中心 = (map_x, map_y, map_z)、航向 = map_yaw_deg
//   3 twist 杠杆臂补偿：Point-LIO 的 twist.linear 是 body 原点速度，
//      底盘中心需要补上 ω × r 项
//
// 变换链：
//   R_BC  = Rz(-90°) * RPY(mount_roll, mount_pitch, mount_yaw)
//   T_BC  = (R_BC, t_BC)
//   T_WC  = T_WB * T_BC                      (底盘中心在 W 中的位姿)
//   anchor = T_WC(t0)^-1                     (启动锚定，anchor_samples 平均)
//   T_out = A * anchor * T_WC                (底盘中心在 map 中的位姿)
//
// 输出话题：nav_msgs/Odometry (frame_id=map, child_frame_id=base_link)
// 输出 TF  ：map -> base_link            动态
//            map -> camera_init = A*anchor  静态常量，把 Point-LIO 的 TF 树接进来
//            base_link -> body = T_BC^-1    可选（默认关闭：body 已由 camera_init 作父系）
//
//    若同时运行 chassis_odom，请把它的 publish_tf 置为 false，否则 base_link
//    会同时有 chassis_init 与 map 两个父系，TF 会报多父系错误。

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{
double deg2rad(double d)
{
  return d * M_PI / 180.0;
}
}  

class GlobalChassisOdom : public rclcpp::Node
{
public:
  GlobalChassisOdom() : Node("global_chassis_odom")
  {
    //  输入 / 输出 
    input_odom_topic_ = this->declare_parameter<std::string>(
      "input_odom_topic", "/aft_mapped_to_init");
    output_odom_topic_ = this->declare_parameter<std::string>(
      "output_odom_topic", "/global/chassis_odom");
    map_frame_id_ = this->declare_parameter<std::string>("map_frame_id", "map");
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");
    camera_init_frame_id_ = this->declare_parameter<std::string>(
      "camera_init_frame_id", "camera_init");
    body_frame_id_ = this->declare_parameter<std::string>("body_frame_id", "body");
    publish_rate_ = this->declare_parameter<double>("publish_rate", 50.0);
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
    publish_map_to_camera_init_tf_ = this->declare_parameter<bool>(
      "publish_map_to_camera_init_tf", true);
    publish_base_link_to_body_tf_ = this->declare_parameter<bool>(
      "publish_base_link_to_body_tf", false);

    //  启动锚定 
    anchor_on_start_ = this->declare_parameter<bool>("anchor_on_start", true);
    anchor_samples_ = this->declare_parameter<int>("anchor_samples", 10);
    if (anchor_samples_ < 1) {
      anchor_samples_ = 1;
    }

    // ---------------- 安装姿态（已标定值）----------------
    // 直行标定：车头方向与 body x 轴相差约 58°
    mount_roll_deg_ = this->declare_parameter<double>("mount_roll_deg", 0.0);
    mount_pitch_deg_ = this->declare_parameter<double>("mount_pitch_deg", 0.0);
    mount_yaw_deg_ = this->declare_parameter<double>("mount_yaw_deg", 58.0);

    // ---------------- 雷达中心在底盘系中的实测坐标（米，x 右 / y 前 / z 上）----------------
    lidar_x_ = this->declare_parameter<double>("lidar_x", 0.138);
    lidar_y_ = this->declare_parameter<double>("lidar_y", 0.277);
    lidar_z_ = this->declare_parameter<double>("lidar_z", 0.047);

    // ---------------- body 系微调（一般留 0）----------------
    trim_x_ = this->declare_parameter<double>("trim_x", 0.0);
    trim_y_ = this->declare_parameter<double>("trim_y", 0.0);
    trim_z_ = this->declare_parameter<double>("trim_z", 0.0);

    // ---------------- 全局偏置：开机时底盘中心在 map 中的位姿 ----------------
    map_x_ = this->declare_parameter<double>("map_x", 0.5);
    map_y_ = this->declare_parameter<double>("map_y", 0.5);
    map_z_ = this->declare_parameter<double>("map_z", 0.153);
    map_yaw_deg_ = this->declare_parameter<double>("map_yaw_deg", 0.0);

    // ---------------- 速度 / 协方差 ----------------
    convert_twist_ = this->declare_parameter<bool>("convert_twist", true);
    compensate_lever_arm_ = this->declare_parameter<bool>("compensate_lever_arm", true);
    copy_covariance_ = this->declare_parameter<bool>("copy_covariance", true);

    if (publish_rate_ <= 0.0) {
      publish_rate_ = 50.0;
    }

    // ---------------- 轴系重映射 + 安装姿态 ----------------
    // body 约定: x 前 / y 左 / z 上 ; 底盘约定: x 右 / y 前 / z 上
    // 由 (f, l, u) -> (-l, f, u) 可得 R_{B<-C} = Rz(-90deg)
    tf2::Quaternion q_axis;
    q_axis.setRPY(0.0, 0.0, -M_PI_2);

    tf2::Quaternion q_mount;
    q_mount.setRPY(deg2rad(mount_roll_deg_), deg2rad(mount_pitch_deg_), deg2rad(mount_yaw_deg_));

    // 底盘系在 body 系中的姿态（先做安装修正，再套轴系重映射）
    tf2::Quaternion q_BC = q_axis * q_mount;
    q_BC.normalize();

    // 平移外参：d_C 是“雷达中心在底盘系中的坐标”，而 T_BC 的平移量必须在 body 系中表达：
    //   p_B = R_BC * p_C + t_BC ，雷达中心满足 p_C = d_C、p_B = 0
    //   => t_BC = -R_BC * d_C
    const tf2::Vector3 d_C(lidar_x_, lidar_y_, lidar_z_);
    const tf2::Vector3 t_BC = -tf2::quatRotate(q_BC, d_C) +
      tf2::Vector3(trim_x_, trim_y_, trim_z_);

    T_BC_ = tf2::Transform(q_BC, t_BC);

    // 把 body 向量旋转到 chassis 分量：R_{C<-B} = R_{B<-C}^-1
    q_CB_ = q_BC.inverse();

    // ---------------- 全局偏置 A：map 中的开机位姿 ----------------
    tf2::Quaternion q_map;
    q_map.setRPY(0.0, 0.0, deg2rad(map_yaw_deg_));
    q_map.normalize();
    A_ = tf2::Transform(q_map, tf2::Vector3(map_x_, map_y_, map_z_));

    // ---------------- 订阅 / 发布 ----------------
    auto qos = rclcpp::QoS(rclcpp::KeepLast(50));
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, qos,
      std::bind(&GlobalChassisOdom::odomCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 50);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GlobalChassisOdom::publishOutput, this));

    if (!anchor_on_start_) {
      anchor_ = tf2::Transform::getIdentity();
      anchored_ = true;
    }

    // ---------------- 启动信息（便于一眼核对）----------------
    const tf2::Vector3 chassis_x_in_body = tf2::quatRotate(q_CB_, tf2::Vector3(1, 0, 0));
    RCLCPP_INFO(
      this->get_logger(),
      "global_chassis_odom: [%s] (%s) -> [%s] (%s -> %s)",
      input_odom_topic_.c_str(), camera_init_frame_id_.c_str(),
      output_odom_topic_.c_str(), map_frame_id_.c_str(), base_frame_id_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "  mount rpy=(%.3f, %.3f, %.3f) deg ; 底盘 x(右) 在 body 中 = (%.3f, %.3f, %.3f)",
      mount_roll_deg_, mount_pitch_deg_, mount_yaw_deg_,
      chassis_x_in_body.x(), chassis_x_in_body.y(), chassis_x_in_body.z());
    RCLCPP_INFO(
      this->get_logger(),
      "  lidar in chassis = (%.3f, %.3f, %.3f) m -> t_BC(body) = (%.3f, %.3f, %.3f) m, |t_BC| = %.3f m",
      lidar_x_, lidar_y_, lidar_z_, t_BC.x(), t_BC.y(), t_BC.z(), t_BC.length());
    RCLCPP_INFO(
      this->get_logger(),
      "  map origin: 开机时底盘中心 = (%.3f, %.3f, %.3f) m, yaw = %.2f deg",
      map_x_, map_y_, map_z_, map_yaw_deg_);
    RCLCPP_INFO(
      this->get_logger(),
      "  anchor_on_start=%s, samples=%d, lever_arm_comp=%s",
      anchor_on_start_ ? "true" : "false", anchor_samples_,
      compensate_lever_arm_ ? "true" : "false");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;

    tf2::Quaternion q_WB(q.x, q.y, q.z, q.w);
    if (q_WB.length2() < 1e-12) {
      q_WB = tf2::Quaternion::getIdentity();
    }
    q_WB.normalize();

    // body/IMU 在世界系 W(camera_init) 中的位姿
    const tf2::Transform T_WB(q_WB, tf2::Vector3(p.x, p.y, p.z));
    // 底盘中心在世界系中的位姿
    const tf2::Transform T_WC = T_WB * T_BC_;

    if (anchor_on_start_ && !anchored_) {
      accumulateAnchorSample(T_WC);
      return;  // 锚定完成前不输出
    }

    // 底盘中心在 map 中的位姿：先归到启动时刻，再套全局偏置
    output_ = A_ * (anchor_ * T_WC);

    if (convert_twist_) {
      // Point-LIO 的 twist.linear 是“世界系”线速度（并非 REP-103 约定的 child 系），
      // 且是 **body(IMU) 原点**的速度；twist.angular 是 body 系角速度。
      const auto & lin = msg->twist.twist.linear;
      const auto & ang = msg->twist.twist.angular;
      const tf2::Vector3 omega_B(ang.x, ang.y, ang.z);
      tf2::Vector3 v_center_W(lin.x, lin.y, lin.z);

      if (compensate_lever_arm_) {
        // r = R_WB * t_BC  : 底盘中心相对 body 原点在 W 中的位置
        // v_center = v_body + ω_W × r
        const tf2::Vector3 r_arm_W = tf2::quatRotate(q_WB, T_BC_.getOrigin());
        const tf2::Vector3 omega_W = tf2::quatRotate(q_WB, omega_B);
        v_center_W = v_center_W + omega_W.cross(r_arm_W);
      }

      // 世界系 -> map 系的旋转：R_map_W = rot(A * anchor) = R_A * R_WC0^T
      // （anchor = T_WC0^-1，所以 rot(anchor) = R_WC0^T；A 的旋转是 R_A）
      const tf2::Vector3 v_map = tf2::quatRotate(
        (A_ * anchor_).getRotation(), v_center_W);
      const tf2::Vector3 w_C = tf2::quatRotate(q_CB_, omega_B);
      out_twist_.linear.x = v_map.x();
      out_twist_.linear.y = v_map.y();
      out_twist_.linear.z = v_map.z();
      out_twist_.angular.x = w_C.x();
      out_twist_.angular.y = w_C.y();
      out_twist_.angular.z = w_C.z();
    } else {
      out_twist_ = msg->twist.twist;
    }

    stamp_ = msg->header.stamp;
    if (copy_covariance_) {
      // 说明：这是刚性变换，直接搬移协方差只是近似（未做 R P R^T 旋转传播）。
      out_pose_cov_ = msg->pose.covariance;
      out_twist_cov_ = msg->twist.covariance;
    }

    has_output_ = true;
  }

  // 累加锚定样本（位置求平均、四元数符号对齐后求平均）
  void accumulateAnchorSample(const tf2::Transform & T_WC)
  {
    anchor_pos_sum_ += T_WC.getOrigin();

    tf2::Quaternion qq = T_WC.getRotation();
    if (!anchor_quats_.empty() && anchor_quats_.front().dot(qq) < 0.0) {
      qq = tf2::Quaternion(-qq.x(), -qq.y(), -qq.z(), -qq.w());
    }
    anchor_quats_.push_back(qq);

    if (static_cast<int>(anchor_quats_.size()) >= anchor_samples_) {
      finalizeAnchor();
    }
  }

  void finalizeAnchor()
  {
    const double n = static_cast<double>(anchor_quats_.size());
    const tf2::Vector3 pos = anchor_pos_sum_ * (1.0 / n);

    tf2::Quaternion q_sum(0.0, 0.0, 0.0, 0.0);
    for (const auto & qq : anchor_quats_) {
      q_sum += qq;
    }
    q_sum.normalize();

    const tf2::Transform T_WC0(q_sum, pos);
    anchor_ = T_WC0.inverse();
    anchored_ = true;

    double r, pp, y;
    tf2::Matrix3x3(T_WC0.getRotation()).getRPY(r, pp, y);
    RCLCPP_INFO(
      this->get_logger(),
      "Anchored: 底盘中心当时在 W 中 pos=(%.3f, %.3f, %.3f), rpy=(%.2f, %.2f, %.2f) deg"
      " -> 归零后加上全局偏置，输出起始点 = (%.3f, %.3f, %.3f)",
      pos.x(), pos.y(), pos.z(), r * 180.0 / M_PI, pp * 180.0 / M_PI, y * 180.0 / M_PI,
      map_x_, map_y_, map_z_);
  }

  void publishOutput()
  {
    if (!has_output_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for odometry on %s ...", input_odom_topic_.c_str());
      return;
    }

    const tf2::Vector3 & o = output_.getOrigin();
    const tf2::Quaternion & r = output_.getRotation();

    nav_msgs::msg::Odometry out;
    out.header.stamp = stamp_;
    out.header.frame_id = map_frame_id_;
    out.child_frame_id = base_frame_id_;

    out.pose.pose.position.x = o.x();
    out.pose.pose.position.y = o.y();
    out.pose.pose.position.z = o.z();
    out.pose.pose.orientation.x = r.x();
    out.pose.pose.orientation.y = r.y();
    out.pose.pose.orientation.z = r.z();
    out.pose.pose.orientation.w = r.w();

    out.twist.twist = out_twist_;

    if (copy_covariance_) {
      out.pose.covariance = out_pose_cov_;
      out.twist.covariance = out_twist_cov_;
    }

    pub_->publish(out);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp_;
      tf.header.frame_id = map_frame_id_;
      tf.child_frame_id = base_frame_id_;
      tf.transform.translation.x = o.x();
      tf.transform.translation.y = o.y();
      tf.transform.translation.z = o.z();
      tf.transform.rotation.x = r.x();
      tf.transform.rotation.y = r.y();
      tf.transform.rotation.z = r.z();
      tf.transform.rotation.w = r.w();
      tf_broadcaster_->sendTransform(tf);
    }

    publishStaticTransforms();
  }

  // 静态 TF 只需发一次（StaticTransformBroadcaster 用 transient_local，晚到的订阅者也能收到）
  void publishStaticTransforms()
  {
    if (static_tf_sent_ || !anchored_) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->now();

    if (publish_map_to_camera_init_tf_) {
      // T_map_camera_init = A * anchor  （常量：T_map_W = T_out * T_WC^-1 = A * anchor）
      const tf2::Transform T = A_ * anchor_;
      tf.header.frame_id = map_frame_id_;
      tf.child_frame_id = camera_init_frame_id_;
      tf.transform = toMsg(T);
      static_tf_broadcaster_->sendTransform(tf);
      RCLCPP_INFO(
        this->get_logger(), "Static TF: %s -> %s 已发布（把 Point-LIO 的树接到 map 下）",
        map_frame_id_.c_str(), camera_init_frame_id_.c_str());
    }

    if (publish_base_link_to_body_tf_) {
      // T_base_link_body = T_BC^-1（平移量恰好等于 lidar_x/y/z）
      const tf2::Transform T = T_BC_.inverse();
      tf.header.frame_id = base_frame_id_;
      tf.child_frame_id = body_frame_id_;
      tf.transform = toMsg(T);
      static_tf_broadcaster_->sendTransform(tf);
      RCLCPP_WARN(
        this->get_logger(),
        "Static TF: %s -> %s 已发布；请确认 Point-LIO 未再给 %s 指定其它父系，否则 TF 会报多父系",
        base_frame_id_.c_str(), body_frame_id_.c_str(), body_frame_id_.c_str());
    }

    static_tf_sent_ = true;
  }

  static geometry_msgs::msg::Transform toMsg(const tf2::Transform & T)
  {
    geometry_msgs::msg::Transform m;
    const tf2::Vector3 & t = T.getOrigin();
    const tf2::Quaternion & q = T.getRotation();
    m.translation.x = t.x();
    m.translation.y = t.y();
    m.translation.z = t.z();
    m.rotation.x = q.x();
    m.rotation.y = q.y();
    m.rotation.z = q.z();
    m.rotation.w = q.w();
    return m;
  }

  // ---- 参数 ----
  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string map_frame_id_;
  std::string base_frame_id_;
  std::string camera_init_frame_id_;
  std::string body_frame_id_;
  double publish_rate_;
  bool publish_tf_;
  bool publish_map_to_camera_init_tf_;
  bool publish_base_link_to_body_tf_;

  bool anchor_on_start_;
  int anchor_samples_;

  double mount_roll_deg_;
  double mount_pitch_deg_;
  double mount_yaw_deg_;

  double lidar_x_;
  double lidar_y_;
  double lidar_z_;

  double trim_x_;
  double trim_y_;
  double trim_z_;

  double map_x_;
  double map_y_;
  double map_z_;
  double map_yaw_deg_;

  bool convert_twist_;
  bool compensate_lever_arm_;
  bool copy_covariance_;

  // ---- 变换 ----
  tf2::Transform T_BC_;   // 底盘系在 body 系中的位姿（安装姿态 + 平移实测值 + 轴系重映射）
  tf2::Quaternion q_CB_;  // body -> chassis 旋转
  tf2::Transform A_;      // 全局偏置：开机时底盘中心在 map 中的位姿
  tf2::Transform anchor_; // T_WC(t0)^-1

  // ---- 锚定累加 ----
  bool anchored_ = false;
  tf2::Vector3 anchor_pos_sum_{0, 0, 0};
  std::vector<tf2::Quaternion> anchor_quats_;

  // ---- 输出状态 ----
  bool has_output_ = false;
  bool static_tf_sent_ = false;
  tf2::Transform output_;  // 底盘中心在 map 中的位姿
  geometry_msgs::msg::Twist out_twist_;
  builtin_interfaces::msg::Time stamp_;
  std::array<double, 36> out_pose_cov_{};
  std::array<double, 36> out_twist_cov_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalChassisOdom>());
  rclcpp::shutdown();
  return 0;
}
