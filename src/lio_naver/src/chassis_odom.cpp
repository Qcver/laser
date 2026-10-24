// chassis_odom.cpp
//
// 作用：把 Point-LIO 的 /aft_mapped_to_init 里程计（世界系 camera_init，z 轴朝上，
//       位姿 = IMU/body 在 camera_init 中的位姿）换算到用户自定义的底盘坐标系：
//
//           底盘坐标系轴向 = x 右 / y 前 / z 上   (右手系)
//
//       并在节点启动时把“底盘中心”锚定为原点：位置 (0,0,0)、朝向 yaw = 0。
//       之后输出的 x,y,z 即“底盘中心在锚定全局系中的坐标”，roll,pitch,yaw 为相对初始朝向的姿态。
//
// 变换链：
//   T_WC(t) = T_WB(t) * T_BC              (T_BC = 安装外参：底盘系在 body 系中的位姿)
//   anchor  = T_WC(t0)^-1                 (启动锚定)
//   T_out   = anchor * T_WC(t)            (输出：base_link 在 chassis_init 中的位姿)
//
// 输出话题：nav_msgs/Odometry (frame_id=chassis_init, child_frame_id=base_link)
// 输出 TF  ：chassis_init -> base_link

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double deg2rad(double d)
{
  return d * M_PI / 180.0;
}
}  // namespace

class ChassisOdom : public rclcpp::Node
{
public:
  ChassisOdom() : Node("chassis_odom")
  {
    // ---------------- 参数声明 ----------------
    input_odom_topic_ = this->declare_parameter<std::string>(
      "input_odom_topic", "/aft_mapped_to_init");
    output_odom_topic_ = this->declare_parameter<std::string>(
      "output_odom_topic", "/chassis/odom");
    chassis_frame_id_ = this->declare_parameter<std::string>("chassis_frame_id", "chassis_init");
    base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");
    publish_rate_ = this->declare_parameter<double>("publish_rate", 50.0);
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);

    // 启动锚定
    anchor_on_start_ = this->declare_parameter<bool>("anchor_on_start", true);
    anchor_samples_ = this->declare_parameter<int>("anchor_samples", 10);
    // 手动指定启动时的 yaw（度）；nan = 不覆盖（保持相对初始朝向为 0）
    anchor_yaw_deg_ = this->declare_parameter<double>("anchor_yaw_deg", kNaN);

    // 安装外参：底盘系在 body(LiDAR/IMU) 系中的姿态(度)与位置(m)
    mount_roll_deg_ = this->declare_parameter<double>("mount_roll_deg", 0.0);
    mount_pitch_deg_ = this->declare_parameter<double>("mount_pitch_deg", 0.0);
    mount_yaw_deg_ = this->declare_parameter<double>("mount_yaw_deg", 0.0);
    mount_x_ = this->declare_parameter<double>("mount_x", 0.0);
    mount_y_ = this->declare_parameter<double>("mount_y", 0.0);
    mount_z_ = this->declare_parameter<double>("mount_z", 0.0);

    convert_twist_ = this->declare_parameter<bool>("convert_twist", true);
    copy_covariance_ = this->declare_parameter<bool>("copy_covariance", true);

    if (anchor_samples_ < 1) {
      anchor_samples_ = 1;
    }
    if (publish_rate_ <= 0.0) {
      publish_rate_ = 50.0;
    }

    // ---------------- 轴系重映射 ----------------
    // body 约定: x 前 / y 左 / z 上 ; 底盘约定: x 右 / y 前 / z 上
    // 由 (f, l, u) -> (-l, f, u) 可得 R_{B<-C} = Rz(-90deg)
    tf2::Quaternion q_axis;
    q_axis.setRPY(0.0, 0.0, -M_PI_2);

    tf2::Quaternion q_mount;
    q_mount.setRPY(deg2rad(mount_roll_deg_), deg2rad(mount_pitch_deg_), deg2rad(mount_yaw_deg_));

    // 底盘系在 body 系中的姿态（先做安装修正，再套轴系重映射）
    tf2::Quaternion q_BC = q_axis * q_mount;
    q_BC.normalize();
    // 底盘系在 body 系中的位姿（平移量以 body 系表达）
    T_BC_ = tf2::Transform(q_BC, tf2::Vector3(mount_x_, mount_y_, mount_z_));

    // 把 body 向量旋转到 chassis 分量：R_{C<-B} = R_{B<-C}^-1
    q_CB_ = q_BC.inverse();

    // ---------------- 订阅 / 发布 ----------------
    auto qos = rclcpp::QoS(rclcpp::KeepLast(50));
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, qos,
      std::bind(&ChassisOdom::odomCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 50);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ChassisOdom::publishOutput, this));

    if (!anchor_on_start_) {
      anchor_ = tf2::Transform::getIdentity();
      anchored_ = true;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "chassis_odom: [%s] (%s) -> [%s] (%s -> %s), anchor_on_start=%s, samples=%d",
      input_odom_topic_.c_str(), "camera_init",
      output_odom_topic_.c_str(), chassis_frame_id_.c_str(), base_frame_id_.c_str(),
      anchor_on_start_ ? "true" : "false", anchor_samples_);
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

    output_ = anchor_ * T_WC;

    if (convert_twist_) {
      // Point-LIO 的 twist.linear 是“世界系”线速度（并非 REP-103 约定的 child 系），
      // twist.angular 是 body 系角速度。这里分别换算到 chassis_init / base_link 系。
      const auto & lin = msg->twist.twist.linear;
      const auto & ang = msg->twist.twist.angular;
      const tf2::Vector3 v_A = tf2::quatRotate(
        anchor_.getRotation(), tf2::Vector3(lin.x, lin.y, lin.z));
      const tf2::Vector3 w_C = tf2::quatRotate(
        q_CB_, tf2::Vector3(ang.x, ang.y, ang.z));
      out_twist_.linear.x = v_A.x();
      out_twist_.linear.y = v_A.y();
      out_twist_.linear.z = v_A.z();
      out_twist_.angular.x = w_C.x();
      out_twist_.angular.y = w_C.y();
      out_twist_.angular.z = w_C.z();
    } else {
      out_twist_ = msg->twist.twist;
    }

    stamp_ = msg->header.stamp;
    if (copy_covariance_) {
      // 说明：这是刚性变换，直接搬移协方差只是近似（未做旋转传播）。
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

    // 可选：把启动瞬间的 yaw 设为指定值
    if (std::isfinite(anchor_yaw_deg_)) {
      tf2::Quaternion q_yaw;
      q_yaw.setRPY(0.0, 0.0, deg2rad(anchor_yaw_deg_));
      anchor_ = tf2::Transform(q_yaw, tf2::Vector3(0, 0, 0)) * anchor_;
    }

    anchored_ = true;

    double r, pp, y;
    tf2::Matrix3x3(T_WC0.getRotation()).getRPY(r, pp, y);
    RCLCPP_INFO(
      this->get_logger(),
      "Anchored at origin: pos=(%.3f, %.3f, %.3f), rpy=(%.2f, %.2f, %.2f) deg (world) -> 归零",
      pos.x(), pos.y(), pos.z(), r * 180.0 / M_PI, pp * 180.0 / M_PI, y * 180.0 / M_PI);
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
    out.header.frame_id = chassis_frame_id_;
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
      tf.header.frame_id = chassis_frame_id_;
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
  }

  // ---- 参数 ----
  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string chassis_frame_id_;
  std::string base_frame_id_;
  double publish_rate_;
  bool publish_tf_;

  bool anchor_on_start_;
  int anchor_samples_;
  double anchor_yaw_deg_;

  double mount_roll_deg_;
  double mount_pitch_deg_;
  double mount_yaw_deg_;
  double mount_x_;
  double mount_y_;
  double mount_z_;

  bool convert_twist_;
  bool copy_covariance_;

  // ---- 变换 ----
  tf2::Transform T_BC_;          // 底盘系在 body 系中的位姿（安装外参 + 轴系重映射）
  tf2::Quaternion q_CB_;         // body -> chassis 旋转
  tf2::Transform anchor_;        // T_WC(t0)^-1

  // ---- 锚定累加 ----
  bool anchored_ = false;
  tf2::Vector3 anchor_pos_sum_{0, 0, 0};
  std::vector<tf2::Quaternion> anchor_quats_;

  // ---- 输出状态 ----
  bool has_output_ = false;
  tf2::Transform output_;        // base_link 在 chassis_init 中的位姿
  geometry_msgs::msg::Twist out_twist_;
  builtin_interfaces::msg::Time stamp_;
  std::array<double, 36> out_pose_cov_{};
  std::array<double, 36> out_twist_cov_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisOdom>());
  rclcpp::shutdown();
  return 0;
}
