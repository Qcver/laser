// global_chassis_printer.cpp
//
// 作用：打印 / 记录 **底盘中心在全局系 map 中相对全局原点的位姿**。
//
//   订阅：/global/chassis_odom（由 global_chassis_odom 节点发布）
//         header.frame_id = map，child_frame_id = base_link
//   输出：x, y, z          —— 底盘中心相对 map 原点（全局原点）的位置 [m]
//         roll, pitch, yaw —— 底盘在 map 中的姿态 [deg]
//
// 与 chassis_printer 的两处关键差异：
//   ① 话题写死为 /global/chassis_odom，并**校验 frame 语义**（不符只告警一次），
//      避免"拿错数据却以为在看全局坐标"；
//   ② 终端打印与写 CSV 放在**同一个定时器回调**里 —— 两者行数一致、频率可控
//      （默认 10 Hz），不会再因为"每条 odom 写一行"产生大量重复行。
//
// 底盘系约定：x 右 / y 前 / z 上；yaw 以 map 的 +x 轴为 0°。

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace
{
// 专用节点：话题与 frame 语义固定，不开放参数
constexpr const char * kOdomTopic = "/global/chassis_odom";
constexpr const char * kExpectedFrame = "map";
constexpr const char * kExpectedChild = "base_link";
}  // namespace

class GlobalChassisPrinter : public rclcpp::Node
{
public:
  GlobalChassisPrinter() : Node("global_chassis_printer")
  {
    print_rate_ = this->declare_parameter<double>("print_rate", 10.0);
    save_csv_ = this->declare_parameter<bool>("save_csv", true);
    csv_path_ = this->declare_parameter<std::string>(
      "csv_path", "/home/qcver/laser/global_chassis_log.csv");
    if (print_rate_ <= 0.0) {
      print_rate_ = 10.0;
    }

    // ---------------- 订阅（话题写死）----------------
    auto qos = rclcpp::QoS(rclcpp::KeepLast(50));
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      kOdomTopic, qos,
      std::bind(&GlobalChassisPrinter::odomCallback, this, std::placeholders::_1));

    // ---------------- 打印 + 写盘共用同一定时器 ----------------
    auto period = std::chrono::duration<double>(1.0 / print_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GlobalChassisPrinter::tick, this));

    if (save_csv_) {
      csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
      if (csv_file_.is_open()) {
        csv_file_ << "timestamp,x,y,z,roll_deg,pitch_deg,yaw_deg\n";
        RCLCPP_INFO(this->get_logger(), "CSV: %s",
          std::filesystem::absolute(csv_path_).c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Cannot open CSV: %s", csv_path_.c_str());
        save_csv_ = false;
      }
    }

    RCLCPP_INFO(
      this->get_logger(),
      "订阅 %s，以 %.1f Hz 输出【map 系下 底盘中心 相对全局原点】的位姿：x,y,z [m] + roll,pitch,yaw [deg]",
      kOdomTopic, print_rate_);
    RCLCPP_INFO(
      this->get_logger(),
      "坐标约定：x 右 / y 前 / z 上；期望 frame_id=%s, child_frame_id=%s",
      kExpectedFrame, kExpectedChild);

    std::cout << "==== 全局系底盘中心监视器（map 系，原点 = 全局原点）====\n"
              << "Press Ctrl+C to exit\n\n";
  }

  ~GlobalChassisPrinter()
  {
    if (csv_file_.is_open()) {
      csv_file_.close();
      RCLCPP_INFO(this->get_logger(), "CSV saved: %s", csv_path_.c_str());
    }
  }

private:
  // 只缓存最新一帧，实际输出交给定时器（保证 CSV 与终端同频同内容）
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!frame_checked_) {
      frame_checked_ = true;
      if (msg->header.frame_id != kExpectedFrame || msg->child_frame_id != kExpectedChild) {
        RCLCPP_WARN(
          this->get_logger(),
          "输入消息的 frame 是 (%s -> %s)，与预期的 (%s -> %s) 不符："
          "输出的可能不是『map 系下的底盘中心』，请检查上游节点。",
          msg->header.frame_id.c_str(), msg->child_frame_id.c_str(),
          kExpectedFrame, kExpectedChild);
      }
    }

    latest_ = *msg;
    last_rx_ = std::chrono::steady_clock::now();
    has_data_ = true;
  }

  void tick()
  {
    if (!has_data_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for odometry on %s ...", kOdomTopic);
      return;
    }

    const auto & p = latest_.pose.pose.position;
    const auto & q = latest_.pose.pose.orientation;

    double roll, pitch, yaw;
    quatToRPY(q.x, q.y, q.z, q.w, roll, pitch, yaw);

    // ---- 终端 ----
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "X:" << std::setw(8) << p.x
        << "  Y:" << std::setw(8) << p.y
        << "  Z:" << std::setw(8) << p.z
        << "  |  R:" << std::setw(7) << roll  << "°"
        << "  P:" << std::setw(7) << pitch << "°"
        << "  Y:" << std::setw(7) << yaw   << "°";
    std::cout << oss.str() << std::endl;

    // ---- CSV（与上面同一组数）----
    if (save_csv_ && csv_file_.is_open()) {
      const double t = latest_.header.stamp.sec + latest_.header.stamp.nanosec * 1e-9;
      csv_file_ << std::fixed << std::setprecision(6)
                << t << "," << p.x << "," << p.y << "," << p.z << ","
                << roll << "," << pitch << "," << yaw << "\n";
      csv_file_.flush();   // 立即落盘，避免异常退出时丢数据
    }

    // ---- 上游停发提示（超 1 s 没有新数据）----
    const auto silence = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_rx_).count();
    if (silence > 1.0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "%.1f s 没有收到 %s 的新数据，以上为最后一帧（可能上游已停止）",
        silence, kOdomTopic);
    }
  }

  // 四元数转欧拉角（弧度转角度）
  static void quatToRPY(double qx, double qy, double qz, double qw,
                        double & roll, double & pitch, double & yaw)
  {
    tf2::Quaternion q(qx, qy, qz, qw);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);   // 弧度
    roll  = r * 180.0 / M_PI;
    pitch = p * 180.0 / M_PI;
    yaw   = y * 180.0 / M_PI;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry latest_;
  bool has_data_ = false;
  bool frame_checked_ = false;
  std::chrono::steady_clock::time_point last_rx_{};

  double print_rate_;

  std::ofstream csv_file_;
  std::string csv_path_;
  bool save_csv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalChassisPrinter>());
  rclcpp::shutdown();
  return 0;
}
