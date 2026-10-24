#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <filesystem>

class PositionPrinter : public rclcpp::Node
{
public:
  // 构造函数
  PositionPrinter() : Node("position_printer")
  { 
    // 声明参数
    this->declare_parameter<std::string>("odom_topic", "/aft_mapped_to_init");
    this->declare_parameter<double>("print_rate", 10.0);
    this->declare_parameter<bool>("show_orientation", true);
    this->declare_parameter<bool>("save_csv", true);
    this->declare_parameter<std::string>("csv_path", "/home/qcver/laser/position_log.csv");
  
    // 获取参数
    odom_topic_      = this->get_parameter("odom_topic").as_string();
    print_rate_      = this->get_parameter("print_rate").as_double();
    show_orientation_= this->get_parameter("show_orientation").as_bool();
    save_csv_        = this->get_parameter("save_csv").as_bool();
    csv_path_        = this->get_parameter("csv_path").as_string();
    
    // 创建订阅器
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos,
      std::bind(&PositionPrinter::odomCallback, this, std::placeholders::_1));
    
    // 创建定时器
    auto period = std::chrono::duration<double>(1.0 / print_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PositionPrinter::printPosition, this));
    
    // 打开 CSV 文件，并写入表头
    if (save_csv_) {
      csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
      if (csv_file_.is_open()) {
        csv_file_ << "timestamp,x,y,z,roll_deg,pitch_deg,yaw_deg\n";
        RCLCPP_INFO(this->get_logger(), "Saving CSV: %s",
          std::filesystem::absolute(csv_path_).c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Cannot open CSV: %s", csv_path_.c_str());
        save_csv_ = false;
      }
    }

    // 打印启动信息
    RCLCPP_INFO(this->get_logger(),
      "Subscribe %s, print %.1f Hz", odom_topic_.c_str(), print_rate_);

    std::cout << "==== Point-LIO Position Monitor ====\n"
              << "Press Ctrl+C to exit\n\n";
  }

  ~PositionPrinter()
  {
    if (csv_file_.is_open()) {
      csv_file_.close();
      RCLCPP_INFO(this->get_logger(), "CSV saved: %s", csv_path_.c_str());
    }
  }

private:
  // 里程计回调函数
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_ = *msg;
    has_data_ = true;

    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;

    double roll, pitch, yaw;
    quatToRPY(q.x, q.y, q.z, q.w, roll, pitch, yaw);

    if (save_csv_ && csv_file_.is_open()) {
      double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
      csv_file_ << std::fixed << std::setprecision(6)
                << t << "," << p.x << "," << p.y << "," << p.z << ","
                << roll << "," << pitch << "," << yaw << "\n";
    }
  }

  // 打印位置信息
  void printPosition()
  {
    if (!has_data_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for odometry on %s ...", odom_topic_.c_str());
      return;
    }

    const auto & p = latest_.pose.pose.position;
    const auto & q = latest_.pose.pose.orientation;

    double roll, pitch, yaw;
    quatToRPY(q.x, q.y, q.z, q.w, roll, pitch, yaw);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "X:" << std::setw(7) << p.x
        << "  Y:" << std::setw(7) << p.y
        << "  Z:" << std::setw(7) << p.z;

    if (show_orientation_) {
      oss << "  |  R:" << std::setw(7) << roll  << "°"
          << "  P:" << std::setw(7) << pitch << "°"
          << "  Y:" << std::setw(7) << yaw   << "°";
    }

    std::cout << oss.str() << std::endl;   
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

  std::string odom_topic_;
  double print_rate_;
  bool show_orientation_;

  std::ofstream csv_file_;
  std::string csv_path_;
  bool save_csv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PositionPrinter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}