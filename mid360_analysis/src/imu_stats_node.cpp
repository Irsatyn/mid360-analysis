#include <memory>
#include <string>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "mid360_analysis/imu_analysis.hpp"

namespace mid360_analysis {
class ImuStatsNode : public rclcpp::Node {
public:
  ImuStatsNode() : Node("imu_stats_node") {
    ImuConfig cfg;
    const int window = declare_parameter<int>("window_size", 200);
    publish_every_n_ = declare_parameter<int>("publish_every_n", 20);
    const double log_period = declare_parameter<double>("log_period_s", 2.);
    if (window <= 0 || publish_every_n_ <= 0) throw std::invalid_argument("window_size and publish_every_n must be positive");
    require_positive(log_period, "log_period_s must be positive");
    log_period_ms_ = static_cast<int64_t>(std::max(1., log_period * 1000.));
    cfg.window_size = static_cast<std::size_t>(window);
    cfg.nominal_rate_hz = declare_parameter<double>("nominal_rate_hz", 200.);
    cfg.drop_factor = declare_parameter<double>("drop_factor", 1.5);
    cfg.acc_unit_is_g = declare_parameter<bool>("acc_unit_is_g", true);
    cfg.static_acc_tol = declare_parameter<double>("static_acc_tol", 0.05);
    cfg.static_gyro_tol = declare_parameter<double>("static_gyro_tol", 0.02);
    cfg.static_std_tol = declare_parameter<double>("static_std_tol", 0.02);
    analysis_ = std::make_unique<ImuAnalysis>(cfg);
    rate_ = create_publisher<std_msgs::msg::Float64>("/analysis/imu/rate", 10);
    jitter_ = create_publisher<std_msgs::msg::Float64>("/analysis/imu/dt_jitter", 10);
    acc_norm_ = create_publisher<std_msgs::msg::Float64>("/analysis/imu/acc_norm", 10);
    static_ = create_publisher<std_msgs::msg::Bool>("/analysis/imu/is_static", 10);
    bias_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/analysis/imu/gyro_bias", 10);
    stats_ = create_publisher<std_msgs::msg::Float64MultiArray>("/analysis/imu/stats", 10);
    const auto topic = declare_parameter<std::string>("imu_topic", "/livox/imu");
    sub_ = create_subscription<sensor_msgs::msg::Imu>(topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { on_imu(*msg); });
    RCLCPP_INFO(get_logger(), "Listening on %s; acceleration input unit=%s", topic.c_str(), cfg.acc_unit_is_g ? "g" : "m/s^2");
  }
private:
  void scalar(const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &pub, double value) {
    std_msgs::msg::Float64 msg; msg.data = value; pub->publish(msg);
  }
  void on_imu(const sensor_msgs::msg::Imu &msg) {
    auto &a = *analysis_;
    if (!frame_.empty() && frame_ != msg.header.frame_id) {
      a.reset(); samples_ = 0;
      RCLCPP_WARN(get_logger(), "IMU frame changed; restarting statistics");
    }
    frame_ = msg.header.frame_id;
    if (!a.update(rclcpp::Time(msg.header.stamp).seconds(),
      {msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z},
      {msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z})) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid or duplicate IMU sample");
      return;
    }
    if (a.segment_reset) { samples_ = 0; RCLCPP_WARN(get_logger(), "IMU time moved backward; new analysis segment"); }
    if (a.is_static()) bias_header_ = msg.header;
    // Playback uses simulated time: this is not the original receive latency.
    const double latency = get_parameter("use_sim_time").as_bool() ? std::numeric_limits<double>::quiet_NaN() :
      (now() - rclcpp::Time(msg.header.stamp, RCL_ROS_TIME)).seconds();
    if (++samples_ % publish_every_n_ == 0) {
      scalar(rate_, a.rate()); scalar(jitter_, a.dt_stats.stddev()); scalar(acc_norm_, a.acc_norm.mean());
      std_msgs::msg::Bool state; state.data = a.is_static(); static_->publish(state);
      geometry_msgs::msg::Vector3Stamped bias;
      bias.header = a.bias_valid() ? bias_header_ : msg.header;
      const double invalid = std::numeric_limits<double>::quiet_NaN();
      bias.vector.x = a.bias_valid() ? a.bias()[0] : invalid;
      bias.vector.y = a.bias_valid() ? a.bias()[1] : invalid;
      bias.vector.z = a.bias_valid() ? a.bias()[2] : invalid;
      bias_->publish(bias);
      std_msgs::msg::Float64MultiArray stats;
      stats.data = {a.rate(), a.dt_stats.stddev(), static_cast<double>(a.drop_count), static_cast<double>(a.non_monotonic_count)};
      for (const auto &s : a.acc_axes) stats.data.push_back(s.mean());
      for (const auto &s : a.acc_axes) stats.data.push_back(s.stddev());
      for (const auto &s : a.gyro_axes) stats.data.push_back(s.mean());
      for (const auto &s : a.gyro_axes) stats.data.push_back(s.stddev());
      stats.data.push_back(latency);
      stats.layout.dim.resize(1); stats.layout.dim[0].label = "see README";
      stats.layout.dim[0].size = 17; stats.layout.dim[0].stride = 17;
      stats_->publish(stats);
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_period_ms_,
      "IMU rate=%.1fHz jitter=%.3fms gaps=%llu nonmonotonic=%llu static=%d |a|=%.4fg bias_valid=%d bias=(%.5f,%.5f,%.5f)",
      a.rate(), a.dt_stats.stddev()*1000., static_cast<unsigned long long>(a.drop_count),
      static_cast<unsigned long long>(a.non_monotonic_count), a.is_static(), a.acc_norm.mean(), a.bias_valid(),
      a.bias()[0], a.bias()[1], a.bias()[2]);
  }
  std::unique_ptr<ImuAnalysis> analysis_;
  int publish_every_n_{20};
  int64_t log_period_ms_{2000};
  std::uint64_t samples_{0};
  std::string frame_;
  std_msgs::msg::Header bias_header_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rate_, jitter_, acc_norm_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr static_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr bias_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stats_;
};
}  // namespace mid360_analysis
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<mid360_analysis::ImuStatsNode>()); }
  catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("imu_stats_node"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
