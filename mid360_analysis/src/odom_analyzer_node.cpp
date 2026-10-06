#include <algorithm>
#include <chrono>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "mid360_analysis/odom_analysis.hpp"

namespace mid360_analysis {
class OdomAnalyzerNode : public rclcpp::Node {
  using Marker = visualization_msgs::msg::Marker;
  using Steady = std::chrono::steady_clock;
  struct Pending { nav_msgs::msg::Odometry msg; Steady::time_point deadline; };
public:
  OdomAnalyzerNode() : Node("odom_analyzer_node") {
    world_ = declare_parameter<std::string>("world_frame", "camera_init");
    body_ = declare_parameter<std::string>("body_frame", "body");
    if (world_.empty() || body_.empty() || world_ == body_) throw std::invalid_argument("world_frame and body_frame must be distinct nonempty frames");
    analysis_ = std::make_unique<OdomAnalysis>(OdomConfig{declare_parameter<double>("min_step_m", 0.005)});
    marker_every_ = declare_parameter<double>("marker_every_m", 1.);
    max_color_speed_ = declare_parameter<double>("max_color_speed", 1.5);
    tf_wait_ = declare_parameter<double>("tf_wait_s", 0.2);
    const double log_period = declare_parameter<double>("log_period_s", 2.);
    require_positive(marker_every_, "marker_every_m must be positive");
    require_positive(max_color_speed_, "max_color_speed must be positive");
    require_positive(tf_wait_, "tf_wait_s must be positive");
    require_positive(log_period, "log_period_s must be positive");
    log_period_ms_ = static_cast<int64_t>(std::max(1., log_period*1000.));
    max_points_ = declare_parameter<int>("max_trajectory_points", 5000);
    max_labels_ = declare_parameter<int>("max_distance_markers", 200);
    if (max_points_ < 2 || max_labels_ < 1) throw std::invalid_argument("max_trajectory_points >= 2 and max_distance_markers >= 1 required");
    next_marker_distance_ = marker_every_;
    distance_ = create_publisher<std_msgs::msg::Float64>("/analysis/odom/distance", 10);
    speed_ = create_publisher<std_msgs::msg::Float64>("/analysis/odom/speed", 10);
    displacement_ = create_publisher<std_msgs::msg::Float64>("/analysis/odom/displacement", 10);
    velocity_ = create_publisher<geometry_msgs::msg::TwistStamped>("/analysis/odom/velocity", 10);
    rpy_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/analysis/odom/rpy", 10);
    tf_error_ = create_publisher<std_msgs::msg::Float64MultiArray>("/analysis/odom/tf_error", 10);
    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("/analysis/odom/markers", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // The listener has its own executor thread. The subscription callback never waits for TF.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { check_pending_tf(); });
    const auto topic = declare_parameter<std::string>("odom_topic", "/Odometry");
    sub_ = create_subscription<nav_msgs::msg::Odometry>(topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_odom(*msg); });
    RCLCPP_INFO(get_logger(), "Listening on %s; TF %s -> %s", topic.c_str(), world_.c_str(), body_.c_str());
  }
private:
  static bool valid_quaternion(const geometry_msgs::msg::Quaternion &q) {
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
      std::isfinite(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w) && q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w > 1e-12;
  }
  static tf2::Quaternion quaternion(const geometry_msgs::msg::Quaternion &q) {
    tf2::Quaternion result(q.x, q.y, q.z, q.w); result.normalize(); return result;
  }
  void scalar(const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &pub, double value) {
    std_msgs::msg::Float64 msg; msg.data = value; pub->publish(msg);
  }
  void on_odom(const nav_msgs::msg::Odometry &input) {
    if (input.header.frame_id != world_ || input.child_frame_id != body_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring Odometry frame mismatch: %s -> %s", input.header.frame_id.c_str(), input.child_frame_id.c_str());
      return;
    }
    if (!valid_quaternion(input.pose.pose.orientation)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid Odometry quaternion"); return;
    }
    auto &a = *analysis_;
    const auto &p = input.pose.pose.position;
    if (!a.update(rclcpp::Time(input.header.stamp).seconds(), {p.x,p.y,p.z})) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid or duplicate Odometry sample"); return;
    }
    if (a.segment_reset) {
      trajectory_.points.clear(); trajectory_.colors.clear(); labels_.clear(); pending_.clear();
      label_id_ = 0; next_marker_distance_ = marker_every_;
      RCLCPP_WARN(get_logger(), "Odometry time moved backward; new trajectory segment");
    }
    scalar(distance_, a.distance); scalar(speed_, a.speed); scalar(displacement_, a.displacement);
    geometry_msgs::msg::TwistStamped velocity;
    velocity.header = input.header;
    velocity.twist.linear.x = a.velocity[0]; velocity.twist.linear.y = a.velocity[1]; velocity.twist.linear.z = a.velocity[2];
    velocity_->publish(velocity);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion(input.pose.pose.orientation)).getRPY(roll,pitch,yaw);
    geometry_msgs::msg::Vector3Stamped rpy;
    rpy.header = input.header; rpy.vector.x = roll*degrees_; rpy.vector.y = pitch*degrees_; rpy.vector.z = yaw*degrees_;
    rpy_->publish(rpy);
    publish_markers(input, yaw*degrees_, a.segment_reset);
    if (pending_.size() >= 100) { pending_.pop_front(); publish_tf_failure(); }
    pending_.push_back(Pending{input, Steady::now() + std::chrono::duration_cast<Steady::duration>(std::chrono::duration<double>(tf_wait_))});
    check_pending_tf();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_period_ms_,
      "Odom rate=%.1fHz distance=%.3fm speed=%.3fm/s max=%.3fm/s displacement=%.3fm yaw=%.1fdeg tf_fail=%llu",
      a.rate(), a.distance, a.speed, a.max_speed, a.displacement, yaw*degrees_, static_cast<unsigned long long>(tf_fail_count_));
  }
  void publish_tf_result(double pos, double rot) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {pos,rot,static_cast<double>(tf_fail_count_)};
    msg.layout.dim.resize(1); msg.layout.dim[0].label = "pos_err_m,rot_err_deg,tf_fail_count";
    msg.layout.dim[0].size = 3; msg.layout.dim[0].stride = 3;
    tf_error_->publish(msg);
  }
  void publish_tf_failure() {
    ++tf_fail_count_;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    publish_tf_result(nan,nan);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF unavailable at Odometry stamp; failures=%llu", static_cast<unsigned long long>(tf_fail_count_));
  }
  void check_pending_tf() {
    for (auto it = pending_.begin(); it != pending_.end();) {
      bool ready = false;
      try {
        const auto tf = tf_buffer_->lookupTransform(world_, body_, rclcpp::Time(it->msg.header.stamp), rclcpp::Duration::from_seconds(0.));
        const auto &p = it->msg.pose.pose.position;
        const auto &t = tf.transform.translation;
        if (!valid_quaternion(tf.transform.rotation) || !finite({t.x,t.y,t.z})) {
          publish_tf_failure();
        } else {
          const double dot = std::clamp(std::abs(quaternion(tf.transform.rotation).dot(quaternion(it->msg.pose.pose.orientation))), 0., 1.);
          publish_tf_result(norm({t.x-p.x,t.y-p.y,t.z-p.z}), 2.*std::acos(dot)*degrees_);
        }
        ready = true;
      } catch (const tf2::TransformException &) {
        if (Steady::now() >= it->deadline) { publish_tf_failure(); ready = true; }
      }
      if (ready) it = pending_.erase(it); else ++it;
    }
  }
  Marker base_marker(const std_msgs::msg::Header &header, const std::string &ns, int id, int type) {
    Marker m; m.header = header; m.ns = ns; m.id = id; m.type = type; m.action = Marker::ADD;
    m.pose.orientation.w = 1.; m.color.a = 1.f; return m;
  }
  void publish_markers(const nav_msgs::msg::Odometry &msg, double yaw, bool reset) {
    auto &a = *analysis_;
    visualization_msgs::msg::MarkerArray array;
    if (reset) { Marker clear; clear.action = Marker::DELETEALL; array.markers.push_back(clear); }
    trajectory_.header = msg.header; trajectory_.ns = "trajectory"; trajectory_.id = 0;
    trajectory_.type = Marker::LINE_STRIP; trajectory_.action = Marker::ADD;
    trajectory_.pose.orientation.w = 1.; trajectory_.scale.x = 0.04; trajectory_.color.a = 1.f;
    trajectory_.points.push_back(msg.pose.pose.position);
    std_msgs::msg::ColorRGBA color;
    color.r = static_cast<float>(std::clamp(a.speed/max_color_speed_, 0., 1.));
    color.b = 1.f-color.r; color.a = 1.f; trajectory_.colors.push_back(color);
    if (trajectory_.points.size() > static_cast<std::size_t>(max_points_)) {
      const auto last_point = trajectory_.points.back(); const auto last_color = trajectory_.colors.back();
      std::size_t write = 0;
      for (std::size_t read=0; read+1 < trajectory_.points.size(); read+=2) {
        trajectory_.points[write] = trajectory_.points[read]; trajectory_.colors[write++] = trajectory_.colors[read];
      }
      trajectory_.points.resize(write); trajectory_.colors.resize(write);
      trajectory_.points.push_back(last_point); trajectory_.colors.push_back(last_color);
    }
    array.markers.push_back(trajectory_);
    if (a.distance >= next_marker_distance_) {
      label_id_ = label_id_ % max_labels_ + 1;
      auto label = base_marker(msg.header, "distance", label_id_, Marker::TEXT_VIEW_FACING);
      label.pose.position = msg.pose.pose.position; label.pose.position.z += 0.3;
      label.scale.z = 0.2; label.color.r = label.color.g = label.color.b = 1.f;
      std::ostringstream text; text << std::fixed << std::setprecision(1) << a.distance << " m"; label.text = text.str();
      if (labels_.size() == static_cast<std::size_t>(max_labels_)) labels_.pop_front();
      labels_.push_back(label);
      next_marker_distance_ = (std::floor(a.distance/marker_every_)+1.)*marker_every_;
    }
    for (auto &label : labels_) { label.header.stamp = msg.header.stamp; array.markers.push_back(label); }
    auto status = base_marker(msg.header, "status", 1000, Marker::TEXT_VIEW_FACING);
    status.pose.position = msg.pose.pose.position; status.pose.position.z += 0.65;
    status.scale.z = 0.25; status.color.g = 1.f;
    std::ostringstream text;
    text << std::fixed << std::setprecision(2) << "d=" << a.distance << "m v=" << a.speed << "m/s yaw=" << std::setprecision(1) << yaw << "deg";
    status.text = text.str(); array.markers.push_back(status); markers_->publish(array);
  }
  static constexpr double degrees_ = 57.29577951308232;
  std::unique_ptr<OdomAnalysis> analysis_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  std::string world_, body_;
  double marker_every_{1.}, max_color_speed_{1.5}, tf_wait_{0.2}, next_marker_distance_{1.};
  int max_points_{5000}, max_labels_{200}, label_id_{0};
  int64_t log_period_ms_{2000};
  std::uint64_t tf_fail_count_{0};
  Marker trajectory_;
  std::deque<Marker> labels_;
  std::deque<Pending> pending_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_, speed_, displacement_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tf_error_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
};
}  // namespace mid360_analysis
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<mid360_analysis::OdomAnalyzerNode>()); }
  catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("odom_analyzer_node"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
