#include "figure8_path_publisher/figure8_path_publisher_node.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace figure8_path_publisher {

Figure8PathPublisherNode::Figure8PathPublisherNode(const rclcpp::NodeOptions & options)
: Node("figure8_path_publisher", options)
{
  // 声明参数、创建发布器并启动定时器
  // 发布开关
  this->declare_parameter<bool>("publish_to_controller", true);
  this->declare_parameter<bool>("publish_to_px4", false);
  this->declare_parameter<bool>("publish_path", false);
  this->declare_parameter<double>("publish_rate_hz", 50.0);

  // 话题
  this->declare_parameter<std::string>(
    "topics.controller_setpoint", "/tanh_controller/trajectory_setpoint");
  this->declare_parameter<std::string>("topics.px4_setpoint", "/fmu/in/trajectory_setpoint");
  this->declare_parameter<std::string>(
    "topics.px4_offboard_control_mode", "/fmu/in/offboard_control_mode");
  this->declare_parameter<std::string>("topics.px4_vehicle_command", "/fmu/in/vehicle_command");
  this->declare_parameter<std::string>("topics.path", "/figure8/path");
  this->declare_parameter<std::string>("path.frame_id", "map");
  this->declare_parameter<int>("path.max_points", 500);

  // PX4 offboard/arming（默认关闭，避免误操作）
  this->declare_parameter<bool>("px4.publish_offboard_control_mode", true);
  this->declare_parameter<bool>("px4.auto_offboard", false);
  this->declare_parameter<bool>("px4.auto_arm", false);
  this->declare_parameter<int>("px4.offboard_warmup", 10);

  // 轨迹参数（NED）
  this->declare_parameter<double>("trajectory.amplitude_x", 2.0);
  this->declare_parameter<double>("trajectory.amplitude_y", 1.0);
  this->declare_parameter<double>("trajectory.period_s", 20.0);
  this->declare_parameter<std::vector<double>>("trajectory.center", {0.0, 0.0, -2.0});
  this->declare_parameter<std::string>("trajectory.yaw_mode", "tangent");
  this->declare_parameter<double>("trajectory.fixed_yaw", 0.0);
  this->declare_parameter<double>("trajectory.yaw_offset", 0.0);

  publish_to_controller_ = this->get_parameter("publish_to_controller").as_bool();
  publish_to_px4_ = this->get_parameter("publish_to_px4").as_bool();
  publish_path_ = this->get_parameter("publish_path").as_bool();
  const double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();

  topic_controller_setpoint_ = this->get_parameter("topics.controller_setpoint").as_string();
  topic_px4_setpoint_ = this->get_parameter("topics.px4_setpoint").as_string();
  topic_px4_offboard_control_mode_ =
    this->get_parameter("topics.px4_offboard_control_mode").as_string();
  topic_px4_vehicle_command_ = this->get_parameter("topics.px4_vehicle_command").as_string();
  topic_path_ = this->get_parameter("topics.path").as_string();
  path_frame_id_ = this->get_parameter("path.frame_id").as_string();
  path_max_points_ = this->get_parameter("path.max_points").as_int();

  publish_px4_offboard_control_mode_ = this->get_parameter("px4.publish_offboard_control_mode").as_bool();
  auto_offboard_ = this->get_parameter("px4.auto_offboard").as_bool();
  auto_arm_ = this->get_parameter("px4.auto_arm").as_bool();
  offboard_warmup_ = this->get_parameter("px4.offboard_warmup").as_int();

  amplitude_x_ = this->get_parameter("trajectory.amplitude_x").as_double();
  amplitude_y_ = this->get_parameter("trajectory.amplitude_y").as_double();
  period_s_ = std::max(1e-3, this->get_parameter("trajectory.period_s").as_double());
  const auto center = this->get_parameter("trajectory.center").as_double_array();
  if (center.size() == 3) {
    center_x_ = center[0];
    center_y_ = center[1];
    center_z_ = center[2];
  }
  yaw_mode_ = this->get_parameter("trajectory.yaw_mode").as_string();
  fixed_yaw_ = this->get_parameter("trajectory.fixed_yaw").as_double();
  yaw_offset_ = this->get_parameter("trajectory.yaw_offset").as_double();
  last_yaw_ = fixed_yaw_;

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
  if (publish_to_controller_) {
    controller_setpoint_pub_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(topic_controller_setpoint_, qos);
  }
  if (publish_to_px4_) {
    px4_setpoint_pub_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(topic_px4_setpoint_, qos);
    px4_offboard_mode_pub_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(topic_px4_offboard_control_mode_, qos);
    px4_vehicle_cmd_pub_ =
      this->create_publisher<px4_msgs::msg::VehicleCommand>(topic_px4_vehicle_command_, qos);
  }
  if (publish_path_) {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(topic_path_, rclcpp::QoS(1).reliable());
    path_msg_.header.frame_id = path_frame_id_;
  }

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz));
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&Figure8PathPublisherNode::onTimer, this));
}

double Figure8PathPublisherNode::wrapPi(double rad)
{
  // 将角度归一化到[-pi, pi]
  while (rad > M_PI) {
    rad -= 2.0 * M_PI;
  }
  while (rad < -M_PI) {
    rad += 2.0 * M_PI;
  }
  return rad;
}

void Figure8PathPublisherNode::publishVehicleCommand(uint32_t command, float param1, float param2, float param3)
{
  // 发布VehicleCommand，用于切换offboard/解锁等
  if (!px4_vehicle_cmd_pub_) {
    return;
  }

  px4_msgs::msg::VehicleCommand cmd{};
  cmd.timestamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL);
  cmd.param1 = param1;
  cmd.param2 = param2;
  cmd.param3 = param3;
  cmd.command = command;
  cmd.target_system = 1;
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;
  px4_vehicle_cmd_pub_->publish(cmd);
}

void Figure8PathPublisherNode::onTimer()
{
  // 生成8字轨迹点，并按配置发布到对应话题
  const rclcpp::Time now = this->get_clock()->now();
  if (!started_) {
    started_ = true;
    start_time_ = now;
  }

  const double t = (now - start_time_).seconds();
  const double w = 2.0 * M_PI / period_s_;

  // 8字形：x = a*sin(wt), y = b*sin(wt)*cos(wt) = 0.5*b*sin(2wt)
  const double s1 = std::sin(w * t);
  const double c1 = std::cos(w * t);
  const double s2 = std::sin(2.0 * w * t);
  const double c2 = std::cos(2.0 * w * t);

  const double x = center_x_ + amplitude_x_ * s1;
  const double y = center_y_ + amplitude_y_ * s1 * c1;
  const double z = center_z_;

  const double vx = amplitude_x_ * w * c1;
  const double vy = amplitude_y_ * w * c2;
  const double vz = 0.0;

  const double ax = -amplitude_x_ * w * w * s1;
  const double ay = -2.0 * amplitude_y_ * w * w * s2;
  const double az = 0.0;

  double yaw = fixed_yaw_;
  if (yaw_mode_ == "tangent") {
    const double v_norm = std::hypot(vx, vy);
    if (v_norm > 1e-3) {
      yaw = std::atan2(vy, vx) + yaw_offset_;
      yaw = wrapPi(yaw);
      last_yaw_ = yaw;
    } else {
      yaw = last_yaw_;
    }
  } else if (yaw_mode_ == "fixed") {
    yaw = wrapPi(fixed_yaw_ + yaw_offset_);
    last_yaw_ = yaw;
  }

  const uint64_t now_us = static_cast<uint64_t>(now.nanoseconds() / 1000ULL);
  px4_msgs::msg::TrajectorySetpoint sp{};
  sp.timestamp = now_us;
  sp.position = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
  sp.velocity = {static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vz)};
  sp.acceleration = {static_cast<float>(ax), static_cast<float>(ay), static_cast<float>(az)};
  sp.jerk = {0.0f, 0.0f, 0.0f};
  sp.yaw = static_cast<float>(yaw);
  sp.yawspeed = 0.0f;

  if (publish_to_controller_ && controller_setpoint_pub_) {
    controller_setpoint_pub_->publish(sp);
  }

  if (publish_to_px4_ && px4_setpoint_pub_) {
    if (publish_px4_offboard_control_mode_ && px4_offboard_mode_pub_) {
      px4_msgs::msg::OffboardControlMode mode{};
      mode.timestamp = now_us;
      mode.position = true;
      mode.velocity = false;
      mode.acceleration = false;
      mode.attitude = false;
      mode.body_rate = false;
      mode.thrust_and_torque = false;
      mode.direct_actuator = false;
      px4_offboard_mode_pub_->publish(mode);
    }

    // 可选：自动切offboard/解锁（默认关闭）
    if ((auto_offboard_ || auto_arm_) && offboard_counter_ <= offboard_warmup_) {
      offboard_counter_++;
      if (offboard_counter_ == offboard_warmup_) {
        if (auto_offboard_) {
          publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f, 0.0f);
        }
        if (auto_arm_) {
          publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f, 0.0f);
        }
      }
    }

    px4_setpoint_pub_->publish(sp);
  }

  if (publish_path_ && path_pub_) {
    geometry_msgs::msg::PoseStamped pose{};
    pose.header.stamp = now;
    pose.header.frame_id = path_frame_id_;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);

    path_msg_.header.stamp = now;
    path_msg_.poses.push_back(pose);
    if (static_cast<int>(path_msg_.poses.size()) > path_max_points_) {
      path_msg_.poses.erase(path_msg_.poses.begin());
    }
    path_pub_->publish(path_msg_);
  }
}

}  // namespace figure8_path_publisher

int main(int argc, char ** argv)
{
  // ROS2主入口
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<figure8_path_publisher::Figure8PathPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
