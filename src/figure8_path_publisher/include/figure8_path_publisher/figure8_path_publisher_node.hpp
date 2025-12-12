#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

namespace figure8_path_publisher {

/**
 * @brief ROS2节点：生成8字形轨迹并发布TrajectorySetpoint（可选发布给PX4和自定义控制器）
 */
class Figure8PathPublisherNode : public rclcpp::Node
{
public:
  /** @brief 构造函数 */
  explicit Figure8PathPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /** @brief 定时发布：计算8字轨迹并发布消息 */
  void onTimer();

  /** @brief 发送PX4 VehicleCommand（可用于解锁/切offboard） */
  void publishVehicleCommand(uint32_t command, float param1, float param2, float param3);

  /** @brief 将角度归一化到[-pi, pi] */
  static double wrapPi(double rad);

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr controller_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr px4_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr px4_offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr px4_vehicle_cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  rclcpp::Time start_time_;
  bool started_{false};

  // 参数：发布开关/话题
  bool publish_to_controller_{true};
  bool publish_to_px4_{false};
  bool publish_path_{false};

  std::string topic_controller_setpoint_;
  std::string topic_px4_setpoint_;
  std::string topic_px4_offboard_control_mode_;
  std::string topic_px4_vehicle_command_;
  std::string topic_path_;
  std::string path_frame_id_;

  // 参数：PX4 offboard/arming
  bool publish_px4_offboard_control_mode_{true};
  bool auto_offboard_{false};
  bool auto_arm_{false};
  int offboard_warmup_{10};
  int offboard_counter_{0};

  // 参数：轨迹
  double amplitude_x_{2.0};
  double amplitude_y_{1.0};
  double period_s_{20.0};
  double center_x_{0.0};
  double center_y_{0.0};
  double center_z_{-2.0};  // NED: 上升为负

  std::string yaw_mode_{"tangent"};  // tangent/fixed
  double fixed_yaw_{0.0};
  double yaw_offset_{0.0};
  double last_yaw_{0.0};

  // Path缓存
  nav_msgs::msg::Path path_msg_;
  int path_max_points_{500};
};

}  // namespace figure8_path_publisher

