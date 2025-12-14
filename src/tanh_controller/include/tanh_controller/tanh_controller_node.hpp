#pragma once

#include <rclcpp/rclcpp.hpp>

#include <array>

#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>

#include "tanh_controller/tanh_controller.hpp"

namespace tanh_controller {

/**
 * @brief ROS2节点：订阅PX4状态与轨迹参考，发布电机控制量（/fmu/in/actuator_motors）
 */
class TanhControllerNode : public rclcpp::Node
{
public:
  /** @brief 构造函数 */
  explicit TanhControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /** @brief PX4里程计回调：更新状态与dt */
  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

  /** @brief 轨迹参考回调：更新期望位置/航向 */
  void setpointCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg);

  /** @brief 定时控制循环：计算并发布ActuatorMotors/OffboardControlMode */
  void controlLoop();

  /** @brief 发送PX4 VehicleCommand（可用于解锁/切offboard） */
  void publishVehicleCommand(uint32_t command, float param1, float param2, float param3);

  /** @brief 读取参数并配置控制器 */
  void loadParams();

  /** @brief 将vector参数读成Eigen::Vector3d（长度必须为3） */
  static Eigen::Vector3d getVec3Param(rclcpp::Node & node, const std::string & name);

private:
  // ROS接口
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_sub_;

  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_sp_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // 数据缓存
  VehicleState state_{};
  TrajectoryRef ref_{};
  bool has_state_{false};
  bool has_ref_{false};
  uint64_t last_odom_us_{0};
  double last_odom_dt_{0.01};
  uint64_t last_control_us_{0};

  // 控制器
  TanhController controller_{};

  // 参数（话题/行为）
  std::string topic_vehicle_odometry_;
  std::string topic_trajectory_setpoint_;
  std::string topic_actuator_motors_;
  std::string topic_offboard_control_mode_;
  std::string topic_vehicle_command_;
  std::string topic_vehicle_thrust_setpoint_;

  double control_rate_hz_{100.0};
  bool publish_offboard_control_mode_{true};
  bool publish_vehicle_thrust_setpoint_{true};
  double motor_force_max_{8.54858};

  // 电机输出顺序映射：out[i] = internal[motor_output_map_[i]]
  // internal顺序来自G1列顺序（默认：前左、前右、后右、后左）
  std::array<int, 4> motor_output_map_{{1, 3, 0, 2}};

  bool enable_auto_offboard_{false};
  bool enable_auto_arm_{false};
  int offboard_setpoint_warmup_{10};
  int offboard_counter_{0};
};

}  // namespace tanh_controller
