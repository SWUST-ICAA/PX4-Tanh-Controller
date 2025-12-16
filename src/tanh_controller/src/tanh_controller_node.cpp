#include "tanh_controller/tanh_controller_node.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace tanh_controller {

TanhControllerNode::TanhControllerNode(const rclcpp::NodeOptions & options)
: Node("tanh_controller", options)
{
  // 声明参数、创建ROS接口并启动控制定时器
  // 话题参数
  this->declare_parameter<std::string>("topics.vehicle_odometry", "/fmu/out/vehicle_odometry");
  this->declare_parameter<std::string>(
    "topics.trajectory_setpoint", "/tanh_controller/trajectory_setpoint");
  this->declare_parameter<std::string>("topics.actuator_motors", "/fmu/in/actuator_motors");
  this->declare_parameter<std::string>(
    "topics.offboard_control_mode", "/fmu/in/offboard_control_mode");
  this->declare_parameter<std::string>("topics.vehicle_command", "/fmu/in/vehicle_command");
  this->declare_parameter<std::string>(
    "topics.vehicle_thrust_setpoint", "/fmu/in/vehicle_thrust_setpoint");

  // 行为参数
  this->declare_parameter<double>("control_rate_hz", 100.0);
  this->declare_parameter<bool>("publish_offboard_control_mode", true);
  this->declare_parameter<bool>("publish_vehicle_thrust_setpoint", true);
  this->declare_parameter<bool>("auto_offboard", false);
  this->declare_parameter<bool>("auto_arm", false);
  this->declare_parameter<int>("offboard_warmup", 10);

  // 模型参数
  this->declare_parameter<double>("model.mass", 2.0643076923076915);
  this->declare_parameter<double>("model.gravity", 9.81);
  this->declare_parameter<std::vector<double>>(
    "model.inertia_diag", {0.02384669, 0.02394962, 0.04399995});

  // 位置环参数
  this->declare_parameter<std::vector<double>>("position.M_P", {1.0, 1.0, 1.0});
  this->declare_parameter<std::vector<double>>("position.K_P", {1.5, 1.5, 1.5});
  this->declare_parameter<std::vector<double>>("position.M_V", {3.0, 3.0, 3.0});
  this->declare_parameter<std::vector<double>>("position.K_V", {1.0, 1.0, 1.0});
  this->declare_parameter<std::vector<double>>("position.K_Acceleration", {0.0, 0.0, 0.0});
  this->declare_parameter<double>("position.max_tilt_deg", 35.0);
  this->declare_parameter<std::vector<double>>("position.observer.P_V", {0.0, 0.0, 0.0});
  this->declare_parameter<std::vector<double>>("position.observer.L_V", {5.0, 5.0, 5.0});

  // 姿态环参数
  this->declare_parameter<std::vector<double>>("attitude.M_Angle", {3.0, 3.0, 3.0});
  this->declare_parameter<std::vector<double>>("attitude.K_Angle", {4.0, 4.0, 4.0});
  this->declare_parameter<std::vector<double>>(
    "attitude.M_AngularVelocity", {20.0, 20.0, 15.0});
  this->declare_parameter<std::vector<double>>(
    "attitude.K_AngularVelocity", {2.0, 2.0, 2.0});
  this->declare_parameter<std::vector<double>>(
    "attitude.K_AngularAcceleration", {0.0, 0.0, 0.0});
  this->declare_parameter<std::vector<double>>(
    "attitude.observer.P_AngularVelocity", {0.0, 0.0, 0.0});
  this->declare_parameter<std::vector<double>>(
    "attitude.observer.L_AngularVelocity", {5.0, 5.0, 5.0});

  // 滤波参数（对差分得到的加速度/角加速度做低通，抑制高频噪声导致的震荡）
  this->declare_parameter<double>("filters.linear_accel_cutoff_hz", 0.0);
  this->declare_parameter<double>("filters.angular_accel_cutoff_hz", 0.0);
  // 滤波参数（对扰动观测器输出μ做低通，降低估计尖峰引起的抖动）
  this->declare_parameter<double>("filters.velocity_disturbance_cutoff_hz", 0.0);
  this->declare_parameter<double>("filters.angular_velocity_disturbance_cutoff_hz", 0.0);

  // 控制分配/电机映射参数
  this->declare_parameter<double>("allocation.l", 0.246073);
  this->declare_parameter<double>("allocation.beta", M_PI_4);
  this->declare_parameter<double>("allocation.cq_ct", 0.016);
  this->declare_parameter<double>("motor.force_max", 8.54858);
  this->declare_parameter<std::vector<int64_t>>("motor.output_map", {1, 3, 0, 2});

  loadParams();

  // 订阅PX4发布(/fmu/out/...)的话题必须使用兼容QoS，否则可能收不到数据
  const auto qos_px4_out = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

  // 发布到PX4订阅(/fmu/in/...)的话题不需要特殊QoS（默认QoS与PX4兼容）
  const auto qos_default = rclcpp::QoS(rclcpp::KeepLast(10));

  odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    topic_vehicle_odometry_, qos_px4_out,
    std::bind(&TanhControllerNode::odomCallback, this, std::placeholders::_1));

  setpoint_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
    topic_trajectory_setpoint_, qos_default,
    std::bind(&TanhControllerNode::setpointCallback, this, std::placeholders::_1));

  motors_pub_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>(topic_actuator_motors_, qos_default);
  offboard_mode_pub_ =
    this->create_publisher<px4_msgs::msg::OffboardControlMode>(topic_offboard_control_mode_, qos_default);
  vehicle_command_pub_ =
    this->create_publisher<px4_msgs::msg::VehicleCommand>(topic_vehicle_command_, qos_default);
  thrust_sp_pub_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
    topic_vehicle_thrust_setpoint_, qos_default);

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_hz_));
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&TanhControllerNode::controlLoop, this));
}

void TanhControllerNode::loadParams()
{
  // 从参数服务器读取参数并配置控制器
  topic_vehicle_odometry_ = this->get_parameter("topics.vehicle_odometry").as_string();
  topic_trajectory_setpoint_ = this->get_parameter("topics.trajectory_setpoint").as_string();
  topic_actuator_motors_ = this->get_parameter("topics.actuator_motors").as_string();
  topic_offboard_control_mode_ = this->get_parameter("topics.offboard_control_mode").as_string();
  topic_vehicle_command_ = this->get_parameter("topics.vehicle_command").as_string();
  topic_vehicle_thrust_setpoint_ = this->get_parameter("topics.vehicle_thrust_setpoint").as_string();

  control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
  publish_offboard_control_mode_ = this->get_parameter("publish_offboard_control_mode").as_bool();
  publish_vehicle_thrust_setpoint_ = this->get_parameter("publish_vehicle_thrust_setpoint").as_bool();
  enable_auto_offboard_ = this->get_parameter("auto_offboard").as_bool();
  enable_auto_arm_ = this->get_parameter("auto_arm").as_bool();
  offboard_setpoint_warmup_ = this->get_parameter("offboard_warmup").as_int();

  controller_.setMass(this->get_parameter("model.mass").as_double());
  controller_.setGravity(this->get_parameter("model.gravity").as_double());

  {
    const auto v = this->get_parameter("model.inertia_diag").as_double_array();
    if (v.size() == 3) {
      Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
      I(0, 0) = v[0];
      I(1, 1) = v[1];
      I(2, 2) = v[2];
      controller_.setInertia(I);
    } else {
      RCLCPP_WARN(this->get_logger(), "model.inertia_diag长度不是3，使用单位阵");
      controller_.setInertia(Eigen::Matrix3d::Identity());
    }
  }

  PositionGains pg;
  pg.M_P = getVec3Param(*this, "position.M_P");
  pg.K_P = getVec3Param(*this, "position.K_P");
  pg.M_V = getVec3Param(*this, "position.M_V");
  pg.K_V = getVec3Param(*this, "position.K_V");
  pg.K_Acceleration = getVec3Param(*this, "position.K_Acceleration");
  pg.P_V = getVec3Param(*this, "position.observer.P_V");
  pg.L_V = getVec3Param(*this, "position.observer.L_V");
  controller_.setPositionGains(pg);

  {
    const double max_tilt_deg = this->get_parameter("position.max_tilt_deg").as_double();
    controller_.setMaxTilt(max_tilt_deg * M_PI / 180.0);
  }

  AttitudeGains ag;
  ag.M_Angle = getVec3Param(*this, "attitude.M_Angle");
  ag.K_Angle = getVec3Param(*this, "attitude.K_Angle");
  ag.M_AngularVelocity = getVec3Param(*this, "attitude.M_AngularVelocity");
  ag.K_AngularVelocity = getVec3Param(*this, "attitude.K_AngularVelocity");
  ag.K_AngularAcceleration = getVec3Param(*this, "attitude.K_AngularAcceleration");
  ag.P_AngularVelocity = getVec3Param(*this, "attitude.observer.P_AngularVelocity");
  ag.L_AngularVelocity = getVec3Param(*this, "attitude.observer.L_AngularVelocity");
  controller_.setAttitudeGains(ag);

  controller_.setLinearAccelerationLowPassHz(
    this->get_parameter("filters.linear_accel_cutoff_hz").as_double());
  controller_.setAngularAccelerationLowPassHz(
    this->get_parameter("filters.angular_accel_cutoff_hz").as_double());
  controller_.setVelocityDisturbanceLowPassHz(
    this->get_parameter("filters.velocity_disturbance_cutoff_hz").as_double());
  controller_.setAngularVelocityDisturbanceLowPassHz(
    this->get_parameter("filters.angular_velocity_disturbance_cutoff_hz").as_double());

  AllocationParams ap;
  ap.l = this->get_parameter("allocation.l").as_double();
  ap.beta = this->get_parameter("allocation.beta").as_double();
  ap.cq_ct = this->get_parameter("allocation.cq_ct").as_double();
  controller_.setAllocationParams(ap);

  motor_force_max_ = this->get_parameter("motor.force_max").as_double();
  controller_.setMotorForceMax(motor_force_max_);

  {
    const auto map = this->get_parameter("motor.output_map").as_integer_array();
    if (map.size() == 4) {
      bool ok = true;
      std::array<bool, 4> used{{false, false, false, false}};
      for (size_t i = 0; i < 4; ++i) {
        const int idx = static_cast<int>(map[i]);
        if (idx < 0 || idx >= 4) {
          ok = false;
          break;
        }
        if (used[idx]) {
          ok = false;
          break;
        }
        used[idx] = true;
        motor_output_map_[i] = idx;
      }
      if (!ok) {
        RCLCPP_WARN(this->get_logger(), "motor.output_map无效，回退为默认映射");
        motor_output_map_ = {{1, 3, 0, 2}};
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "motor.output_map长度不是4，回退为默认映射");
      motor_output_map_ = {{1, 3, 0, 2}};
    }
  }
}

Eigen::Vector3d TanhControllerNode::getVec3Param(rclcpp::Node & node, const std::string & name)
{
  // 读取长度为3的double数组参数
  const auto v = node.get_parameter(name).as_double_array();
  if (v.size() != 3) {
    throw std::runtime_error("参数" + name + "长度必须为3");
  }
  return Eigen::Vector3d(v[0], v[1], v[2]);
}

void TanhControllerNode::odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  // 接收PX4里程计，更新状态与采样周期
  if (!msg) {
    return;
  }

  // 基本有效性检查：避免NaN传播导致控制输出全为0
  const bool pos_ok =
    std::isfinite(msg->position[0]) && std::isfinite(msg->position[1]) && std::isfinite(msg->position[2]);
  const bool q_ok =
    std::isfinite(msg->q[0]) && std::isfinite(msg->q[1]) && std::isfinite(msg->q[2]) && std::isfinite(msg->q[3]);

  if (!pos_ok || !q_ok) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "vehicle_odometry包含无效数据(position/q为NaN)，等待估计器就绪...");
    return;
  }

  // 计算里程计dt（PX4时间戳单位us），仅用于诊断/观测
  if (last_odom_us_ != 0 && msg->timestamp > last_odom_us_) {
    const double dt = static_cast<double>(msg->timestamp - last_odom_us_) * 1e-6;
    last_odom_dt_ = std::clamp(dt, 1e-4, 0.1);
  }
  last_odom_us_ = msg->timestamp;

  if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "vehicle_odometry.pose_frame不是NED(%u)，当前控制器按NED解释，可能导致不稳定",
      static_cast<unsigned>(msg->pose_frame));
  }

  Eigen::Quaterniond q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
  q.normalize();
  state_.q_body_to_ned = q;

  state_.position_ned = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
  {
    const double wx = std::isfinite(msg->angular_velocity[0]) ? msg->angular_velocity[0] : 0.0;
    const double wy = std::isfinite(msg->angular_velocity[1]) ? msg->angular_velocity[1] : 0.0;
    const double wz = std::isfinite(msg->angular_velocity[2]) ? msg->angular_velocity[2] : 0.0;
    state_.angular_velocity_body = Eigen::Vector3d(wx, wy, wz);
  }

  const double vx = std::isfinite(msg->velocity[0]) ? msg->velocity[0] : 0.0;
  const double vy = std::isfinite(msg->velocity[1]) ? msg->velocity[1] : 0.0;
  const double vz = std::isfinite(msg->velocity[2]) ? msg->velocity[2] : 0.0;
  const Eigen::Vector3d v_raw(vx, vy, vz);
  if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED) {
    state_.velocity_ned = v_raw;
  } else if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
    // 速度在机体系(FRD)，旋转到NED
    state_.velocity_ned = q * v_raw;
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "vehicle_odometry.velocity_frame=%u未处理，当前直接按NED使用，可能导致不稳定",
      static_cast<unsigned>(msg->velocity_frame));
    state_.velocity_ned = v_raw;
  }

  has_state_ = true;
}

void TanhControllerNode::setpointCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
{
  // 接收期望轨迹点：控制器位置环使用 xi_d（期望位置）与 yaw_d（期望航向角）来构造推力方向与期望姿态
  if (!msg) {
    return;
  }

  const float x = msg->position[0];
  const float y = msg->position[1];
  const float z = msg->position[2];
  const float yaw = msg->yaw;

  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    ref_.valid = false;
    has_ref_ = false;
    return;
  }

  ref_.position_ned = Eigen::Vector3d(x, y, z);
  ref_.yaw = std::isfinite(yaw) ? static_cast<double>(yaw) : 0.0;

  ref_.valid = true;
  has_ref_ = true;
}

void TanhControllerNode::publishVehicleCommand(uint32_t command, float param1, float param2, float param3)
{
  // 发布VehicleCommand，用于切换offboard/解锁等
  if (!vehicle_command_pub_) {
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
  vehicle_command_pub_->publish(cmd);
}

void TanhControllerNode::controlLoop()
{
  // 周期控制循环：发布offboard模式并输出电机控制
  const uint64_t now_us = static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL);
  double dt = 1.0 / std::max(1.0, control_rate_hz_);
  if (last_control_us_ != 0 && now_us > last_control_us_) {
    dt = static_cast<double>(now_us - last_control_us_) * 1e-6;
  }
  last_control_us_ = now_us;
  dt = std::clamp(dt, 1e-4, 0.1);

  if (publish_offboard_control_mode_ && offboard_mode_pub_) {
    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = now_us;
    mode.position = false;
    mode.velocity = false;
    mode.acceleration = false;
    mode.attitude = false;
    mode.body_rate = false;
    mode.thrust_and_torque = false;
    mode.direct_actuator = true;
    offboard_mode_pub_->publish(mode);
  }

  // 可选：自动切offboard/解锁（默认关闭，避免误操作）
  // 为避免在参考/状态未就绪时误解锁导致坠机，这里要求已收到状态与参考后才开始计数
  if ((enable_auto_offboard_ || enable_auto_arm_) && has_state_ && has_ref_ &&
    offboard_counter_ <= offboard_setpoint_warmup_)
  {
    offboard_counter_++;
    if (offboard_counter_ == offboard_setpoint_warmup_) {
      if (enable_auto_offboard_) {
        // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1, PX4_CUSTOM_MAIN_MODE_OFFBOARD=6
        publishVehicleCommand(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f, 0.0f);
      }
      if (enable_auto_arm_) {
        publishVehicleCommand(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f, 0.0f);
      }
    }
  }

  if (!has_state_ || !has_ref_) {
    return;
  }

  ControlOutput out;
  if (!controller_.compute(state_, ref_, dt, &out)) {
    return;
  }

  px4_msgs::msg::ActuatorMotors motors{};
  motors.timestamp = now_us;
  motors.timestamp_sample = now_us;
  motors.reversible_flags = 0;

  const float nan = std::numeric_limits<float>::quiet_NaN();
  motors.control.fill(nan);
  // 输出顺序按PX4电机函数顺序（motor.output_map可配置）
  for (int out_i = 0; out_i < 4; ++out_i) {
    const int internal_i = motor_output_map_[out_i];
    motors.control[out_i] = static_cast<float>(out.motor_controls(internal_i));
  }

  motors_pub_->publish(motors);

  // PX4 land detector使用vehicle_thrust_setpoint判断“低油门/落地”状态。
  // 在direct_actuator模式下如果不发布该topic，它可能长期认为油门=0，从而误判落地并自动上锁。
  if (publish_vehicle_thrust_setpoint_ && thrust_sp_pub_) {
    px4_msgs::msg::VehicleThrustSetpoint thrust_sp{};
    thrust_sp.timestamp = now_us;
    thrust_sp.timestamp_sample = now_us;

    double throttle = 0.0;
    const double denom = 4.0 * std::max(1e-6, motor_force_max_);
    if (std::isfinite(out.thrust_total)) {
      throttle = std::clamp(out.thrust_total / denom, 0.0, 1.0);
    }

    // 多旋翼推力沿机体系-FRD的-z方向（向上），因此z为负。
    thrust_sp.xyz = {0.0f, 0.0f, static_cast<float>(-throttle)};
    thrust_sp_pub_->publish(thrust_sp);
  }
}

}  // namespace tanh_controller

int main(int argc, char ** argv)
{
  // ROS2主入口
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tanh_controller::TanhControllerNode>());
  rclcpp::shutdown();
  return 0;
}
