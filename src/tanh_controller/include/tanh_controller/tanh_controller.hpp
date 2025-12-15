#pragma once

#include <Eigen/Dense>

namespace tanh_controller {

/**
 * @brief 四旋翼状态（NED/FRD）
 */
struct VehicleState
{
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};      ///< 位置 [m]
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};      ///< 速度 [m/s]
  Eigen::Quaterniond q_body_to_ned{Eigen::Quaterniond::Identity()}; ///< 姿态（机体->NED）
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};   ///< 机体角速度 [rad/s]
};

/**
 * @brief 轨迹参考（NED）
 */
struct TrajectoryRef
{
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()}; ///< 期望位置 [m]
  double yaw{0.0};                                       ///< 期望航向角 [rad]
  bool valid{false};                                    ///< 是否有效
};

/**
 * @brief 位置环增益
 */
struct PositionGains
{
  Eigen::Vector3d M_P{Eigen::Vector3d::Ones()};
  Eigen::Vector3d K_P{Eigen::Vector3d::Ones()};
  Eigen::Vector3d M_V{Eigen::Vector3d::Ones()};
  Eigen::Vector3d K_V{Eigen::Vector3d::Ones()};
  Eigen::Vector3d K_Acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d P_V{Eigen::Vector3d::Zero()};
  Eigen::Vector3d L_V{Eigen::Vector3d::Ones()};
};

/**
 * @brief 姿态环增益
 */
struct AttitudeGains
{
  Eigen::Vector3d M_Angle{Eigen::Vector3d::Ones()};
  Eigen::Vector3d K_Angle{Eigen::Vector3d::Ones()};
  Eigen::Vector3d M_AngularVelocity{Eigen::Vector3d::Ones()};
  Eigen::Vector3d K_AngularVelocity{Eigen::Vector3d::Ones()};
  Eigen::Vector3d K_AngularAcceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d P_AngularVelocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d L_AngularVelocity{Eigen::Vector3d::Ones()};
};

/**
 * @brief 控制分配参数（对应推导中的G1）
 */
struct AllocationParams
{
  double l{0.2};        ///< 机臂长度 [m]
  double beta{M_PI_4};  ///< 机臂与x_B夹角 [rad]
  double cq_ct{0.01};   ///< c_q/c_t 比值
};

/**
 * @brief 控制输出
 */
struct ControlOutput
{
  double thrust_total{0.0};             ///< 总推力 [N]
  Eigen::Vector3d torque_body{Eigen::Vector3d::Zero()}; ///< 期望力矩（机体FRD）[N*m]
  Eigen::Vector4d motor_forces{Eigen::Vector4d::Zero()}; ///< 每个电机推力 [N]
  Eigen::Vector4d motor_controls{Eigen::Vector4d::Zero()}; ///< 归一化控制量 [0..1]
};

/**
 * @brief tanh S形反馈控制器（位置+姿态+扰动观测器）
 */
class TanhController
{
public:
  /** @brief 构造函数 */
  TanhController();

  /** @brief 设置质量 */
  void setMass(double mass);

  /** @brief 设置重力加速度（NED向下为正） */
  void setGravity(double gravity);

  /** @brief 设置位置环增益 */
  void setPositionGains(const PositionGains & gains);

  /** @brief 设置姿态环增益 */
  void setAttitudeGains(const AttitudeGains & gains);

  /** @brief 设置惯性矩阵（机体系FRD） */
  void setInertia(const Eigen::Matrix3d & inertia);

  /** @brief 设置控制分配参数 */
  void setAllocationParams(const AllocationParams & alloc);

  /** @brief 设置单电机最大推力，用于归一化映射 */
  void setMotorForceMax(double max_force);

  /** @brief 设置位置环最大倾角限制（单位rad，<=0表示不限制） */
  void setMaxTilt(double max_tilt_rad);

  /** @brief 重置观测器状态 */
  void reset();

  /**
   * @brief 计算控制输出
   * @param state 当前状态
   * @param ref   轨迹参考
   * @param dt    采样周期 [s]
   * @param out   控制输出
   * @return 是否成功计算（ref无效时返回false）
   */
  bool compute(const VehicleState & state, const TrajectoryRef & ref, double dt, ControlOutput * out);

private:
  /** @brief 逐元素tanh饱和 */
  static Eigen::Vector3d tanhVec(const Eigen::Vector3d & x);

  /** @brief 根据推力方向和航向角计算期望姿态 */
  Eigen::Quaterniond computeDesiredAttitude(
    const Eigen::Vector3d & z_b_d_ned, double yaw_d) const;

  /** @brief 位置环与速度扰动观测 */
  void computePosition(
    const VehicleState & state, const TrajectoryRef & ref, double dt,
    Eigen::Vector3d * thrust_vec_ned, double * thrust_norm);

  /** @brief 姿态环与力矩扰动观测 */
  void computeAttitude(
    const VehicleState & state, const Eigen::Quaterniond & q_d, double dt,
    Eigen::Vector3d * torque_body);

  /** @brief 由(T,τ)分配得到各电机推力 */
  Eigen::Vector4d allocateMotors(double thrust_total, const Eigen::Vector3d & torque_body) const;

  /** @brief 推力到归一化控制量线性映射 */
  Eigen::Vector4d forcesToControls(const Eigen::Vector4d & forces) const;

private:
  double mass_{1.0};
  double gravity_{9.81};
  Eigen::Matrix3d inertia_{Eigen::Matrix3d::Identity()};
  PositionGains pos_gains_{};
  AttitudeGains att_gains_{};
  AllocationParams alloc_{};
  double motor_force_max_{10.0};
  double max_tilt_rad_{0.0};

  Eigen::Vector3d velocity_error_hat_ned_{Eigen::Vector3d::Zero()};  ///< 速度误差观测器状态(NED)
  Eigen::Vector3d angular_velocity_error_hat_body_{Eigen::Vector3d::Zero()};  ///< 角速度误差观测器状态(FRD)
  Eigen::Vector3d last_velocity_ned_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_angular_velocity_body_{Eigen::Vector3d::Zero()};
  bool has_last_velocity_{false};
  bool has_last_angular_velocity_{false};
  bool first_run_{true};
};

}  // namespace tanh_controller
