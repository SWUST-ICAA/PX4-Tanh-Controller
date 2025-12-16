#include "tanh_controller/tanh_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tanh_controller {

TanhController::TanhController() = default;

void TanhController::setMass(double mass)
{
  // 设置质量，并做最小值保护
  mass_ = std::max(1e-3, mass);
}

void TanhController::setGravity(double gravity)
{
  // 设置重力加速度（向下为正），并限制非负
  gravity_ = std::max(0.0, gravity);
}

void TanhController::setPositionGains(const PositionGains & gains)
{
  // 设置位置环增益
  pos_gains_ = gains;
}

void TanhController::setAttitudeGains(const AttitudeGains & gains)
{
  // 设置姿态环增益
  att_gains_ = gains;
}

void TanhController::setInertia(const Eigen::Matrix3d & inertia)
{
  // 设置机体系惯性矩阵
  inertia_ = inertia;
}

void TanhController::setAllocationParams(const AllocationParams & alloc)
{
  // 设置控制分配参数
  alloc_ = alloc;
}

void TanhController::setMotorForceMax(double max_force)
{
  // 设置单电机最大推力，用于推力->归一化控制量映射
  motor_force_max_ = std::max(1e-3, max_force);
}

void TanhController::setMaxTilt(double max_tilt_rad)
{
  // 位置环最大倾角限制（0/负数表示不限制）
  if (!std::isfinite(max_tilt_rad) || max_tilt_rad <= 0.0) {
    max_tilt_rad_ = 0.0;
    return;
  }
  // 限制到(0, pi/2)之间，避免tan发散
  max_tilt_rad_ = std::clamp(max_tilt_rad, 1e-3, (M_PI_2 - 1e-3));
}

void TanhController::setLinearAccelerationLowPassHz(double cutoff_hz)
{
  linear_accel_lpf_cutoff_hz_ = (std::isfinite(cutoff_hz) && cutoff_hz > 0.0) ? cutoff_hz : 0.0;
  linear_accel_lpf_initialized_ = false;
}

void TanhController::setAngularAccelerationLowPassHz(double cutoff_hz)
{
  angular_accel_lpf_cutoff_hz_ = (std::isfinite(cutoff_hz) && cutoff_hz > 0.0) ? cutoff_hz : 0.0;
  angular_accel_lpf_initialized_ = false;
}

void TanhController::reset()
{
  // 重置观测器内部状态
  velocity_error_hat_ned_.setZero();
  angular_velocity_error_hat_body_.setZero();
  last_velocity_ned_.setZero();
  last_angular_velocity_body_.setZero();
  has_last_velocity_ = false;
  has_last_angular_velocity_ = false;
  first_run_ = true;

  linear_accel_lpf_state_ned_.setZero();
  angular_accel_lpf_state_body_.setZero();
  linear_accel_lpf_initialized_ = false;
  angular_accel_lpf_initialized_ = false;
}

Eigen::Vector3d TanhController::tanhVec(const Eigen::Vector3d & x)
{
  // 逐元素tanh
  return x.array().tanh().matrix();
}

Eigen::Vector3d TanhController::lowPassVec3(
  const Eigen::Vector3d & x, double cutoff_hz, double dt,
  Eigen::Vector3d * state, bool * initialized)
{
  if (!state || !initialized) {
    return x;
  }
  if (!(cutoff_hz > 0.0) || !std::isfinite(cutoff_hz) || !(dt > 0.0) || !std::isfinite(dt)) {
    return x;
  }

  const double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
  const double alpha = dt / (tau + dt);

  if (!(*initialized)) {
    *state = x;
    *initialized = true;
    return x;
  }

  *state += alpha * (x - *state);
  return *state;
}

bool TanhController::compute(
  const VehicleState & state, const TrajectoryRef & ref, double dt, ControlOutput * out)
{
  // 根据状态与参考计算控制输出（总推力+力矩），并做电机分配得到归一化推力（0..1）
  //
  // 主要步骤：
  // 1) 位置环得到推力矢量：T_d z_B,d（NED）
  // 2) 推力方向单位化：z_B,d = (T_d z_B,d) / ||T_d z_B,d||
  // 3) 推力矢量分解得到期望姿态：q_d = f(z_B,d, yaw_d)
  // 4) 姿态环得到期望力矩：tau_d（FRD）
  // 5) 控制分配得到单电机推力：G1 f = [T_d, tau_d]^T
  if (!out || !ref.valid) {
    return false;
  }

  if (first_run_) {
    // 首次运行时清零观测器，避免历史残留
    velocity_error_hat_ned_.setZero();
    angular_velocity_error_hat_body_.setZero();
    last_velocity_ned_ = state.velocity_ned;
    last_angular_velocity_body_ = state.angular_velocity_body;
    has_last_velocity_ = true;
    has_last_angular_velocity_ = true;
    first_run_ = false;
  }

  dt = std::clamp(dt, 1e-4, 0.1);  // 防止dt异常

  Eigen::Vector3d thrust_vec_ned;
  double thrust_norm = 0.0;
  computePosition(state, ref, dt, &thrust_vec_ned, &thrust_norm);

  Eigen::Vector3d z_b_d_ned = thrust_vec_ned.normalized();
  if (!std::isfinite(z_b_d_ned.x()) || thrust_norm < 1e-3) {
    z_b_d_ned = Eigen::Vector3d(0.0, 0.0, 1.0);
  }

  Eigen::Quaterniond q_d = computeDesiredAttitude(z_b_d_ned, ref.yaw);

  Eigen::Vector3d torque_body;
  computeAttitude(state, q_d, dt, &torque_body);

  Eigen::Vector4d motor_forces = allocateMotors(thrust_norm, torque_body);
  Eigen::Vector4d motor_controls = forcesToControls(motor_forces);

  out->thrust_total = thrust_norm;
  out->torque_body = torque_body;
  out->motor_forces = motor_forces;
  out->motor_controls = motor_controls;
  return true;
}

Eigen::Quaterniond TanhController::computeDesiredAttitude(
  const Eigen::Vector3d & z_b_d_ned, double yaw_d) const
{
  // 推力矢量分解：由z轴期望方向+航向角构造期望姿态
  // 期望机体系轴（NED）：
  // z_B,d = z_b_d_ned / ||z_b_d_ned||
  // x_C,d = [cos(yaw_d), sin(yaw_d), 0]^T
  // y_B,d = (z_B,d × x_C,d) / ||z_B,d × x_C,d||
  // x_B,d = y_B,d × z_B,d
  // R_d   = [x_B,d, y_B,d, z_B,d]
  Eigen::Vector3d z_b = z_b_d_ned.normalized();
  Eigen::Vector3d x_c(std::cos(yaw_d), std::sin(yaw_d), 0.0);

  Eigen::Vector3d y_b = z_b.cross(x_c);
  if (y_b.norm() < 1e-6) {
    // 当z轴与x_c接近平行时，选取备用方向
    x_c = Eigen::Vector3d(1.0, 0.0, 0.0);
    y_b = z_b.cross(x_c);
  }
  y_b.normalize();

  Eigen::Vector3d x_b = y_b.cross(z_b);
  x_b.normalize();

  Eigen::Matrix3d R;
  R.col(0) = x_b;
  R.col(1) = y_b;
  R.col(2) = z_b;

  Eigen::Quaterniond q_d(R);
  q_d.normalize();
  return q_d;
}

void TanhController::computePosition(
  const VehicleState & state, const TrajectoryRef & ref, double dt,
  Eigen::Vector3d * thrust_vec_ned, double * thrust_norm)
{
  // 位置环：计算期望推力矢量，并更新速度扰动观测器
  //
  // 误差与中间量（NED）：
  // e_xi = xi - xi_d
  // e_v  = dot(xi) + M_xi ⊙ tanh(K_xi ⊙ e_xi)
  // ε_v  = e_v - ê_v
  // μ_v  = P_v ⊙ tanh(L_v ⊙ ε_v)
  //
  // 加速度项：ddot(xi) 由速度差分估计
  // ddot(xi) ≈ (v_k - v_{k-1}) / dt
  //
  // 推力矢量控制律（NED）：
  // (T_d z_B,d)/m = μ_v + g + M_v ⊙ tanh(K_v ⊙ e_v) + K_a ⊙ ddot(xi)
  //
  // 速度扰动观测器：
  // ê̇_v = -(T_d z_B,d)/m + g + μ_v
  const Eigen::Vector3d position_error_ned = state.position_ned - ref.position_ned;

  Eigen::Vector3d linear_acceleration_ned = Eigen::Vector3d::Zero();
  if (has_last_velocity_) {
    linear_acceleration_ned = (state.velocity_ned - last_velocity_ned_) / dt;
  }
  if (!linear_acceleration_ned.allFinite()) {
    linear_acceleration_ned.setZero();
  }
  linear_acceleration_ned = lowPassVec3(
    linear_acceleration_ned, linear_accel_lpf_cutoff_hz_, dt,
    &linear_accel_lpf_state_ned_, &linear_accel_lpf_initialized_);

  // 速度误差 e_v
  const Eigen::Vector3d tanh_position_error =
    tanhVec(pos_gains_.K_P.cwiseProduct(position_error_ned));
  const Eigen::Vector3d velocity_error_ned =
    state.velocity_ned + pos_gains_.M_P.cwiseProduct(tanh_position_error);

  // 观测误差 velocity_error_estimation_error = velocity_error - velocity_error_hat
  const Eigen::Vector3d velocity_error_estimation_error_ned =
    velocity_error_ned - velocity_error_hat_ned_;
  const Eigen::Vector3d velocity_disturbance_estimate_ned =
    pos_gains_.P_V.cwiseProduct(
      tanhVec(pos_gains_.L_V.cwiseProduct(velocity_error_estimation_error_ned)));

  Eigen::Vector3d g_ned(0.0, 0.0, gravity_);

  // 推力矢量：T_d z_B,d
  const Eigen::Vector3d tanh_velocity_error =
    tanhVec(pos_gains_.K_V.cwiseProduct(velocity_error_ned));
  Eigen::Vector3d thrust_vector_over_mass_ned =
    velocity_disturbance_estimate_ned + g_ned +
    pos_gains_.M_V.cwiseProduct(tanh_velocity_error) +
    pos_gains_.K_Acceleration.cwiseProduct(linear_acceleration_ned);  // (T_d*z_B,d)/m

  // 限制最大倾角：约束 ||v_xy|| <= v_z * tan(max_tilt)
  if (max_tilt_rad_ > 0.0) {
    // 避免出现需要倒飞(v_z<=0)的情况
    thrust_vector_over_mass_ned.z() = std::max(thrust_vector_over_mass_ned.z(), 1e-3);

    const double horiz =
      std::hypot(thrust_vector_over_mass_ned.x(), thrust_vector_over_mass_ned.y());
    const double max_horiz = thrust_vector_over_mass_ned.z() * std::tan(max_tilt_rad_);
    if (horiz > max_horiz && horiz > 1e-9) {
      const double scale = max_horiz / horiz;
      thrust_vector_over_mass_ned.x() *= scale;
      thrust_vector_over_mass_ned.y() *= scale;
    }
  }

  const Eigen::Vector3d desired_thrust_vector_ned = mass_ * thrust_vector_over_mass_ned;

  // 更新速度扰动观测器 ê_v
  const Eigen::Vector3d velocity_error_hat_dot_ned =
    (-desired_thrust_vector_ned / mass_) + g_ned + velocity_disturbance_estimate_ned;
  velocity_error_hat_ned_ += dt * velocity_error_hat_dot_ned;
  last_velocity_ned_ = state.velocity_ned;
  has_last_velocity_ = true;

  if (thrust_vec_ned) {
    *thrust_vec_ned = desired_thrust_vector_ned;
  }
  if (thrust_norm) {
    *thrust_norm = desired_thrust_vector_ned.norm();
  }
}

void TanhController::computeAttitude(
  const VehicleState & state, const Eigen::Quaterniond & q_d, double dt,
  Eigen::Vector3d * torque_body)
{
  // 姿态环：计算期望力矩，并更新角速度扰动观测器
  //
  // 四元数误差：
  // q_e' = q_d^{-1} ⊗ q
  // q_e  = sgn(q_e,w') * q_e'
  // e_theta = vec(q_e)
  //
  // 角速度误差与扰动观测（FRD）：
  // e_Omega = Omega + M_theta ⊙ tanh(K_theta ⊙ e_theta)
  // ε_Omega = e_Omega - ê_Omega
  // μ_Omega = P_Omega ⊙ tanh(L_Omega ⊙ ε_Omega)
  //
  // 角加速度项：dot(Omega) 由角速度差分估计
  // dot(Omega) ≈ (Omega_k - Omega_{k-1}) / dt
  //
  // 力矩控制律（FRD）：
  // tau_d = Omega × (I Omega) - I μ_Omega
  //         - I (M_Omega ⊙ tanh(K_Omega ⊙ e_Omega))
  //         - I (K_alpha ⊙ dot(Omega))
  //
  // 角速度误差观测器（显式更新）：
  // ê̇_Omega = -I^{-1} (Omega × (I Omega)) + I^{-1} tau_d + μ_Omega
  const Eigen::Quaterniond q = state.q_body_to_ned.normalized();

  Eigen::Vector3d angular_acceleration_body = Eigen::Vector3d::Zero();
  if (has_last_angular_velocity_) {
    angular_acceleration_body = (state.angular_velocity_body - last_angular_velocity_body_) / dt;
  }
  if (!angular_acceleration_body.allFinite()) {
    angular_acceleration_body.setZero();
  }
  angular_acceleration_body = lowPassVec3(
    angular_acceleration_body, angular_accel_lpf_cutoff_hz_, dt,
    &angular_accel_lpf_state_body_, &angular_accel_lpf_initialized_);

  // 四元数误差：q_e' = q_d^{-1} ⊗ q（对应 R_e = R_d^T R）
  Eigen::Quaterniond q_error = q_d.conjugate() * q;
  if (q_error.w() < 0.0) {
    q_error.coeffs() *= -1.0;
  }

  // 姿态误差向量：e_theta = vec(q_e)
  const Eigen::Vector3d attitude_error = q_error.vec();

  // 角速度误差：e_Omega
  const Eigen::Vector3d tanh_attitude_error =
    tanhVec(att_gains_.K_Angle.cwiseProduct(attitude_error));
  const Eigen::Vector3d angular_velocity_error_body =
    state.angular_velocity_body + att_gains_.M_Angle.cwiseProduct(tanh_attitude_error);

  // 观测误差：angular_velocity_estimation_error = angular_velocity_error - angular_velocity_error_hat
  const Eigen::Vector3d angular_velocity_estimation_error_body =
    angular_velocity_error_body - angular_velocity_error_hat_body_;
  const Eigen::Vector3d angular_velocity_disturbance_estimate_body =
    att_gains_.P_AngularVelocity.cwiseProduct(
      tanhVec(att_gains_.L_AngularVelocity.cwiseProduct(angular_velocity_estimation_error_body)));

  const Eigen::Vector3d angular_velocity_body = state.angular_velocity_body;
  const Eigen::Vector3d omega_cross_Iomega =
    angular_velocity_body.cross(inertia_ * angular_velocity_body);
  const Eigen::Matrix3d inertia_inv = inertia_.inverse();

  // 力矩控制律：tau_d
  const Eigen::Vector3d tanh_angular_velocity_error =
    tanhVec(att_gains_.K_AngularVelocity.cwiseProduct(angular_velocity_error_body));
  const Eigen::Vector3d angular_velocity_control_term =
    att_gains_.M_AngularVelocity.cwiseProduct(tanh_angular_velocity_error);
  const Eigen::Vector3d desired_torque_body =
    omega_cross_Iomega - inertia_ * angular_velocity_disturbance_estimate_body -
    inertia_ * angular_velocity_control_term -
    inertia_ * att_gains_.K_AngularAcceleration.cwiseProduct(angular_acceleration_body);

  // 更新角速度误差观测器 ê_Omega
  const Eigen::Vector3d angular_velocity_error_hat_dot_body =
    (-inertia_inv * omega_cross_Iomega) + inertia_inv * desired_torque_body +
    angular_velocity_disturbance_estimate_body;
  angular_velocity_error_hat_body_ += dt * angular_velocity_error_hat_dot_body;
  last_angular_velocity_body_ = state.angular_velocity_body;
  has_last_angular_velocity_ = true;

  if (torque_body) {
    *torque_body = desired_torque_body;
  }
}

Eigen::Vector4d TanhController::allocateMotors(
  double thrust_total, const Eigen::Vector3d & torque_body) const
{
  // 控制分配：由(T,τ)求解单电机推力（对应推导中的G1）
  // [T, tau_x, tau_y, tau_z]^T = G1 * f，求解 f = G1^{-1} * wrench
  const double s = std::sin(alloc_.beta);
  const double c = std::cos(alloc_.beta);
  const double l = alloc_.l;

  Eigen::Matrix4d G1;
  G1 << 1.0, 1.0, 1.0, 1.0,
    l * s, -l * s, -l * s, l * s,
    l * c, l * c, -l * c, -l * c,
    -alloc_.cq_ct, alloc_.cq_ct, -alloc_.cq_ct, alloc_.cq_ct;

  Eigen::Vector4d wrench;
  wrench << thrust_total, torque_body.x(), torque_body.y(), torque_body.z();

  // 使用LU求解，避免显式求逆
  Eigen::Vector4d forces = G1.fullPivLu().solve(wrench);

  for (int i = 0; i < 4; ++i) {
    if (!std::isfinite(forces(i))) {
      forces(i) = 0.0;
    }
  }

  return forces;
}

Eigen::Vector4d TanhController::forcesToControls(const Eigen::Vector4d & forces) const
{
  // 线性映射：推力[N] -> 归一化控制量[0..1]
  Eigen::Vector4d controls;
  for (int i = 0; i < 4; ++i) {
    double u = forces(i) / motor_force_max_;
    u = std::clamp(u, 0.0, 1.0);
    controls(i) = u;
  }
  return controls;
}

}  // namespace tanh_controller
