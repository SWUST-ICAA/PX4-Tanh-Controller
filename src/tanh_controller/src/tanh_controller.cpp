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

void TanhController::reset()
{
  // 重置观测器内部状态
  e_hat_v_.setZero();
  e_hat_omega_.setZero();
  first_run_ = true;
}

Eigen::Vector3d TanhController::tanhVec(const Eigen::Vector3d & x)
{
  // 逐元素tanh
  return x.array().tanh().matrix();
}

bool TanhController::compute(
  const VehicleState & state, const TrajectoryRef & ref, double dt, ControlOutput * out)
{
  // 根据状态与参考计算电机归一化推力（0..1）
  if (!out || !ref.valid) {
    return false;
  }

  if (first_run_) {
    // 首次运行时清零观测器，避免历史残留
    e_hat_v_.setZero();
    e_hat_omega_.setZero();
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
  // 根据推导式(19)-(22)构造期望机体系三个轴
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
  // 位置误差 e_xi = xi - xi_d
  Eigen::Vector3d e_xi = state.position_ned - ref.position_ned;

  // 速度误差 e_v = v + M1*tanh(K1*e_xi)
  Eigen::Vector3d tanh_exi = tanhVec(pos_gains_.K1.cwiseProduct(e_xi));
  Eigen::Vector3d e_v = state.velocity_ned + pos_gains_.M1.cwiseProduct(tanh_exi);

  // 观测误差 epsilon_v = e_v - e_hat_v
  Eigen::Vector3d epsilon_v = e_v - e_hat_v_;
  Eigen::Vector3d mu_v =
    pos_gains_.Pv.cwiseProduct(tanhVec(pos_gains_.Lv.cwiseProduct(epsilon_v)));

  Eigen::Vector3d g_ned(0.0, 0.0, gravity_);

  // 推力矢量 T_d*z_B,d = m*(mu_v + g + M2*tanh(K2*e_v))
  Eigen::Vector3d tanh_ev = tanhVec(pos_gains_.K2.cwiseProduct(e_v));
  Eigen::Vector3d thrust_vec = mass_ * (mu_v + g_ned + pos_gains_.M2.cwiseProduct(tanh_ev));

  // 更新速度扰动观测器(13)
  Eigen::Vector3d dot_e_hat_v = (-thrust_vec / mass_) + g_ned + mu_v;
  e_hat_v_ += dt * dot_e_hat_v;

  if (thrust_vec_ned) {
    *thrust_vec_ned = thrust_vec;
  }
  if (thrust_norm) {
    *thrust_norm = thrust_vec.norm();
  }
}

void TanhController::computeAttitude(
  const VehicleState & state, const Eigen::Quaterniond & q_d, double dt,
  Eigen::Vector3d * torque_body)
{
  // 姿态环：计算期望力矩，并更新角速度扰动观测器
  Eigen::Quaterniond q = state.q_body_to_ned.normalized();
  // PX4的姿态四元数定义为：机体(FRD)->NED。
  // 为保证负反馈方向一致，这里使用 q_err = q_d^{-1} ⊗ q（对应 R_err = R_d^T * R）。
  Eigen::Quaterniond q_err = q_d.conjugate() * q;

  if (q_err.w() < 0.0) {
    q_err.coeffs() *= -1.0;  // (24)
  }

  // 姿态误差向量 e_theta 取四元数虚部
  Eigen::Vector3d e_theta = q_err.vec();

  // 角速度误差 e_omega = Ω + M_theta*tanh(K_theta*e_theta)
  Eigen::Vector3d tanh_eth = tanhVec(att_gains_.K_theta.cwiseProduct(e_theta));
  Eigen::Vector3d e_omega =
    state.angular_velocity_body + att_gains_.M_theta.cwiseProduct(tanh_eth);

  // 观测误差 epsilon_omega = e_omega - e_hat_omega
  Eigen::Vector3d epsilon_omega = e_omega - e_hat_omega_;
  Eigen::Vector3d mu_omega =
    att_gains_.P_omega.cwiseProduct(tanhVec(att_gains_.L_omega.cwiseProduct(epsilon_omega)));

  Eigen::Vector3d omega = state.angular_velocity_body;
  Eigen::Vector3d omega_cross_Iomega = omega.cross(inertia_ * omega);

  // 力矩控制律(31)
  Eigen::Vector3d tanh_eomega = tanhVec(att_gains_.K_omega.cwiseProduct(e_omega));
  Eigen::Vector3d u_term = att_gains_.M_omega.cwiseProduct(tanh_eomega);
  Eigen::Vector3d tau_d =
    omega_cross_Iomega - inertia_ * mu_omega - inertia_ * u_term;

  // 更新姿态扰动观测器(27)（按推导简化）
  Eigen::Vector3d dot_e_hat_omega =
    (-omega_cross_Iomega) + inertia_.inverse() * tau_d + mu_omega;
  e_hat_omega_ += dt * dot_e_hat_omega;

  if (torque_body) {
    *torque_body = tau_d;
  }
}

Eigen::Vector4d TanhController::allocateMotors(
  double thrust_total, const Eigen::Vector3d & torque_body) const
{
  // 控制分配：由(T,τ)求解单电机推力（对应推导中的G1）
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

  // 推力不允许为负，做简单截断
  for (int i = 0; i < 4; ++i) {
    if (!std::isfinite(forces(i))) {
      forces(i) = 0.0;
    }
    forces(i) = std::max(0.0, forces(i));
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
