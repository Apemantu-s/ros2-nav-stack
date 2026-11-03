#include "nav_stack/ekf.hpp"
#include <cmath>

namespace nav_stack {

double wrapAngle(double a) {
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

Ekf::Ekf() : Ekf(Params{}) {}

Ekf::Ekf(const Params& p) : p_(p) { reset(0, 0, 0); }

void Ekf::reset(double x, double y, double theta) {
  x_ << x, y, theta, 0, 0;
  P_.setZero();
  P_(0, 0) = 1e-6;
  P_(1, 1) = 1e-6;
  P_(2, 2) = 1e-6;
  P_(3, 3) = 0.01;
  P_(4, 4) = 0.01;
}

void Ekf::predict(double dt) {
  if (dt <= 0) return;
  double th = x_(2);
  double v = x_(3);
  double w = x_(4);

  x_(0) += v * std::cos(th) * dt;
  x_(1) += v * std::sin(th) * dt;
  x_(2) = wrapAngle(th + w * dt);

  Eigen::Matrix<double, 5, 5> F = Eigen::Matrix<double, 5, 5>::Identity();
  F(0, 2) = -v * std::sin(th) * dt;
  F(0, 3) = std::cos(th) * dt;
  F(1, 2) = v * std::cos(th) * dt;
  F(1, 3) = std::sin(th) * dt;
  F(2, 4) = dt;

  Eigen::Matrix<double, 5, 5> Q = Eigen::Matrix<double, 5, 5>::Zero();
  Q(0, 0) = p_.q_pos * dt;
  Q(1, 1) = p_.q_pos * dt;
  Q(2, 2) = p_.q_theta * dt;
  Q(3, 3) = p_.q_v * dt;
  Q(4, 4) = p_.q_w * dt;

  P_ = F * P_ * F.transpose() + Q;
}

void Ekf::updateOne(int i, double z, double r) {
  double S = P_(i, i) + r;
  Eigen::Matrix<double, 5, 1> K = P_.col(i) / S;
  double error = z - x_(i);
  x_ += K * error;
  x_(2) = wrapAngle(x_(2));
  P_ = P_ - K * P_.row(i);
  P_ = 0.5 * (P_ + P_.transpose());  
}

void Ekf::updateWheel(double v, double w, bool still) {
  if (still) {
    updateOne(3, 0.0, p_.r_still);
    updateOne(4, 0.0, p_.r_still);
  } else {
    updateOne(3, v, p_.r_wheel_v);
    updateOne(4, w, p_.r_wheel_w);
  }
}

void Ekf::updateGyro(double w_meas) { updateOne(4, w_meas, p_.r_gyro); }

}
