#pragma once
#include <Eigen/Dense>

namespace nav_stack {

class Ekf {
 public:
  struct Params {
    double q_pos = 1e-4;
    double q_theta = 1e-4;
    double q_v = 0.5;
    double q_w = 0.1;
                           
    double r_wheel_v = 0.02 * 0.02;
    double r_wheel_w = 0.2 * 0.2;
    double r_gyro = 0.01 * 0.01;
    double r_still = 1e-8;
  };

  Ekf();
  explicit Ekf(const Params& p);

  void reset(double x, double y, double theta);
  void predict(double dt);
  void updateWheel(double v, double w, bool still = false);
  void updateGyro(double w_meas);

  double x() const { return x_(0); }
  double y() const { return x_(1); }
  double theta() const { return x_(2); }
  double v() const { return x_(3); }
  double w() const { return x_(4); }
  const Eigen::Matrix<double, 5, 5>& cov() const { return P_; }

 private:
  void updateOne(int index, double z, double r);

  Params p_;
  Eigen::Matrix<double, 5, 1> x_;
  Eigen::Matrix<double, 5, 5> P_;
};

double wrapAngle(double a);

} 
