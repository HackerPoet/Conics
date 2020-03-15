#include "ConicSolver.h"

void ConicSolver::Reset() {
  m_xx.setZero();
  m_x.setZero();
}

void ConicSolver::AddPoint(const Eigen::Vector2d& p) {
  Eigen::Matrix<double, 5, 1> x;
  x << p.x()*p.x(), p.x()*p.y(), p.y()*p.y(), p.x(), p.y();
  m_x += x;
  m_xx += x * x.transpose();
}

void ConicSolver::AddDualLine(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
  const Eigen::Vector2d v = (p2 - p1).normalized();
  const double d = p1.dot(v);
  AddPoint((d*v - p1) / (p1.dot(p1) - d*d));
}

void ConicSolver::Solve(double& a, double& b, double& c, double& d, double& e, double& f) {
  Eigen::Matrix<double, 5, 1> y = m_xx.ldlt().solve(m_x);
  a = y[0];
  b = y[1]*0.5f;
  c = y[2];
  d = y[3]*0.5f;
  e = y[4]*0.5f;
  f = -1.0f;
}

void ConicSolver::SolveDual(double& a, double& b, double& c, double& d, double& e, double& f) {
  Solve(a, b, c, d, e, f);
  Eigen::Matrix3d m;
  m << a, b, d,
       b, c, e,
       d, e, f;
  const Eigen::Matrix3d minv = -m.inverse();
  a = minv(0,0);
  b = minv(0,1);
  c = minv(1,1);
  d = minv(0,2);
  e = minv(1,2);
  f = minv(2,2);
}
