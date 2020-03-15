#include "Ellipse.h"
#include <iostream>

const double Ellipse::pi = 3.14159265359;

bool Ellipse::FromConic(double a, double b, double c, double d, double e, double f) {
  Eigen::Matrix2d m_inv;
  m_inv << a, b,
           b, c;
  const Eigen::Matrix2d m_sqr = m_inv.inverse();
  const Eigen::Vector2d de(-d, -e);
  v_c = m_sqr * de;
  const double r = de.dot(v_c) - f;
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(m_sqr * r);
  const Eigen::Matrix2d m = solver.eigenvectors() * solver.eigenvalues().cwiseAbs().cwiseSqrt().asDiagonal();
  v_a = m.col(1);
  v_b = m.col(0);
  return b*b > a*c;
}

void Ellipse::ToConic(double& a, double& b, double& c, double& d, double& e, double& f, bool hyperbola) const {
  Eigen::Matrix2d m1, m2;
  m1 << v_a, v_b;
  m2 << v_a, (hyperbola ? -v_b : v_b);
  const Eigen::Matrix2d m_inv = (m1 * m2.transpose()).inverse();
  const Eigen::Vector2d de = m_inv * v_c;
  d = -de[0];
  e = -de[1];
  f = de.dot(v_c) - 1.0;
  a = m_inv(0,0);
  b = m_inv(1,0);
  c = m_inv(1,1);
}
