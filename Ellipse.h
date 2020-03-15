#pragma once
#include <Eigen/Dense>
#include <cmath>

class Ellipse {
public:
  typedef Eigen::Vector2d Vector2d;
  static const double pi;
  static double CrossMag(const Vector2d& a, const Vector2d& b) {
    return a.x()*b.y() - a.y()*b.x();
  }
  static double CrossMagSqr(const Vector2d& a, const Vector2d& b) {
    const double mag = CrossMag(a, b);
    return mag * mag;
  }

  //Constructor
  Ellipse(const Vector2d& a = Vector2d::UnitX(),
          const Vector2d& b = Vector2d::UnitY(),
          const Vector2d& c = Vector2d::Zero()) : v_a(a), v_b(b), v_c(c) {}

  //Constructor from [ax^2 + 2bxy + cy^2 + 2dx + 2ey + f = 0]
  bool FromConic(double a, double b, double c, double d, double e, double f);
  void ToConic(double& a, double& b, double& c, double& d, double& e, double& f, bool hyperbola=false) const;

  //Rotational
  Vector2d VectorAtAngle(double theta) const {
    return v_a*std::cos(theta) + v_b*std::sin(theta);
  }
  Vector2d PointAtAngle(double theta) const {
    return v_a*std::cos(theta) + v_b*std::sin(theta) + v_c;
  }
  Vector2d PointAtAngleH1(double theta) const {
    return v_a*std::cosh(theta) + v_b*std::sinh(theta) + v_c;
  }
  Vector2d PointAtAngleH2(double theta) const {
    return -v_a*std::cosh(theta) + v_b*std::sinh(theta) + v_c;
  }
  Vector2d VelocityAtAngle(double theta) const {
    return v_a*-std::sin(theta) + v_b*std::cos(theta);
  }

  //Others
  bool IsInside(const Vector2d& pt) const {
    return CrossMagSqr(pt - v_c, v_a) + CrossMagSqr(pt - v_c, v_b) < CrossMagSqr(v_a, v_b);
  }

  //Invariants
  double CSqr() const {
    return v_a.squaredNorm() + v_b.squaredNorm();
  }
  double Area() const {
    return pi * CrossMag(v_a, v_b);
  }

  Vector2d v_a;
  Vector2d v_b;
  Vector2d v_c;
};
