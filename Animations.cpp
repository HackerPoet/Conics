#define NOGDI
#include "Animations.h"
#include "Ellipse.h"
#include "DrawUtil.h"
#include "ConicSolver.h"
#include <iostream>
#include <random>
#include <sstream>
#include <cassert>
#include <fstream>

#define RUN_ONCE static int once = 0; if (!once && !once++)

Eigen::Vector2d Animations::moveable_points[5];
Eigen::Vector2d Animations::unfiltered_points[5];
int Animations::frame = 0;
bool Animations::animate_out = false;

static bool UpdateT(double& t, double t_in, double t_out) {
  if (Animations::animate_out) {
    t = std::max(t - t_out, 0.0);
    return t <= 0.0;
  } else {
    t = std::min(t + t_in, 1.0);
  }
  return false;
}
template<typename T>
static bool UpdateTMode(double& t, T& mode, double t_in, double t_out) {
  if (Animations::animate_out) {
    t = std::max(t - t_out, 0.0);
    if (t <= 0.0) {
      mode = T(mode + 1);
      if (mode == typename T::MODE_END) {
        return true;
      } else {
        Animations::animate_out = false;
      }
    }
  } else {
    t = std::min(t + t_in, 1.0);
  }
  return false;
}

static inline Eigen::Vector3d OuterProd(const Eigen::Vector2d& v) {
  return Eigen::Vector3d(v.x()*v.x(), 2.0*v.x()*v.y(), v.y()*v.y());
}
static inline Eigen::Vector3d OuterProdR(const Eigen::Vector2d& v) {
  return Eigen::Vector3d(v.y()*v.y(), -v.x()*v.y(), v.x()*v.x());
}
static inline double IntersectT(const Eigen::Vector2d& p1, const Eigen::Vector2d& v1, const Eigen::Vector2d& p2, const Eigen::Vector2d& v2) {
  Eigen::Matrix2d x;
  x << v1, v2;
  return (x.inverse() * (p2 - p1))(0);
}
static inline Eigen::Vector2d Intersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& v1, const Eigen::Vector2d& p2, const Eigen::Vector2d& v2) {
  return p1 + IntersectT(p1, v1, p2, v2)*v1;
}
static inline Eigen::Matrix2d PermuteMatrix(const Eigen::Matrix2d& u, const Eigen::Vector3d M) {
  Eigen::Matrix2d result = Eigen::Matrix2d::Identity();
  if (M(0)*M(2) < M(1)*M(1) && u.determinant() * (M(0) - M(2)) < 0.0) {
    result.col(0).swap(result.col(1));
  }
  return result;
}

bool Animations::Background(sf::RenderTarget& window, bool start_anim) {
  static double t = 0.0;
  DrawUtil::DrawGrid(window, t);
  if (start_anim) {
    t = std::min(t + 0.02, 1.0);
  }
  return animate_out;
}

bool Animations::RandomConics(sf::RenderTarget& window) {
  enum Mode {
    MODE_RANDOM,
    MODE_END,
  };
  static Mode mode = MODE_RANDOM;
  static double t = 0.0;
  static double anim = 0.0;

  moveable_points[0] = Eigen::Vector2d(std::sin(anim*0.11 + 1.0) + 0.7, std::sin(anim*0.09 + 2.0) + 0.7);
  moveable_points[1] = Eigen::Vector2d(std::sin(anim*0.17 + 2.0) + 0.7, std::sin(anim*0.13 + 4.0) - 0.7);
  moveable_points[2] = Eigen::Vector2d(std::sin(anim*0.23 + 3.0), std::sin(anim*0.15 + 6.0));
  moveable_points[3] = Eigen::Vector2d(std::sin(anim*0.27 + 4.0) - 0.7, std::sin(anim*0.19 + 8.0) + 0.7);
  moveable_points[4] = Eigen::Vector2d(std::sin(anim*0.07 + 5.0) - 0.7, std::sin(anim*0.29 + 10.0) - 0.7);

  //Get points
  const Eigen::Vector2d& p1 = moveable_points[0];
  const Eigen::Vector2d& p2 = moveable_points[1];
  const Eigen::Vector2d& p3 = moveable_points[2];
  const Eigen::Vector2d& p4 = moveable_points[3];
  const Eigen::Vector2d& p5 = moveable_points[4];

  //Add points to the solver
  ConicSolver s;
  s.AddPoint(p1);
  s.AddPoint(p2);
  s.AddPoint(p3);
  s.AddPoint(p4);
  s.AddPoint(p5);

  //Solve for the conic
  double c1, c2, c3, c4, c5, c6;
  s.Solve(c1, c2, c3, c4, c5, c6);

  //Create the conic
  Ellipse ellipse;
  const bool is_hyperbola = ellipse.FromConic(c1, c2, c3, c4, c5, c6);

  //Draw the result
  DrawUtil::DrawConic(window, ellipse, sf::Color::White, is_hyperbola, t, 18.0);

  anim += 0.025;
  return UpdateTMode(t, mode, 0.02, 0.04);
}

bool Animations::DualLines(sf::RenderTarget& window) {
  enum Mode {
    MODE_PT_ONLY,
    MODE_LINE_ONLY,
    MODE_PT_AND_LINE,
    MODE_CIRCLE,
    MODE_PERP,
    MODE_END,
  };
  static Mode mode = MODE_PT_ONLY;
  static double t = 0.0;
  RUN_ONCE{
    unfiltered_points[0] = Eigen::Vector2d(0.405, -0.48);
    moveable_points[0] = unfiltered_points[0];
  }

  const Eigen::Vector2d& p1 = moveable_points[0];

  if (mode >= MODE_CIRCLE) {
    const bool anim_point = (mode == MODE_CIRCLE && !animate_out);
    const double pbounce = anim_point ? DrawUtil::SmoothBounce(t, 0.25, 40.0) : 1.0;
    DrawUtil::DrawPoint(window, Eigen::Vector2d::Zero(), sf::Color::White, pbounce*20.0);
    DrawUtil::DrawCircle(window, Eigen::Vector2d::Zero(), sf::Color::White, pbounce*DrawUtil::scale);
  }

  if (mode == MODE_PT_ONLY || mode >= MODE_PT_AND_LINE) {
    const bool anim_point = (mode == MODE_PT_ONLY || (mode == MODE_PT_AND_LINE && !animate_out));
    const double pbounce = anim_point ? DrawUtil::SmoothBounce(t, 0.25, 40.0) : 1.0;
    DrawUtil::DrawPoint(window, p1, sf::Color::Red, pbounce*20.0);
  }
  if (mode >= MODE_LINE_ONLY) {
    const bool anim_line = (mode == MODE_LINE_ONLY || (mode == MODE_PT_AND_LINE && !animate_out));
    const sf::Color line_color(255, 0, 0, (anim_line ? uint8_t(255 * t) : 255));
    DrawUtil::DrawDualLine(window, p1, line_color, mode >= MODE_PERP);
  }

  return UpdateTMode(t, mode, 0.05, 0.05);
}

bool Animations::EllipseConstruction(sf::RenderTarget& window) {
  enum Mode {
    MODE_ELLIPSE_1,
    MODE_TANGENT_JOKE,
    MODE_ELLIPSE_2,
    MODE_NO_ELLIPSE,
    MODE_AXIS_A,
    MODE_AXIS_B,
    MODE_CENTER_C,
    MODE_SWEEP,
    MODE_HYPERBOLA,
    MODE_HYPERBOLA_ONLY,
    MODE_END,
  };
  static Mode mode = MODE_ELLIPSE_1;
  static double t = 0.0;
  static double theta = 0.0f;
  RUN_ONCE {
    unfiltered_points[0] = Eigen::Vector2d(0.0, 0.0);
    unfiltered_points[1] = Eigen::Vector2d(-0.5, -1.0);
    unfiltered_points[2] = Eigen::Vector2d(2.0, -1.0);
  }

  const Eigen::Vector2d a = moveable_points[1] - moveable_points[0];
  const Eigen::Vector2d b = moveable_points[2] - moveable_points[0];
  const Ellipse ellipse(a, b, moveable_points[0]);

  if (mode >= MODE_ELLIPSE_1 && mode < MODE_NO_ELLIPSE) {
    const bool anim_ellipse = (mode == MODE_ELLIPSE_1 && !animate_out) || (mode == MODE_ELLIPSE_2 && animate_out);
    DrawUtil::DrawConic(window, ellipse, sf::Color::White, false, anim_ellipse ? DrawUtil::Snappy(t) : 1.0);
  }
  if (mode == MODE_TANGENT_JOKE) {
    const Eigen::Vector2d pt = ellipse.PointAtAngle(theta);
    const Eigen::Vector2d dir = ellipse.VelocityAtAngle(theta).normalized();
    const sf::Color line_color(255, 0, 0, uint8_t(255 * t));
    DrawUtil::DrawLine(window, pt, pt + dir, line_color, true);
    DrawUtil::DrawPoint(window, pt, line_color);
  }
  if (mode >= MODE_AXIS_A) {
    const bool anim_line = (mode == MODE_AXIS_A && !animate_out);
    const sf::Color line_color(0, 255, 0, anim_line ? uint8_t(255 * t) : 255);
    DrawUtil::DrawLine(window, moveable_points[2], moveable_points[0], line_color, false);
  }
  if (mode >= MODE_AXIS_B) {
    const bool anim_line = (mode == MODE_AXIS_B && !animate_out);
    const sf::Color line_color(255, 0, 0, anim_line ? uint8_t(255 * t) : 255);
    DrawUtil::DrawLine(window, moveable_points[1], moveable_points[0], line_color, false);
  }
  if (mode >= MODE_CENTER_C) {
    const bool anim_pt = (mode == MODE_CENTER_C && !animate_out);
    const double pbounce = DrawUtil::SmoothBounce(t, 0.25, 40.0);
    DrawUtil::DrawPoint(window, moveable_points[0], sf::Color::White, 20.0 * (anim_pt ? pbounce : 1.0));
  }
  if (mode >= MODE_SWEEP) {
    const bool anim_ellipse = (mode == MODE_SWEEP && !animate_out);
    const double t2 = anim_ellipse ? t : 1.0f;
    const Eigen::Vector2d pt = ellipse.PointAtAngle(t2 * 2.0 * Ellipse::pi);
    const sf::Color line_color(255, 0, 0, uint8_t(255 * t2));
    if (mode == MODE_SWEEP) {
      DrawUtil::DrawLine(window, pt, moveable_points[0], sf::Color::Yellow, false);
    }
    DrawUtil::DrawConic(window, ellipse, sf::Color::White, false, t2);
    if (mode == MODE_SWEEP) {
      DrawUtil::DrawPoint(window, pt, sf::Color::Yellow);
    }
  }
  if (mode >= MODE_HYPERBOLA) {
    const bool anim_hyper = (mode == MODE_HYPERBOLA && !animate_out);
    const double t2 = anim_hyper ? t : 1.0f;
    const Eigen::Vector2d pt1 = ellipse.PointAtAngleH1(20.0 * (t2 - 0.5));
    const sf::Color line_color(255, 0, 0, uint8_t(255 * t2));
    if (mode == MODE_HYPERBOLA) {
      DrawUtil::DrawLine(window, pt1, moveable_points[0], sf::Color::Yellow, false);
    }
    DrawUtil::DrawConic(window, ellipse, sf::Color::White, true, t2);
    if (mode == MODE_HYPERBOLA) {
      DrawUtil::DrawPoint(window, pt1, sf::Color::Yellow);
    }
  }

  const double anim_in_speed = (mode == MODE_SWEEP ? 0.005 : (mode == MODE_HYPERBOLA ? 0.002 : 0.05));
  return UpdateTMode(t, mode, anim_in_speed, 0.05);
}

bool Animations::SkewAxes(sf::RenderTarget& window) {
  enum Mode {
    MODE_AXES,
    MODE_AUTO_ROTATE,
    MODE_END,
  };
  static Mode mode = MODE_AXES;
  static double t = 0.0;
  static double theta = 0.0f;
  RUN_ONCE {
    unfiltered_points[0] = Eigen::Vector2d(0.0, 0.0);
    unfiltered_points[1] = Eigen::Vector2d(-0.5, -1.0);
    unfiltered_points[2] = Eigen::Vector2d(2.0, -1.0);
  }

  const Eigen::Vector2d a = moveable_points[1] - moveable_points[0];
  const Eigen::Vector2d b = moveable_points[2] - moveable_points[0];
  const Ellipse ellipse(a, b, moveable_points[0]);

  if (mode >= MODE_AXES) {
    const bool anim_ellipse = (mode == MODE_AXES && !animate_out);
    const Eigen::Vector2d pt1 = ellipse.PointAtAngle(theta);
    const Eigen::Vector2d pt2 = ellipse.PointAtAngle(theta + 0.5*ellipse.pi);
    const sf::Color line_color1(0, 255, 0, anim_ellipse ? uint8_t(255 * t) : 255);
    const sf::Color line_color2(255, 0, 0, anim_ellipse ? uint8_t(255 * t) : 255);
    DrawUtil::DrawLine(window, pt1, moveable_points[0], line_color1, false);
    DrawUtil::DrawLine(window, pt2, moveable_points[0], line_color2, false);
    DrawUtil::DrawConic(window, ellipse, sf::Color::White, false, anim_ellipse ? DrawUtil::Snappy(t) : 1.0);
  }

  if (mode == MODE_AUTO_ROTATE) {
    theta += 0.02;
  }
  return UpdateTMode(t, mode, 0.05, 0.05);
}

bool Animations::HardestProblem1(sf::RenderTarget& window) {
  enum Mode {
    MODE_QUAD_ONLY,
    MODE_QUAD_WITH_CONIC,
    MODE_QUAD_ANIMATED,
    MODE_TRIVIAL_1,
    MODE_TRIVIAL_ANIM,
    MODE_TRIVIAL_2,
    MODE_QUAD_EXPAND,
    MODE_TRIVIAL_3,
    MODE_CENTERS,
    MODE_CENTER_LINE,
    MODE_WITH_ELLIPSE,
    MODE_JUST_POINT,
    MODE_JUST_POINT_AND_CONIC,
    MODE_END,
  };
  static Mode mode = MODE_QUAD_ONLY;
  static double t = 0.0;
  RUN_ONCE {
    unfiltered_points[0] = Eigen::Vector2d(-2.0025, 1.5375);
    unfiltered_points[1] = Eigen::Vector2d(-0.8025, -1.215);
    unfiltered_points[2] = Eigen::Vector2d(0.765, -0.4575);
    unfiltered_points[3] = Eigen::Vector2d(1.8225, 1.395);
    unfiltered_points[4] = Eigen::Vector2d(-0.6825, -1.2);
  }

  //Animations
  const double pbounce = DrawUtil::SmoothBounce(t, 0.25, 40.0);

  //Get time
  static double anim_t = 0.0;
  double s = std::sin(anim_t)*0.4 + 0.5;
  if (mode == MODE_TRIVIAL_ANIM) {
    s = 0.04*std::pow(0.5 - std::cos(anim_t)*0.499, 4);
  }

  //Get points
  const Eigen::Vector2d& p1 = moveable_points[0];
  const Eigen::Vector2d& p2 = moveable_points[1];
  const Eigen::Vector2d& p3 = moveable_points[2];
  const Eigen::Vector2d& p4 = moveable_points[3];

  //Get rays
  const Eigen::Vector2d r1 = p2 - p1;
  const Eigen::Vector2d r2 = p3 - p2;
  const Eigen::Vector2d r3 = p4 - p3;
  const Eigen::Vector2d r4 = p1 - p4;

  //Compute additional line intersections
  const Eigen::Vector2d p5 = Intersect(p1, r1, p3, r3);
  const Eigen::Vector2d p6 = Intersect(p1, r4, p3, r2);

  //Compute midpoints
  const Eigen::Vector2d m1 = (p1 + p3) * 0.5;
  const Eigen::Vector2d m2 = (p2 + p4) * 0.5;
  const Eigen::Vector2d m3 = (p5 + p6) * 0.5;

  //Compute midpoint line
  const Eigen::Vector2d L = m2 - m1;

  //Compute ellipse center
  const Eigen::Vector2d c = m1 + L*s;

  //Compute R matrix
  Eigen::Matrix3d R;
  R << OuterProd(r1), OuterProd(r2), OuterProd(r3);
  R.transposeInPlace();

  //Compute E vector
  const double e1 = OuterProd(r1).dot(OuterProdR(p1 - c));
  const double e2 = OuterProd(r2).dot(OuterProdR(p2 - c));
  const double e3 = OuterProd(r3).dot(OuterProdR(p3 - c));
  const Eigen::Vector3d E(e1, e2, e3);

  //Solve the linear part of the equation
  Eigen::Vector3d M = R.inverse() * E;

  //Solve the nonlinear part of the equation
  Eigen::Matrix2d matAB;
  matAB << M(2), -M(1), -M(1), M(0);
  const Eigen::JacobiSVD<Eigen::Matrix2d> svd(matAB, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Vector2d singularValues = svd.singularValues().cwiseAbs().cwiseSqrt();
  matAB = PermuteMatrix(svd.matrixU(), M) * singularValues.asDiagonal() * svd.matrixV().transpose();

  //Create the ellipse
  const bool is_hyperbola = (M(0)*M(2) < M(1)*M(1));
  Ellipse ellipse(Eigen::Vector2d(matAB(0, 0), matAB(0, 1)), Eigen::Vector2d(matAB(1, 0), matAB(1, 1)), c);

  //Draw trivial conics
  if (mode == MODE_TRIVIAL_1 || mode == MODE_CENTERS || mode == MODE_CENTER_LINE) {
    anim_t = 0.0;
    const bool is_anim = (mode == MODE_TRIVIAL_1 && !animate_out) || (mode == MODE_CENTERS && !animate_out) || (mode == MODE_CENTER_LINE && animate_out);
    const sf::Color anim_color(255, 255, 255, is_anim ? uint8_t(255 * t) : 255);
    DrawUtil::DrawLine(window, p1, p3, anim_color, false);
  }
  if (mode == MODE_TRIVIAL_2 || mode == MODE_CENTERS || mode == MODE_CENTER_LINE) {
    const bool is_anim = (mode == MODE_TRIVIAL_2) || (mode == MODE_CENTERS && !animate_out) || (mode == MODE_CENTER_LINE && animate_out);
    const sf::Color anim_color(255, 255, 255, is_anim ? uint8_t(255 * t) : 255);
    DrawUtil::DrawLine(window, p2, p4, anim_color, false);
  }
  if (mode == MODE_TRIVIAL_3 || mode == MODE_CENTERS || mode == MODE_CENTER_LINE) {
    const bool is_anim = (mode == MODE_TRIVIAL_3) || (mode == MODE_CENTERS && !animate_out) || (mode == MODE_CENTER_LINE && animate_out);
    const sf::Color anim_color(255, 255, 255, is_anim ? uint8_t(255 * t) : 255);
    DrawUtil::DrawLine(window, p5, p6, anim_color, false);
  }
  if (mode == MODE_TRIVIAL_ANIM) {
    const sf::Color anim_color(255, 255, 255, animate_out ? uint8_t(255 * t) : 255);
    DrawUtil::DrawConic(window, ellipse, sf::Color::White, is_hyperbola, 1.0);
  }

  //Draw center line
  if (mode >= MODE_CENTER_LINE) {
    const bool is_anim = (mode == MODE_CENTER_LINE && !animate_out);
    const sf::Color anim_color(0, 255, 255, is_anim ? uint8_t(255 * t) : 255);
    DrawUtil::DrawLine(window, m1, m2, anim_color, true, 6.0);
  }

  //Draw center line points
  if (mode >= MODE_CENTERS && mode < MODE_JUST_POINT) {
    DrawUtil::DrawPoint(window, m1, sf::Color::Cyan, 20.0 * ((mode == MODE_CENTERS && !animate_out) ? pbounce : 1.0));
    DrawUtil::DrawPoint(window, m2, sf::Color::Cyan, 20.0 * ((mode == MODE_CENTERS && !animate_out) ? pbounce : 1.0));
    DrawUtil::DrawPoint(window, m3, sf::Color::Cyan, 20.0 * ((mode == MODE_CENTERS && !animate_out) ? pbounce : 1.0));
  }

  //Draw the axes
  if (mode == MODE_WITH_ELLIPSE || mode == MODE_JUST_POINT_AND_CONIC) {
    const sf::Color anim_color1(255, 0, 0, uint8_t(255 * t));
    const sf::Color anim_color2(0, 255, 0, uint8_t(255 * t));
    DrawUtil::DrawAxes(window, ellipse, anim_color1, anim_color2);
    DrawUtil::DrawPoint(window, c, sf::Color::White, pbounce * 20.0);
  }

  //Draw center only
  if (mode == MODE_JUST_POINT) {
    anim_t = 0.0;
    DrawUtil::DrawPoint(window, c, sf::Color::White, pbounce * 20.0);
  }

  //Draw the main conic
  if (mode == MODE_QUAD_WITH_CONIC || mode == MODE_QUAD_ANIMATED || mode == MODE_WITH_ELLIPSE || mode == MODE_JUST_POINT_AND_CONIC) {
    const bool is_anim = (mode != MODE_QUAD_WITH_CONIC || !animate_out) && (mode != MODE_QUAD_ANIMATED || animate_out);
    DrawUtil::DrawConic(window, ellipse, sf::Color::White, is_hyperbola, is_anim ? DrawUtil::Snappy(t) : 1.0);
  }

  //Draw the quad
  if (mode >= MODE_QUAD_ONLY) {
    const bool anim_line = (mode == MODE_QUAD_ONLY && !animate_out);
    const sf::Color anim_color(255, 255, 0, anim_line ? uint8_t(255 * t) : 255);
    DrawUtil::DrawQuad(window, p1, p2, p3, p4, anim_color, false);
  }
  if (mode >= MODE_QUAD_EXPAND) {
    const bool anim_line = (mode == MODE_QUAD_EXPAND && !animate_out);
    const sf::Color anim_color(255, 255, 0, anim_line ? uint8_t(255 * t) : 255);
    DrawUtil::DrawQuad(window, p1, p2, p3, p4, anim_color, true);
    DrawUtil::DrawPoint(window, p5, sf::Color::Red, 12.0 * ((mode == MODE_QUAD_EXPAND && !animate_out) ? pbounce : 1.0));
    DrawUtil::DrawPoint(window, p6, sf::Color::Red, 12.0 * ((mode == MODE_QUAD_EXPAND && !animate_out) ? pbounce : 1.0));
  }

  //Draw the points
  DrawUtil::DrawPoint(window, p1, sf::Color::Red, 12.0 * ((mode == MODE_QUAD_ONLY && !animate_out) ? pbounce : 1.0));
  DrawUtil::DrawPoint(window, p2, sf::Color::Red, 12.0 * ((mode == MODE_QUAD_ONLY && !animate_out) ? pbounce : 1.0));
  DrawUtil::DrawPoint(window, p3, sf::Color::Red, 12.0 * ((mode == MODE_QUAD_ONLY && !animate_out) ? pbounce : 1.0));
  DrawUtil::DrawPoint(window, p4, sf::Color::Red, 12.0 * ((mode == MODE_QUAD_ONLY && !animate_out) ? pbounce : 1.0));

  if (mode == MODE_QUAD_ANIMATED || mode == MODE_TRIVIAL_ANIM || mode == MODE_WITH_ELLIPSE) {
    anim_t += 0.02;
  }
  return UpdateTMode(t, mode, 0.05, 0.05);
}

bool Animations::HardestProblem2(sf::RenderTarget& window) {
  enum Mode {
    MODE_SETUP,
    MODE_JUST_POINT,
    MODE_SKEW_CONIC,
    MODE_EMPTY,
    MODE_SVD_1,
    MODE_SVD_2,
    MODE_SVD_3,
    MODE_SVD_ANIM,
    MODE_SVD_FIXED,
    MODE_ORTHO_CONIC,
    MODE_END,
  };
  static Mode mode = MODE_SETUP;
  static double t = 0.0;
  static const double target_ang1 = 1.0;
  static const Eigen::Vector2d target_stretch(0.8, 1.7);
  static const double target_ang2 = -0.6;
  static double ang1 = 0.0;
  static Eigen::Vector2d stretch(1.0, 1.0);
  static double ang2 = 0.0;
  RUN_ONCE {
    unfiltered_points[0] = Eigen::Vector2d(-2.0025, 1.5375);
    unfiltered_points[1] = Eigen::Vector2d(-0.8025, -1.215);
    unfiltered_points[2] = Eigen::Vector2d(0.765, -0.4575);
    unfiltered_points[3] = Eigen::Vector2d(1.8225, 1.395);
    unfiltered_points[4] = Eigen::Vector2d(-0.6825, -1.2);
  }

  //Animations
  const double pbounce = DrawUtil::SmoothBounce(t, 0.25, 40.0);

  //Get points
  const Eigen::Vector2d& p1 = moveable_points[0];
  const Eigen::Vector2d& p2 = moveable_points[1];
  const Eigen::Vector2d& p3 = moveable_points[2];
  const Eigen::Vector2d& p4 = moveable_points[3];

  //Get rays
  const Eigen::Vector2d r1 = p2 - p1;
  const Eigen::Vector2d r2 = p3 - p2;
  const Eigen::Vector2d r3 = p4 - p3;
  const Eigen::Vector2d r4 = p1 - p4;

  //Compute additional line intersections
  const Eigen::Vector2d p5 = Intersect(p1, r1, p3, r3);
  const Eigen::Vector2d p6 = Intersect(p1, r4, p3, r2);

  //Compute midpoints
  const Eigen::Vector2d m1 = (p1 + p3) * 0.5;
  const Eigen::Vector2d m2 = (p2 + p4) * 0.5;
  const Eigen::Vector2d m3 = (p5 + p6) * 0.5;

  //Compute midpoint line
  const Eigen::Vector2d L = m2 - m1;

  //Get time
  double s = std::sin(double(frame) * 0.02)*0.4 + 0.5;
  if (mode == MODE_ORTHO_CONIC) {
    s = std::sin(double(frame) * 0.012)*0.9 + 0.5;
  }

  //Compute ellipse center
  const Eigen::Vector2d c = m1 + L*s;

  //Compute R matrix
  Eigen::Matrix3d R;
  R << OuterProd(r1), OuterProd(r2), OuterProd(r3);
  R.transposeInPlace();

  //Compute E vector
  const double e1 = OuterProd(r1).dot(OuterProdR(p1 - c));
  const double e2 = OuterProd(r2).dot(OuterProdR(p2 - c));
  const double e3 = OuterProd(r3).dot(OuterProdR(p3 - c));
  const Eigen::Vector3d E(e1, e2, e3);

  //Solve the linear part of the equation
  Eigen::Vector3d M = R.inverse() * E;

  //Solve the nonlinear part of the equation
  Eigen::Matrix2d matAB;
  matAB << M(2), -M(1), -M(1), M(0);
  const Eigen::JacobiSVD<Eigen::Matrix2d> svd(matAB, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Vector2d singularValues = svd.singularValues().cwiseAbs().cwiseSqrt();
  if (mode < MODE_ORTHO_CONIC) {
    matAB = svd.matrixU() * singularValues.asDiagonal() * svd.matrixV().transpose();
  } else {
    matAB = PermuteMatrix(svd.matrixU(), M) * singularValues.asDiagonal() * svd.matrixV().transpose();
  }

  //Create the ellipse
  const bool is_hyperbola = (M(0)*M(2) < M(1)*M(1));
  Ellipse ellipse(Eigen::Vector2d(matAB(0, 0), matAB(0, 1)), Eigen::Vector2d(matAB(1, 0), matAB(1, 1)), c);

  //Draw center line and quad
  if (mode <= MODE_SKEW_CONIC || mode == MODE_ORTHO_CONIC) {
    const bool is_anim = (mode == MODE_SETUP && !animate_out) || (mode == MODE_SKEW_CONIC && animate_out) || (mode == MODE_ORTHO_CONIC);
    const sf::Color anim_color1(0, 255, 255, is_anim ? uint8_t(255 * t) : 255);
    const sf::Color anim_color2(255, 255, 0, is_anim ? uint8_t(255 * t) : 255);
    DrawUtil::DrawLine(window, m1, m2, anim_color1, true, 6.0);
    DrawUtil::DrawQuad(window, p1, p2, p3, p4, anim_color2, true);
    DrawUtil::DrawPoint(window, p1, sf::Color::Red, (is_anim ? pbounce : 1.0) * 12.0);
    DrawUtil::DrawPoint(window, p2, sf::Color::Red, (is_anim ? pbounce : 1.0) * 12.0);
    DrawUtil::DrawPoint(window, p3, sf::Color::Red, (is_anim ? pbounce : 1.0) * 12.0);
    DrawUtil::DrawPoint(window, p4, sf::Color::Red, (is_anim ? pbounce : 1.0) * 12.0);
  }

  //Draw the ellipse
  if (mode == MODE_SKEW_CONIC || mode == MODE_ORTHO_CONIC) {
    DrawUtil::DrawAxes(window, ellipse, sf::Color::Red, sf::Color::Green, pbounce * 8.0);
    DrawUtil::DrawConic(window, ellipse, sf::Color::White, is_hyperbola,DrawUtil::Snappy(t));
  }

  //Draw the center of the ellipse
  if (mode == MODE_JUST_POINT || mode == MODE_SKEW_CONIC || mode == MODE_ORTHO_CONIC) {
    const bool is_anim = (mode == MODE_JUST_POINT && !animate_out) || (mode == MODE_SKEW_CONIC && animate_out) || (mode == MODE_ORTHO_CONIC);
    DrawUtil::DrawPoint(window, c, sf::Color::White, (is_anim ? pbounce : 1.0) * 20.0);
  }

  //Draw the first SVD
  Ellipse e;
  if (mode >= MODE_SVD_1 && mode <= MODE_SVD_FIXED) {
    if (t >= 1.0 && mode < MODE_SVD_ANIM) {
      static double alpha = 1.0;
      ang1 = ang1*alpha + target_ang1*(1.0 - alpha);
      alpha *= 0.997;
    } else if (mode == MODE_SVD_ANIM) {
      ang1 += 0.012;
    } else if (mode == MODE_SVD_FIXED) {
      ang1 = ang1*0.94 + 2.0*Ellipse::pi*0.06;
    }
    const bool is_anim = (mode == MODE_SVD_1 && !animate_out) || (mode == MODE_SVD_FIXED && animate_out);
    const sf::Color anim_color1(255, 255, 255, is_anim ? uint8_t(255 * t) : 255);
    e.v_a = Eigen::Vector2d(std::cos(ang1), std::sin(ang1));
    e.v_b = Eigen::Vector2d(std::sin(ang1), -std::cos(ang1));
    e.v_c = Eigen::Vector2d(-3.0f, 0.0f);
    DrawUtil::DrawAxes(window, e, sf::Color::Red, sf::Color::Green, (is_anim ? pbounce : 1.0) * 8.0);
    DrawUtil::DrawConic(window, e, anim_color1, false, 1.0);
    DrawUtil::DrawPoint(window, e.v_c, sf::Color::White, (is_anim ? pbounce : 1.0) * 20.0);
  }

  //Draw the second SVD
  if (mode >= MODE_SVD_2 && mode <= MODE_SVD_FIXED) {
    if (t >= 1.0) {
      static double alpha = 1.0;
      stretch = stretch*alpha + target_stretch*(1.0 - alpha);
      alpha *= 0.997;
    }
    const bool is_anim = (mode == MODE_SVD_2 && !animate_out) || (mode == MODE_SVD_FIXED && animate_out);
    const sf::Color anim_color1(255, 255, 255, is_anim ? uint8_t(255 * t) : 255);
    e.v_a = e.v_a.cwiseProduct(stretch);
    e.v_b = e.v_b.cwiseProduct(stretch);
    e.v_c = Eigen::Vector2d(0.0f, 0.0f);
    DrawUtil::DrawAxes(window, e, sf::Color::Red, sf::Color::Green, (is_anim ? pbounce : 1.0) * 8.0);
    DrawUtil::DrawConic(window, e, anim_color1, false, 1.0);
    DrawUtil::DrawPoint(window, e.v_c, sf::Color::White, (is_anim ? pbounce : 1.0) * 20.0);
  }

  //Draw the third SVD
  if (mode >= MODE_SVD_3 && mode <= MODE_SVD_FIXED) {
    if (t >= 1.0) {
      static double alpha = 1.0;
      ang2 = ang2*alpha + target_ang2*(1.0 - alpha);
      alpha *= 0.997;
    }
    const bool is_anim = (mode == MODE_SVD_3 && !animate_out) || (mode == MODE_SVD_FIXED && animate_out);
    const sf::Color anim_color1(255, 255, 255, is_anim ? uint8_t(255 * t) : 255);
    Eigen::Matrix2d rotm;
    rotm << std::cos(ang2), std::sin(ang2), -std::sin(ang2), std::cos(ang2);
    e.v_a = rotm * e.v_a;
    e.v_b = rotm * e.v_b;
    e.v_c = Eigen::Vector2d(3.0f, 0.0f);
    DrawUtil::DrawAxes(window, e, sf::Color::Red, sf::Color::Green, (is_anim ? pbounce : 1.0) * 8.0);
    DrawUtil::DrawConic(window, e, anim_color1, false, 1.0);
    DrawUtil::DrawPoint(window, e.v_c, sf::Color::White, (is_anim ? pbounce : 1.0) * 20.0);
  }

  return UpdateTMode(t, mode, 0.05, 0.05);
}

bool Animations::HardestProblem3(sf::RenderTarget& window) {
  enum Mode {
    MODE_WITH_C_SQR,
    MODE_PARABOLA,
    MODE_PARABOLA_TRIVIAL,
    MODE_PARABOLA2,
    MODE_PARABOLA_LINE,
    MODE_PARABOLA_MIN,
    MODE_WITHOUT_C_SQR,
    MODE_CUBIC,
    MODE_CUBIC_TRIVIAL,
    MODE_CUBIC_MAX,
    MODE_PLAIN,
    MODE_END,
  };
  static Mode mode = MODE_WITH_C_SQR;
  static const double intercept = 3.2;
  static double t = 0.0;
  RUN_ONCE {
    unfiltered_points[0] = Eigen::Vector2d(-2.0025, 1.5375);
    unfiltered_points[1] = Eigen::Vector2d(-0.8025, -1.215);
    unfiltered_points[2] = Eigen::Vector2d(0.765, -0.4575);
    unfiltered_points[3] = Eigen::Vector2d(1.8225, 1.395);
    unfiltered_points[4] = Eigen::Vector2d(-0.054375, 0.315);
  }

  //Animations
  const double pbounce = DrawUtil::SmoothBounce(t, 0.25, 40.0);

  //Get points
  const Eigen::Vector2d& p1 = moveable_points[0];
  const Eigen::Vector2d& p2 = moveable_points[1];
  const Eigen::Vector2d& p3 = moveable_points[2];
  const Eigen::Vector2d& p4 = moveable_points[3];

  //Get rays
  const Eigen::Vector2d r1 = p2 - p1;
  const Eigen::Vector2d r2 = p3 - p2;
  const Eigen::Vector2d r3 = p4 - p3;
  const Eigen::Vector2d r4 = p1 - p4;

  //Compute additional line intersections
  const Eigen::Vector2d p5 = Intersect(p1, r1, p3, r3);
  const Eigen::Vector2d p6 = Intersect(p1, r4, p3, r2);

  //Compute midpoints
  const Eigen::Vector2d m1 = (p1 + p3) * 0.5;
  const Eigen::Vector2d m2 = (p2 + p4) * 0.5;
  const Eigen::Vector2d m3 = (p5 + p6) * 0.5;

  //Compute midpoint line
  const Eigen::Vector2d L = m2 - m1;

  //Calculate trivial c-squared values
  const double cSqr_trivial1 = (p1 - p3).squaredNorm() * 0.25;
  const double cSqr_trivial2 = (p2 - p4).squaredNorm() * 0.25;
  const double cSqr_trivial3 = (p5 - p6).squaredNorm() * 0.25;

  //Calculate the t values for the trivial solutions
  const double t_trivial1 = 0.0;
  const double t_trivial2 = 1.0;
  const double t_trivial3 = (m3.x() - m1.x()) / (m2.x() - m1.x());

  //Solve for the c-squared quadratic
  Eigen::Matrix3d cSqr_A;
  cSqr_A << t_trivial1*t_trivial1, t_trivial1, 1.0,
            t_trivial2*t_trivial2, t_trivial2, 1.0,
            t_trivial3*t_trivial3, t_trivial3, 1.0;
  const Eigen::Vector3d cSqr_y(cSqr_trivial1, cSqr_trivial2, cSqr_trivial3);
  const Eigen::Vector3d cSqr_quadratic = cSqr_A.inverse() * cSqr_y;

  //Find the minimum c-squared t value
  const double cSqr_min_t = -0.5 * cSqr_quadratic[1] / cSqr_quadratic[0];

  //Find the maximum area t value
  const double area_A = 3.0;
  const double area_B = -2.0*(t_trivial1 + t_trivial2 + t_trivial3);
  const double area_C = t_trivial1*t_trivial2 + t_trivial2*t_trivial3 + t_trivial3*t_trivial1;
  const double area_t1 = (-area_B + std::sqrt(area_B*area_B - 4.0*area_A*area_C)) / (2.0 * area_A);
  const double area_t2 = (-area_B - std::sqrt(area_B*area_B - 4.0*area_A*area_C)) / (2.0 * area_A);
  const double area_t = (area_t1 >= t_trivial1 && area_t1 <= t_trivial2 ? area_t1 : area_t2);

  //Compute center from the movable point
  const double s = (moveable_points[4].x() - m1.x()) / (m2.x() - m1.x());
  const Eigen::Vector2d c = m1 + L*s;
  unfiltered_points[4] = m1 + L*(unfiltered_points[4].x() - m1.x()) / (m2.x() - m1.x());

  //Compute R matrix
  Eigen::Matrix3d R;
  R << OuterProd(r1), OuterProd(r2), OuterProd(r3);
  R.transposeInPlace();

  //Compute E vector
  const double e1 = OuterProd(r1).dot(OuterProdR(p1 - c));
  const double e2 = OuterProd(r2).dot(OuterProdR(p2 - c));
  const double e3 = OuterProd(r3).dot(OuterProdR(p3 - c));
  const Eigen::Vector3d E(e1, e2, e3);

  //Solve the linear part of the equation
  Eigen::Vector3d M = R.inverse() * E;

  //Solve the nonlinear part of the equation
  Eigen::Matrix2d matAB;
  matAB << M(2), -M(1), -M(1), M(0);
  const Eigen::JacobiSVD<Eigen::Matrix2d> svd(matAB, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Vector2d singularValues = svd.singularValues().cwiseAbs().cwiseSqrt();
  matAB = PermuteMatrix(svd.matrixU(), M) * singularValues.asDiagonal() * svd.matrixV().transpose();

  //Create the ellipse
  const bool is_hyperbola = (M(0)*M(2) < M(1)*M(1));
  Ellipse ellipse(Eigen::Vector2d(matAB(0, 0), matAB(0, 1)), Eigen::Vector2d(matAB(1, 0), matAB(1, 1)), c);

  //Draw center line
  if (mode >= MODE_WITH_C_SQR) {
    //Animation
    const bool is_anim = (mode == MODE_WITH_C_SQR && !animate_out);
    const bool anim_fade = (mode == MODE_PARABOLA && !animate_out);
    const int max_opacity = (mode >= MODE_PARABOLA && mode < MODE_PLAIN) ? (anim_fade ? int(255 - t*192) : 64) : 255;

    //Colors
    const sf::Color anim_red(255, 0, 0, is_anim ? uint8_t(255 * t) : max_opacity);
    const sf::Color anim_green(0, 255, 0, is_anim ? uint8_t(255 * t) : max_opacity);
    const sf::Color anim_cyan(0, 255, 255, is_anim ? uint8_t(255 * t) : max_opacity);
    const sf::Color anim_white(255, 255, 255, is_anim ? uint8_t(255 * t) : max_opacity);
    const sf::Color anim_white_full(255, 255, 255, is_anim ? uint8_t(255 * t) : 255);
    const sf::Color anim_yellow(255, 255, 0, is_anim ? uint8_t(255 * t) : max_opacity);
    const sf::Color anim_yellow_full(255, 255, 0, is_anim ? uint8_t(255 * t) : 255);

    //Draw the center line
    DrawUtil::DrawLine(window, m1, m2, anim_cyan, true, 6.0);
 
    //Draw some solutions
    if (mode == MODE_PARABOLA_TRIVIAL) {
      //Draw trivial conics
      const sf::Color anim_trivial_yellow(255, 255, 0, uint8_t(255 * t));
      DrawUtil::DrawLine(window, p1, p3, anim_trivial_yellow, false);
      DrawUtil::DrawLine(window, p2, p4, anim_trivial_yellow, false);
      DrawUtil::DrawLine(window, p5, p6, anim_trivial_yellow, false);
      DrawUtil::DrawPoint(window, m1, anim_trivial_yellow, 12.0);
      DrawUtil::DrawPoint(window, m2, anim_trivial_yellow, 12.0);
      DrawUtil::DrawPoint(window, m3, anim_trivial_yellow, 12.0);
    }

    //Draw the ellipse
    DrawUtil::DrawConic(window, ellipse, anim_white_full, is_hyperbola, 1.0);

    //Draw ellipse axes
    if (mode <= MODE_WITHOUT_C_SQR) {
      DrawUtil::DrawLine(window, ellipse.v_c + ellipse.v_a, ellipse.v_c + ellipse.v_b, is_hyperbola ? anim_yellow : anim_yellow_full, false);
    }
    DrawUtil::DrawAxes(window, ellipse, anim_red, anim_green);
    DrawUtil::DrawPoint(window, c, anim_white_full, 20.0);

    //Draw the quad
    DrawUtil::DrawQuad(window, p1, p2, p3, p4, anim_yellow, true);

    //Draw the quad points
    DrawUtil::DrawPoint(window, p1, anim_red, 12.0);
    DrawUtil::DrawPoint(window, p2, anim_red, 12.0);
    DrawUtil::DrawPoint(window, p3, anim_red, 12.0);
    DrawUtil::DrawPoint(window, p4, anim_red, 12.0);
  }

  if (mode >= MODE_PARABOLA && mode < MODE_PLAIN) {
    //Fade colors
    const bool is_anim = ((mode == MODE_PARABOLA || mode == MODE_CUBIC) && !animate_out) || ((mode == MODE_WITHOUT_C_SQR || mode == MODE_CUBIC_MAX) && animate_out);
    const sf::Color anim_pink(255, 0, 255, is_anim ? uint8_t(255 * t) : 255);

    //Draw the graph
    static const int GRAPH_RES = 512;
    const double y_scale = (mode < MODE_CUBIC ? 0.2 : 2.0);
    const double y_offset = 0.0;
    const double x_min = DrawUtil::center.x() - 0.5 * double(window.getSize().x) / DrawUtil::scale;
    const double x_max = DrawUtil::center.x() + 0.5 * double(window.getSize().x) / DrawUtil::scale;
    Eigen::Vector2d prev_pt;
    for (int i = 0; i < GRAPH_RES + 1; ++i) {
      //Get x coordinate for graph
      const double alpha = double(i) / GRAPH_RES;
      const double x = x_min + alpha*(x_max - x_min);
      const double t_x = (x - m1.x()) / (m2.x() - m1.x());

      //Get y coordinate for graph
      double y = 0.0;
      if (mode < MODE_CUBIC) {
        y = y_offset - y_scale*(cSqr_quadratic[0] * t_x * t_x + cSqr_quadratic[1] * t_x + cSqr_quadratic[2]);
      } else {
        y = y_offset - y_scale*(t_x - t_trivial1)*(t_x - t_trivial2)*(t_x - t_trivial3);
      }

      //Draw line
      const Eigen::Vector2d new_pt(x, y);
      if (i > 0) {
        DrawUtil::DrawLine(window, prev_pt, new_pt, anim_pink, false, 16.0);
      }
      prev_pt = new_pt;
    }

    //Draw solutions
    if (mode == MODE_PARABOLA_TRIVIAL) {
      //Draw trivial points
      const sf::Color anim_yellow(255, 255, 0, uint8_t(255 * t));
      DrawUtil::DrawPoint(window, Eigen::Vector2d(m1.x(), y_offset - y_scale * cSqr_trivial1), anim_yellow, 20.0);
      DrawUtil::DrawPoint(window, Eigen::Vector2d(m2.x(), y_offset - y_scale * cSqr_trivial2), anim_yellow, 20.0);
      DrawUtil::DrawPoint(window, Eigen::Vector2d(m3.x(), y_offset - y_scale * cSqr_trivial3), anim_yellow, 20.0);
    } else if (mode == MODE_PARABOLA_LINE) {
      //Draw an intersection line
      const sf::Color anim_yellow(255, 255, 0, uint8_t(255 * t));
      const double y = y_offset - y_scale * intercept;
      DrawUtil::DrawLine(window, Eigen::Vector2d(-1.0, y), Eigen::Vector2d(1.0, y), anim_yellow, true);

      //Solve for interception points
      const double d = (cSqr_quadratic[1]*cSqr_quadratic[1] - 4.0*cSqr_quadratic[0]*(cSqr_quadratic[2] - intercept));
      if (d >= 0.0) {
        //Solve quadratic equation
        const double t1 = (-cSqr_quadratic[1] + std::sqrt(d)) / (2.0 * cSqr_quadratic[0]);
        const double t2 = (-cSqr_quadratic[1] - std::sqrt(d)) / (2.0 * cSqr_quadratic[0]);
        const double x1 = m1.x() + t1*(m2.x() - m1.x());
        const double x2 = m1.x() + t2*(m2.x() - m1.x());

        //Draw intersection points
        DrawUtil::DrawPoint(window, Eigen::Vector2d(x1, y), anim_yellow, 20.0);
        DrawUtil::DrawPoint(window, Eigen::Vector2d(x2, y), anim_yellow, 20.0);
      }
    } else if (mode == MODE_PARABOLA_MIN) {
      //Draw the minimum point
      const sf::Color anim_yellow(255, 255, 0, uint8_t(255 * t));
      const double x = m1.x() + cSqr_min_t*(m2.x() - m1.x());
      const double csqr = (cSqr_quadratic[0] * cSqr_min_t * cSqr_min_t + cSqr_quadratic[1] * cSqr_min_t + cSqr_quadratic[2]);
      DrawUtil::DrawPoint(window, Eigen::Vector2d(x, y_offset - y_scale * csqr), anim_yellow, 20.0);
    } else if (mode == MODE_CUBIC_TRIVIAL) {
      //Draw trivial points
      const sf::Color anim_yellow(255, 255, 0, uint8_t(255 * t));
      DrawUtil::DrawLine(window, Eigen::Vector2d(-1.0, y_offset), Eigen::Vector2d(1.0, y_offset), anim_yellow, true);
      DrawUtil::DrawPoint(window, Eigen::Vector2d(m1.x(), y_offset), anim_yellow, 20.0);
      DrawUtil::DrawPoint(window, Eigen::Vector2d(m2.x(), y_offset), anim_yellow, 20.0);
      DrawUtil::DrawPoint(window, Eigen::Vector2d(m3.x(), y_offset), anim_yellow, 20.0);
    } else if (mode == MODE_CUBIC_MAX) {
      //Draw the maximum point
      const sf::Color anim_yellow(255, 255, 0, uint8_t(255 * t));
      const double x = m1.x() + area_t*(m2.x() - m1.x());
      const double asqr = (area_t - t_trivial1)*(area_t - t_trivial2)*(area_t - t_trivial3);
      DrawUtil::DrawPoint(window, Eigen::Vector2d(x, y_offset - y_scale * asqr), anim_yellow, 20.0);
    }

    //Draw the current point as well
    if (true) {
      double y;
      const double x = c.x();
      const double t_x = (x - m1.x()) / (m2.x() - m1.x());
      if (mode < MODE_CUBIC) {
        y = y_offset - y_scale*(cSqr_quadratic[0] * t_x * t_x + cSqr_quadratic[1] * t_x + cSqr_quadratic[2]);
      } else {
        y = y_offset - y_scale*(t_x - t_trivial1)*(t_x - t_trivial2)*(t_x - t_trivial3);
      }
      DrawUtil::DrawPoint(window, Eigen::Vector2d(x, y), anim_pink, 20.0);
    }
  }

  return UpdateTMode(t, mode, 0.05, 0.05);
}

bool Animations::FivePointSolver(sf::RenderTarget& window) {
  enum Mode {
    MODE_POINTS,
    MODE_WITH_CONIC,
    MODE_END,
  };
  static Mode mode = MODE_POINTS;
  static double t = 0.0;
  RUN_ONCE {
    unfiltered_points[0] = Eigen::Vector2d(-0.81, -1.7475);
    unfiltered_points[1] = Eigen::Vector2d(-2.19, 0.2175);
    unfiltered_points[2] = Eigen::Vector2d(1.7475, 1.7925);
    unfiltered_points[3] = Eigen::Vector2d(2.1375, -0.21);
    unfiltered_points[4] = Eigen::Vector2d(-1.6575, 0.7575);
  }

  //Get points
  const Eigen::Vector2d& p1 = moveable_points[0];
  const Eigen::Vector2d& p2 = moveable_points[1];
  const Eigen::Vector2d& p3 = moveable_points[2];
  const Eigen::Vector2d& p4 = moveable_points[3];
  const Eigen::Vector2d& p5 = moveable_points[4];

  //Add points to the solver
  ConicSolver s;
  s.AddPoint(p1);
  s.AddPoint(p2);
  s.AddPoint(p3);
  s.AddPoint(p4);
  s.AddPoint(p5);

  //Solve for the conic
  double c1, c2, c3, c4, c5, c6;
  s.Solve(c1, c2, c3, c4, c5, c6);

  //Create the conic
  Ellipse ellipse;
  const bool is_hyperbola = ellipse.FromConic(c1, c2, c3, c4, c5, c6);

  //Draw the result
  const double bounce = (mode == MODE_POINTS && !animate_out) ? DrawUtil::SmoothBounce(t, 0.25, 40.0) : 1.0;
  if (mode == MODE_WITH_CONIC) {
    DrawUtil::DrawConic(window, ellipse, sf::Color::White, is_hyperbola, t);
  }
  DrawUtil::DrawPoint(window, p1, sf::Color::Red, 20.0 * bounce);
  DrawUtil::DrawPoint(window, p2, sf::Color::Red, 20.0 * bounce);
  DrawUtil::DrawPoint(window, p3, sf::Color::Red, 20.0 * bounce);
  DrawUtil::DrawPoint(window, p4, sf::Color::Red, 20.0 * bounce);
  DrawUtil::DrawPoint(window, p5, sf::Color::Red, 20.0 * bounce);

  return UpdateTMode(t, mode, 0.02, 0.04);
}

bool Animations::FourPointSolver(sf::RenderTarget& window) {
  enum Mode {
    MODE_EMPTY,
    MODE_SOL1,
    MODE_SOL2,
    MODE_END,
  };
  static Mode mode = MODE_EMPTY;
  static double t = 0.0;
  RUN_ONCE {
    unfiltered_points[0] = Eigen::Vector2d(0.8775, -0.945);
    unfiltered_points[1] = Eigen::Vector2d(-1.14, 0.195);
    unfiltered_points[2] = Eigen::Vector2d(0.3075, 1.2225);
    unfiltered_points[3] = Eigen::Vector2d(1.5, 0);
    unfiltered_points[4] = Eigen::Vector2d(-0.2025, -0.5625);
  }

  //Get points
  const Eigen::Vector2d& p1 = moveable_points[0];
  const Eigen::Vector2d& p2 = moveable_points[1];
  const Eigen::Vector2d& p3 = moveable_points[2];
  const Eigen::Vector2d& p4 = moveable_points[3];
  const Eigen::Vector2d& p5 = moveable_points[4];

  const Eigen::Vector2d pinv = -p5 / p5.squaredNorm();
  const Eigen::Vector2d pperp(p5.y(), -p5.x());
  
  const double t12 = IntersectT(pinv, pperp, p1, p2 - p1);
  const double t23 = IntersectT(pinv, pperp, p2, p3 - p2);
  const double t34 = IntersectT(pinv, pperp, p3, p4 - p3);
  const double t41 = IntersectT(pinv, pperp, p4, p1 - p4);

  const double qt = (t12*t34 - t23*t41) / (t12 + t34 - t23 - t41);
  const double r2 = (t12 - qt) * (t34 - qt);
  const double st1 = qt + std::sqrt(std::abs(r2));
  const double st2 = qt - std::sqrt(std::abs(r2));

  //Add points to the solver
  ConicSolver s1;
  s1.AddPoint(p1);
  s1.AddPoint(p2);
  s1.AddPoint(p3);
  s1.AddPoint(p4);
  ConicSolver s2 = s1;
  s1.AddPoint(pinv + st1*pperp);
  s2.AddPoint(pinv + st2*pperp);

  //Solve for the conic
  Ellipse ellipse1, ellipse2;
  double c1, c2, c3, c4, c5, c6;
  s1.Solve(c1, c2, c3, c4, c5, c6);
  const bool is_hyperbola1 = ellipse1.FromConic(c1, c2, c3, c4, c5, c6);
  s2.Solve(c1, c2, c3, c4, c5, c6);
  const bool is_hyperbola2 = ellipse2.FromConic(c1, c2, c3, c4, c5, c6);

  //Draw the result
  const bool anim_pt = (mode == MODE_EMPTY && !animate_out) || (mode == MODE_SOL2 && animate_out);
  DrawUtil::DrawDualLine(window, p5, sf::Color(0, 255, 0, anim_pt ? uint8_t(255 * t) : 255));
  const double bounce = (animate_out ? 1.0 : DrawUtil::SmoothBounce(t, 0.25, 40.0));
  const double snap = DrawUtil::Snappy(t);
  if (r2 > 0.0) {
    if (mode >= MODE_SOL1) {
      const bool anim_conic = (mode == MODE_SOL1 && !animate_out) || (mode == MODE_SOL2 && animate_out);
      DrawUtil::DrawConic(window, ellipse1, sf::Color::White, is_hyperbola1, anim_conic ? snap : 1.0);
    }
    if (mode >= MODE_SOL2) {
      DrawUtil::DrawConic(window, ellipse2, sf::Color::White, is_hyperbola2, (mode == MODE_SOL2) ? snap : 1.0);
    }
  }
  DrawUtil::DrawPoint(window, p1, sf::Color::Red, 20.0 * (anim_pt ? bounce : 1.0));
  DrawUtil::DrawPoint(window, p2, sf::Color::Red, 20.0 * (anim_pt ? bounce : 1.0));
  DrawUtil::DrawPoint(window, p3, sf::Color::Red, 20.0 * (anim_pt ? bounce : 1.0));
  DrawUtil::DrawPoint(window, p4, sf::Color::Red, 20.0 * (anim_pt ? bounce : 1.0));
  //DrawUtil::DrawCircle(window, p5, sf::Color::Green, 18.0 * (anim_pt ? bounce : 1.0), 4.0);

  return UpdateTMode(t, mode, 0.02, 0.04);
}

bool Animations::ThreePointSolver(sf::RenderTarget& window) {
  enum Mode {
    MODE_EMPTY,
    MODE_SOL1,
    MODE_SOL2,
    MODE_SOL3,
    MODE_SOL4,
    MODE_END,
  };
  static Mode mode = MODE_EMPTY;
  static double t = 0.0;
  RUN_ONCE {
    unfiltered_points[0] = Eigen::Vector2d(0.6075, -0.195);
    unfiltered_points[1] = Eigen::Vector2d(-1.14, 0.195);
    unfiltered_points[2] = Eigen::Vector2d(0.3075, 1.2225);
    unfiltered_points[3] = Eigen::Vector2d(0.69, 0.2325);
    unfiltered_points[4] = Eigen::Vector2d(-0.1575, -0.57);
  }

  //Get points
  const Eigen::Vector2d& p1 = moveable_points[0];
  const Eigen::Vector2d& p2 = moveable_points[1];
  const Eigen::Vector2d& p3 = moveable_points[2];
  const Eigen::Vector2d& p4 = moveable_points[3];
  const Eigen::Vector2d& p5 = moveable_points[4];

  const Eigen::Vector2d p4inv = -p4 / p4.squaredNorm();
  const Eigen::Vector2d p4perp(p4.y(), -p4.x());
  const Eigen::Vector2d p5inv = -p5 / p5.squaredNorm();
  const Eigen::Vector2d p5perp(p5.y(), -p5.x());

  const double id1 = IntersectT(p2, p3 - p2, p4inv, p4perp);
  const double ie1 = IntersectT(p2, p3 - p2, p5inv, p5perp);
  const double qt1 = (id1* ie1) / (id1 + ie1 - 1.0);
  const double rSq1 = (id1 - qt1) * (ie1 - qt1);

  const double id2 = IntersectT(p3, p1 - p3, p4inv, p4perp);
  const double ie2 = IntersectT(p3, p1 - p3, p5inv, p5perp);
  const double qt2 = (id2 * ie2) / (id2 + ie2 - 1.0);
  const double rSq2 = (id2 - qt2) * (ie2 - qt2);

  const Eigen::Vector2d n1 = p2 + (p3 - p2)*(qt1 + std::sqrt(std::abs(rSq1)));
  const Eigen::Vector2d n2 = p2 + (p3 - p2)*(qt1 - std::sqrt(std::abs(rSq1)));
  const Eigen::Vector2d n3 = p3 + (p1 - p3)*(qt2 + std::sqrt(std::abs(rSq2)));
  const Eigen::Vector2d n4 = p3 + (p1 - p3)*(qt2 - std::sqrt(std::abs(rSq2)));

  //Add points to the solver
  ConicSolver s1;
  s1.AddPoint(p1);
  s1.AddPoint(p2);
  s1.AddPoint(p3);
  ConicSolver s2 = s1;
  s1.AddPoint(Intersect(p4inv, p4perp, n1, n3 - n1));
  s1.AddPoint(Intersect(p5inv, p5perp, n1, n3 - n1));
  ConicSolver s3 = s2;
  s2.AddPoint(Intersect(p4inv, p4perp, n1, n4 - n1));
  s2.AddPoint(Intersect(p5inv, p5perp, n1, n4 - n1));
  ConicSolver s4 = s3;
  s3.AddPoint(Intersect(p4inv, p4perp, n2, n3 - n2));
  s3.AddPoint(Intersect(p5inv, p5perp, n2, n3 - n2));
  s4.AddPoint(Intersect(p4inv, p4perp, n2, n4 - n2));
  s4.AddPoint(Intersect(p5inv, p5perp, n2, n4 - n2));

  //Solve for the conic
  Ellipse ellipse1, ellipse2, ellipse3, ellipse4;
  double c1, c2, c3, c4, c5, c6;
  s1.Solve(c1, c2, c3, c4, c5, c6);
  const bool is_hyperbola1 = ellipse1.FromConic(c1, c2, c3, c4, c5, c6);
  s2.Solve(c1, c2, c3, c4, c5, c6);
  const bool is_hyperbola2 = ellipse2.FromConic(c1, c2, c3, c4, c5, c6);
  s3.Solve(c1, c2, c3, c4, c5, c6);
  const bool is_hyperbola3 = ellipse3.FromConic(c1, c2, c3, c4, c5, c6);
  s4.Solve(c1, c2, c3, c4, c5, c6);
  const bool is_hyperbola4 = ellipse4.FromConic(c1, c2, c3, c4, c5, c6);

  //Draw the result
  const bool anim_pt = (mode == MODE_EMPTY && !animate_out) || (mode == MODE_SOL4 && animate_out);
  sf::Color lineCol(0, (anim_pt ? uint8_t(255 * t) : 255), 0);
  DrawUtil::DrawDualLine(window, p4, lineCol);
  DrawUtil::DrawDualLine(window, p5, lineCol);
  const double bounce = (animate_out ? 1.0 : DrawUtil::SmoothBounce(t, 0.25, 40.0));
  const double snap = DrawUtil::Snappy(t);
  if (rSq1 > 0.0 && rSq2 > 0.0) {
    if (mode >= MODE_SOL1) {
      const bool anim_conic = (mode == MODE_SOL1 && !animate_out) || (mode == MODE_SOL4 && animate_out);
      DrawUtil::DrawConic(window, ellipse1, sf::Color::White, is_hyperbola1, anim_conic ? snap : 1.0);
    }
    if (mode >= MODE_SOL2) {
      const bool anim_conic = (mode == MODE_SOL2 && !animate_out) || (mode == MODE_SOL4 && animate_out);
      DrawUtil::DrawConic(window, ellipse2, sf::Color::White, is_hyperbola2, anim_conic ? snap : 1.0);
    }
    if (mode >= MODE_SOL3) {
      const bool anim_conic = (mode == MODE_SOL3 && !animate_out) || (mode == MODE_SOL4 && animate_out);
      DrawUtil::DrawConic(window, ellipse3, sf::Color::White, is_hyperbola3, anim_conic ? snap : 1.0);
    }
    if (mode >= MODE_SOL4) {
      const bool anim_conic = (mode == MODE_SOL4);
      DrawUtil::DrawConic(window, ellipse4, sf::Color::White, is_hyperbola4, anim_conic ? snap : 1.0);
    }
  }
  DrawUtil::DrawPoint(window, p1, sf::Color::Red, 20.0 * (anim_pt ? bounce : 1.0));
  DrawUtil::DrawPoint(window, p2, sf::Color::Red, 20.0 * (anim_pt ? bounce : 1.0));
  DrawUtil::DrawPoint(window, p3, sf::Color::Red, 20.0 * (anim_pt ? bounce : 1.0));

  return UpdateTMode(t, mode, 0.02, 0.04);
}

bool Animations::ThreeTangentSolver(sf::RenderTarget& window) {
  enum Mode {
    MODE_EMPTY,
    MODE_SOL1,
    MODE_SOL2,
    MODE_SOL3,
    MODE_SOL4,
    MODE_END,
  };
  static Mode mode = MODE_EMPTY;
  static double t = 0.0;
  RUN_ONCE {
    unfiltered_points[0] = Eigen::Vector2d(0.405, -0.48);
    unfiltered_points[1] = Eigen::Vector2d(-0.5925, -0.39);
    unfiltered_points[2] = Eigen::Vector2d(0.0975, 0.51);
    unfiltered_points[3] = Eigen::Vector2d(0.69, 0.2325);
    unfiltered_points[4] = Eigen::Vector2d(-0.6825, -1.2);
  }

  //Get points
  const Eigen::Vector2d& p1 = moveable_points[0];
  const Eigen::Vector2d& p2 = moveable_points[1];
  const Eigen::Vector2d& p3 = moveable_points[2];
  const Eigen::Vector2d& p4 = moveable_points[3];
  const Eigen::Vector2d& p5 = moveable_points[4];

  const Eigen::Vector2d p4inv = -p4 / p4.squaredNorm();
  const Eigen::Vector2d p4perp(p4.y(), -p4.x());
  const Eigen::Vector2d p5inv = -p5 / p5.squaredNorm();
  const Eigen::Vector2d p5perp(p5.y(), -p5.x());

  const double id1 = IntersectT(p2, p3 - p2, p4inv, p4perp);
  const double ie1 = IntersectT(p2, p3 - p2, p5inv, p5perp);
  const double qt1 = (id1* ie1) / (id1 + ie1 - 1.0);
  const double rSq1 = (id1 - qt1) * (ie1 - qt1);

  const double id2 = IntersectT(p3, p1 - p3, p4inv, p4perp);
  const double ie2 = IntersectT(p3, p1 - p3, p5inv, p5perp);
  const double qt2 = (id2 * ie2) / (id2 + ie2 - 1.0);
  const double rSq2 = (id2 - qt2) * (ie2 - qt2);

  const Eigen::Vector2d n1 = p2 + (p3 - p2)*(qt1 + std::sqrt(std::abs(rSq1)));
  const Eigen::Vector2d n2 = p2 + (p3 - p2)*(qt1 - std::sqrt(std::abs(rSq1)));
  const Eigen::Vector2d n3 = p3 + (p1 - p3)*(qt2 + std::sqrt(std::abs(rSq2)));
  const Eigen::Vector2d n4 = p3 + (p1 - p3)*(qt2 - std::sqrt(std::abs(rSq2)));

  //Add points to the solver
  ConicSolver s1;
  s1.AddPoint(p1);
  s1.AddPoint(p2);
  s1.AddPoint(p3);
  ConicSolver s2 = s1;
  s1.AddPoint(Intersect(p4inv, p4perp, n1, n3 - n1));
  s1.AddPoint(Intersect(p5inv, p5perp, n1, n3 - n1));
  ConicSolver s3 = s2;
  s2.AddPoint(Intersect(p4inv, p4perp, n1, n4 - n1));
  s2.AddPoint(Intersect(p5inv, p5perp, n1, n4 - n1));
  ConicSolver s4 = s3;
  s3.AddPoint(Intersect(p4inv, p4perp, n2, n3 - n2));
  s3.AddPoint(Intersect(p5inv, p5perp, n2, n3 - n2));
  s4.AddPoint(Intersect(p4inv, p4perp, n2, n4 - n2));
  s4.AddPoint(Intersect(p5inv, p5perp, n2, n4 - n2));

  //Solve for the conic
  Ellipse ellipse1, ellipse2, ellipse3, ellipse4;
  double c1, c2, c3, c4, c5, c6;
  s1.SolveDual(c1, c2, c3, c4, c5, c6);
  const bool is_hyperbola1 = ellipse1.FromConic(c1, c2, c3, c4, c5, c6);
  s2.SolveDual(c1, c2, c3, c4, c5, c6);
  const bool is_hyperbola2 = ellipse2.FromConic(c1, c2, c3, c4, c5, c6);
  s3.SolveDual(c1, c2, c3, c4, c5, c6);
  const bool is_hyperbola3 = ellipse3.FromConic(c1, c2, c3, c4, c5, c6);
  s4.SolveDual(c1, c2, c3, c4, c5, c6);
  const bool is_hyperbola4 = ellipse4.FromConic(c1, c2, c3, c4, c5, c6);

  //Draw the result
  const bool anim_pt = (mode == MODE_EMPTY && !animate_out) || (mode == MODE_SOL4 && animate_out);
  sf::Color lineCol(0, (anim_pt ? uint8_t(255 * t) : 255), 0);
  DrawUtil::DrawDualLine(window, p1, lineCol);
  DrawUtil::DrawDualLine(window, p2, lineCol);
  DrawUtil::DrawDualLine(window, p3, lineCol);
  const double bounce = (animate_out ? 1.0 : DrawUtil::SmoothBounce(t, 0.25, 40.0));
  const double snap = DrawUtil::Snappy(t);
  if (rSq1 > 0.0 && rSq2 > 0.0) {
    if (mode >= MODE_SOL1) {
      const bool anim_conic = (mode == MODE_SOL1 && !animate_out) || (mode == MODE_SOL4 && animate_out);
      DrawUtil::DrawConic(window, ellipse1, sf::Color::White, is_hyperbola1, anim_conic ? snap : 1.0);
    }
    if (mode >= MODE_SOL2) {
      const bool anim_conic = (mode == MODE_SOL2 && !animate_out) || (mode == MODE_SOL4 && animate_out);
      DrawUtil::DrawConic(window, ellipse2, sf::Color::White, is_hyperbola2, anim_conic ? snap : 1.0);
    }
    if (mode >= MODE_SOL3) {
      const bool anim_conic = (mode == MODE_SOL3 && !animate_out) || (mode == MODE_SOL4 && animate_out);
      DrawUtil::DrawConic(window, ellipse3, sf::Color::White, is_hyperbola3, anim_conic ? snap : 1.0);
    }
    if (mode >= MODE_SOL4) {
      const bool anim_conic = (mode == MODE_SOL4);
      DrawUtil::DrawConic(window, ellipse4, sf::Color::White, is_hyperbola4, anim_conic ? snap : 1.0);
    }
  }
  DrawUtil::DrawPoint(window, p4, sf::Color::Red, 20.0 * (anim_pt ? bounce : 1.0));
  DrawUtil::DrawPoint(window, p5, sf::Color::Red, 20.0 * (anim_pt ? bounce : 1.0));

  return UpdateTMode(t, mode, 0.02, 0.04);
}

bool Animations::FourTangentSolver(sf::RenderTarget& window) {
  enum Mode {
    MODE_EMPTY,
    MODE_SOL1,
    MODE_SOL2,
    MODE_END,
  };
  static Mode mode = MODE_EMPTY;
  static double t = 0.0;
  RUN_ONCE {
    unfiltered_points[0] = Eigen::Vector2d(0.8775, -0.945)*0.7;
    unfiltered_points[1] = Eigen::Vector2d(-1.14, 0.195)*0.7;
    unfiltered_points[2] = Eigen::Vector2d(0.3075, 1.2225)*0.7;
    unfiltered_points[3] = Eigen::Vector2d(1.5, 0)*0.7;
    unfiltered_points[4] = Eigen::Vector2d(-0.2025, -0.5625);
  }

  //Get points
  const Eigen::Vector2d& p1 = moveable_points[0];
  const Eigen::Vector2d& p2 = moveable_points[1];
  const Eigen::Vector2d& p3 = moveable_points[2];
  const Eigen::Vector2d& p4 = moveable_points[3];
  const Eigen::Vector2d& p5 = moveable_points[4];

  const Eigen::Vector2d pinv = -p5 / p5.squaredNorm();
  const Eigen::Vector2d pperp = Eigen::Vector2d(p5.y(), -p5.x()).normalized();

  const double t12 = IntersectT(pinv, pperp, p1, p2 - p1);
  const double t23 = IntersectT(pinv, pperp, p2, p3 - p2);
  const double t34 = IntersectT(pinv, pperp, p3, p4 - p3);
  const double t41 = IntersectT(pinv, pperp, p4, p1 - p4);

  const double qt = (t12*t34 - t23*t41) / (t12 + t34 - t23 - t41);
  const double r2 = (t12 - qt) * (t34 - qt);
  const double st1 = qt + std::sqrt(std::abs(r2));
  const double st2 = qt - std::sqrt(std::abs(r2));

  //Add points to the solver
  ConicSolver s1;
  s1.AddPoint(p1);
  s1.AddPoint(p2);
  s1.AddPoint(p3);
  s1.AddPoint(p4);
  ConicSolver s2 = s1;
  s1.AddPoint(pinv + st1*pperp);
  s2.AddPoint(pinv + st2*pperp);

  //Solve for the conic
  Ellipse ellipse1, ellipse2;
  double c1, c2, c3, c4, c5, c6;
  s1.SolveDual(c1, c2, c3, c4, c5, c6);
  const bool is_hyperbola1 = ellipse1.FromConic(c1, c2, c3, c4, c5, c6);
  s2.SolveDual(c1, c2, c3, c4, c5, c6);
  const bool is_hyperbola2 = ellipse2.FromConic(c1, c2, c3, c4, c5, c6);

  //Draw the result
  const bool anim_pt = (mode == MODE_EMPTY && !animate_out) || (mode == MODE_SOL2 && animate_out);
  sf::Color lineCol(0, (anim_pt ? uint8_t(255 * t) : 255), 0);
  DrawUtil::DrawDualLine(window, p1, lineCol);
  DrawUtil::DrawDualLine(window, p2, lineCol);
  DrawUtil::DrawDualLine(window, p3, lineCol);
  DrawUtil::DrawDualLine(window, p4, lineCol);
  const double bounce = (animate_out ? 1.0 : DrawUtil::SmoothBounce(t, 0.25, 40.0));
  const double snap = DrawUtil::Snappy(t);
  if (r2 > 0.0) {
    if (mode >= MODE_SOL1) {
      const bool anim_conic = (mode == MODE_SOL1 && !animate_out) || (mode == MODE_SOL2 && animate_out);
      DrawUtil::DrawConic(window, ellipse1, sf::Color::White, is_hyperbola1, anim_conic ? snap : 1.0);
    }
    if (mode >= MODE_SOL2) {
      DrawUtil::DrawConic(window, ellipse2, sf::Color::White, is_hyperbola2, (mode == MODE_SOL2) ? snap : 1.0);
    }
  }
  DrawUtil::DrawPoint(window, p5, sf::Color::Red, 20.0 * (anim_pt ? bounce : 1.0));

  return UpdateTMode(t, mode, 0.02, 0.04);
}

bool Animations::FiveTangentSolver(sf::RenderTarget& window) {
  static double t = 0.0;

  //Get points
  const Eigen::Vector2d& p1 = moveable_points[0];
  const Eigen::Vector2d& p2 = moveable_points[1];
  const Eigen::Vector2d& p3 = moveable_points[2];
  const Eigen::Vector2d& p4 = moveable_points[3];
  const Eigen::Vector2d& p5 = moveable_points[4];

  //Add points to the solver
  ConicSolver s;
  s.AddDualLine(p1, p2);
  s.AddDualLine(p2, p3);
  s.AddDualLine(p3, p4);
  s.AddDualLine(p4, p5);
  s.AddDualLine(p5, p1);

  //Solve for the conic
  double c1, c2, c3, c4, c5, c6;
  s.SolveDual(c1, c2, c3, c4, c5, c6);

  //Create the conic
  Ellipse ellipse;
  const bool is_hyperbola = ellipse.FromConic(c1, c2, c3, c4, c5, c6);

  //Draw the result
  DrawUtil::DrawLine(window, p1, p2, sf::Color(255, 0, 0, uint8_t(255.0 * t)), true, 5.0);
  DrawUtil::DrawLine(window, p2, p3, sf::Color(255, 0, 0, uint8_t(255.0 * t)), true, 5.0);
  DrawUtil::DrawLine(window, p3, p4, sf::Color(255, 0, 0, uint8_t(255.0 * t)), true, 5.0);
  DrawUtil::DrawLine(window, p4, p5, sf::Color(255, 0, 0, uint8_t(255.0 * t)), true, 5.0);
  DrawUtil::DrawLine(window, p5, p1, sf::Color(255, 0, 0, uint8_t(255.0 * t)), true, 5.0);
  DrawUtil::DrawConic(window, ellipse, sf::Color::White, is_hyperbola, t);
  DrawUtil::DrawPoint(window, p1, sf::Color::Red);
  DrawUtil::DrawPoint(window, p2, sf::Color::Red);
  DrawUtil::DrawPoint(window, p3, sf::Color::Red);
  DrawUtil::DrawPoint(window, p4, sf::Color::Red);
  DrawUtil::DrawPoint(window, p5, sf::Color::Red);

  return UpdateT(t, 0.04, 0.02);
}
