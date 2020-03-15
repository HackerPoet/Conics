#pragma once
#include <Eigen/Dense>

class ConicSolver {
public:
  ConicSolver() { Reset(); }
  void Reset();

  //For point p
  void AddPoint(const Eigen::Vector2d& p);
  //For line connecting points p1 and p2
  void AddDualLine(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);

  //For equation [ax^2 + 2bxy + cy^2 + 2dx + 2ey + f = 0]
  void Solve(double& a, double& b, double& c, double& d, double& e, double& f);
  void SolveDual(double& a, double& b, double& c, double& d, double& e, double& f);

private:
  Eigen::Matrix<double, 5, 5> m_xx;
  Eigen::Matrix<double, 5, 1> m_x;
};
