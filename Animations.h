#pragma once
#include <SFML/Graphics.hpp>
#include <Eigen/Dense>

class Animations {
public:
  static Eigen::Vector2d unfiltered_points[5];
  static Eigen::Vector2d moveable_points[5];
  static int frame;
  static bool animate_out;

  static bool Background(sf::RenderTarget& window, bool staret_anim);

  static bool RandomConics(sf::RenderTarget& window);

  static bool FivePointSolver(sf::RenderTarget& window);
  static bool FourPointSolver(sf::RenderTarget& window);
  static bool ThreePointSolver(sf::RenderTarget& window);
  static bool ThreeTangentSolver(sf::RenderTarget& window);
  static bool FourTangentSolver(sf::RenderTarget& window);
  static bool FiveTangentSolver(sf::RenderTarget& window);

  static bool DualLines(sf::RenderTarget& window);

  static bool EllipseConstruction(sf::RenderTarget& window);
  static bool SkewAxes(sf::RenderTarget& window);

  static bool HardestProblem1(sf::RenderTarget& window);
  static bool HardestProblem2(sf::RenderTarget& window);
  static bool HardestProblem3(sf::RenderTarget& window);
};
