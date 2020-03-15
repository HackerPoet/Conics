#pragma once
#include <SFML/Graphics.hpp>
#include "Ellipse.h"

class DrawUtil {
public:
  static void DrawConic(sf::RenderTarget& window, const Ellipse& ellipse, const sf::Color& color, bool is_hyperbola, double t, double thickness = 8.0f);
  static void DrawEllipse(sf::RenderTarget& window, const Ellipse& ellipse, const sf::Color& color, double t, double thickness = 8.0f);
  static void DrawHyperbola(sf::RenderTarget& window, const Ellipse& ellipse, const sf::Color& color, double t, double thickness = 8.0f);
  static void DrawCircle(sf::RenderTarget& window, const Eigen::Vector2d& c, const sf::Color& color, double r, double thickness = 8.0f);
  static void DrawLine(sf::RenderTarget& window, const Eigen::Vector2d& a, const Eigen::Vector2d& b, const sf::Color& color, bool extend, double thickness = 8.0f);
  static void DrawDualLine(sf::RenderTarget& window, const Eigen::Vector2d& p, const sf::Color& color, bool draw_perp = false, double thickness = 8.0f);
  static void DrawPoint(sf::RenderTarget& window, const Eigen::Vector2d& c, const sf::Color& color, double radius = 20.0f);
  static void DrawDualPoint(sf::RenderTarget& window, const Eigen::Vector2d& a, const Eigen::Vector2d& b, const sf::Color& color, double radius = 20.0f);
  static void DrawAxes(sf::RenderTarget& window, const Ellipse& ellipse, const sf::Color& color1, const sf::Color& color2, double thickness = 8.0f);
  static void DrawQuad(sf::RenderTarget& window, const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c, const Eigen::Vector2d& d,
                       const sf::Color& color, bool extend=false, double thickness = 8.0f);
  static void DrawGrid(sf::RenderTarget& window, double t);

  static Eigen::Vector2d FromPix(const sf::RenderTarget& window, const sf::Vector2i& p);
  static double SmoothBounce(double t, double trigger_t, double a);
  static double Snappy(double t);

  static Eigen::Vector2d center;
  static double scale;
  static float render_scale;
};
