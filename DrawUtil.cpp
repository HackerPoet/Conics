#define NOGDI
#include "DrawUtil.h"
#include <SFML/OpenGL.hpp>

Eigen::Vector2d DrawUtil::center = Eigen::Vector2d::Zero();
double DrawUtil::scale = 100.0;
float DrawUtil::render_scale = 1.0f;

static sf::Vector2f ToSF(const Eigen::Vector2d& v) {
  return sf::Vector2f((float)v.x(), (float)v.y());
}
static Eigen::Vector2d HalfSize(const sf::RenderTarget& window) {
  return Eigen::Vector2d((double)window.getSize().x, (double)window.getSize().y) * 0.5;
}

void DrawUtil::DrawConic(sf::RenderTarget& window, const Ellipse& ellipse, const sf::Color& color, bool is_hyperbola, double t, double thickness) {
  if (is_hyperbola) {
    DrawUtil::DrawHyperbola(window, ellipse, color, t, thickness);
  } else {
    DrawUtil::DrawEllipse(window, ellipse, color, t, thickness);
  }
}

void DrawUtil::DrawEllipse(sf::RenderTarget& window, const Ellipse& ellipse, const sf::Color& color, double t, double thickness) {
  static const int NUM_PTS = 800;
  std::vector<sf::Vertex> vertex_array(NUM_PTS + 1);
  for (size_t i = 0; i < vertex_array.size(); ++i) {
    const double theta = double(2 * i) * Ellipse::pi / double(NUM_PTS);
    const Eigen::Vector2d pt = (ellipse.PointAtAngle(theta) - center)*scale;
    vertex_array[i] = sf::Vertex(ToSF(pt + HalfSize(window)), color);
  }
  glLineWidth((float)thickness * render_scale);
  const size_t num_draw = (size_t)std::min(std::max((t * (double)vertex_array.size()), 0.0), (double)vertex_array.size());
  window.draw(vertex_array.data(), num_draw, sf::PrimitiveType::LineStrip);
}

void DrawUtil::DrawHyperbola(sf::RenderTarget& window, const Ellipse& ellipse, const sf::Color& color, double t, double thickness) {
  static const int NUM_PTS = 800;
  std::vector<sf::Vertex> vertex_array1(NUM_PTS + 1);
  std::vector<sf::Vertex> vertex_array2(NUM_PTS + 1);
  for (size_t i = 0; i < vertex_array1.size(); ++i) {
    const double theta = 20.0 * double(i) / double(NUM_PTS) - 10.0;
    const Eigen::Vector2d pt1 = (ellipse.PointAtAngleH1(theta) - center)*scale;
    vertex_array1[i] = sf::Vertex(ToSF(pt1 + HalfSize(window)), color);
    const Eigen::Vector2d pt2 = (ellipse.PointAtAngleH2(theta) - center)*scale;
    vertex_array2[i] = sf::Vertex(ToSF(pt2 + HalfSize(window)), color);
  }
  glLineWidth((float)thickness * render_scale);
  const size_t num_draw = (size_t)std::min(std::max((t * (double)vertex_array1.size()), 0.0), (double)vertex_array1.size());
  window.draw(vertex_array1.data(), num_draw, sf::PrimitiveType::LineStrip);
  window.draw(vertex_array2.data(), num_draw, sf::PrimitiveType::LineStrip);
}

void DrawUtil::DrawCircle(sf::RenderTarget& window, const Eigen::Vector2d& c, const sf::Color& color, double r, double thickness) {
  sf::CircleShape circle(float(r), 200);
  circle.setOrigin(circle.getRadius(), circle.getRadius());
  circle.setFillColor(sf::Color::Transparent);
  circle.setOutlineColor(color);
  circle.setOutlineThickness((float)thickness);
  circle.setPosition(ToSF((c - center)*scale + HalfSize(window)));
  window.draw(circle);
}

void DrawUtil::DrawLine(sf::RenderTarget& window, const Eigen::Vector2d& _a, const Eigen::Vector2d& _b, const sf::Color& color, bool extend, double thickness) {
  std::vector<sf::Vertex> vertex_array(2);
  Eigen::Vector2d a, b;
  if (extend) {
    a = _a + (_a - _b).normalized()*100.0;
    b = _b + (_b - _a).normalized()*100.0;
  } else {
    a = _a;
    b = _b;
  }
  vertex_array[0] = sf::Vertex(ToSF((a - center)*scale + HalfSize(window)), color);
  vertex_array[1] = sf::Vertex(ToSF((b - center)*scale + HalfSize(window)), color);
  glLineWidth((float)thickness * render_scale);
  window.draw(vertex_array.data(), vertex_array.size(), sf::PrimitiveType::Lines);
}

void DrawUtil::DrawDualLine(sf::RenderTarget& window, const Eigen::Vector2d& p, const sf::Color& color, bool draw_perp, double thickness) {
  const Eigen::Vector2d pinv = -p / p.squaredNorm();
  Eigen::Vector2d pperp = Eigen::Vector2d(p.y(), -p.x()).normalized();
  DrawLine(window, pinv - pperp*100.0, pinv + pperp*100.0, color, false, thickness);
  if (draw_perp) {
    pperp *= (20.0 / scale);
    const Eigen::Vector2d pnorm = p.normalized() * (20.0 / scale);
    DrawLine(window, pinv, p, color, false, thickness * 0.6);
    DrawLine(window, pinv + pnorm + pperp, pinv + pperp, color, false, thickness * 0.6);
    DrawLine(window, pinv + pnorm + pperp, pinv + pnorm, color, false, thickness * 0.6);
  }
}

void DrawUtil::DrawAxes(sf::RenderTarget& window, const Ellipse& ellipse, const sf::Color& color1, const sf::Color& color2, double thickness) {
  const Eigen::Vector2d a = ellipse.v_c + ellipse.v_a;
  const Eigen::Vector2d b = ellipse.v_c + ellipse.v_b;
  const Eigen::Vector2d c = ellipse.v_c;
  DrawLine(window, c, a, color1, false, thickness);
  DrawLine(window, c, b, color2, false, thickness);
}

void DrawUtil::DrawPoint(sf::RenderTarget& window, const Eigen::Vector2d& c, const sf::Color& color, double radius) {
  sf::CircleShape circle((float)radius * render_scale);
  circle.setOrigin(circle.getRadius(), circle.getRadius());
  circle.setFillColor(color);
  circle.setPosition(ToSF((c - center)*scale + HalfSize(window)));
  window.draw(circle);
}

void DrawUtil::DrawDualPoint(sf::RenderTarget& window, const Eigen::Vector2d& a, const Eigen::Vector2d& b, const sf::Color& color, double radius) {
  const Eigen::Vector2d v = (b - a).normalized();
  const double d = a.dot(v);
  DrawPoint(window, (d*v - a) / (a.dot(a) - d*d), color, radius);
}

void DrawUtil::DrawQuad(sf::RenderTarget& window, const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c, const Eigen::Vector2d& d,
                        const sf::Color& color, bool extend, double thickness) {
  DrawLine(window, a, b, color, extend, thickness);
  DrawLine(window, b, c, color, extend, thickness);
  DrawLine(window, c, d, color, extend, thickness);
  DrawLine(window, d, a, color, extend, thickness);
}

void DrawUtil::DrawGrid(sf::RenderTarget& window, double t) {
  const int min_x = int(center.x() - 0.5 * double(window.getSize().x) / scale) - 1;
  const int max_x = int(center.x() + 0.5 * double(window.getSize().x) / scale) + 1;
  const int min_y = int(center.y() - 0.5 * double(window.getSize().y) / scale) - 1;
  const int max_y = int(center.y() + 0.5 * double(window.getSize().y) / scale) + 1;
  window.clear(sf::Color(0, 42, 77));
  for (int x = min_x; x < max_x; ++x) {
    for (int i = 0; i < 4; ++i) {
      const double q = 0.25 * double((x - min_x)*4 + i) / double(max_x - min_x);
      const double bounce = SmoothBounce(t, q*0.5 + 0.5, 20.0);
      if (bounce <= 0.0) { continue; }
      const Eigen::Vector2d a((double)x + i*0.25, (double)min_y);
      const Eigen::Vector2d b((double)x + i*0.25, (double)max_y);
      if (i == 0) {
        DrawLine(window, a, b, sf::Color::White, false, 2.0*bounce);
      } else {
        DrawLine(window, a, b, sf::Color(255, 255, 255, 128), false, bounce);
      }
    }
  }
  for (int y = min_y; y < max_y; ++y) {
    for (int i = 0; i < 4; ++i) {
    const double q = 0.25 * double((y - min_y)*4 + i) / double(max_y - min_y);
    const double bounce = SmoothBounce(t, q*0.5, 20.0);
    if (bounce <= 0.0) { continue; }
      const Eigen::Vector2d a((double)min_x, (double)y + i*0.25);
      const Eigen::Vector2d b((double)max_x, (double)y + i*0.25);
      if (i == 0) {
        DrawLine(window, a, b, sf::Color::White, false, 2.0*bounce);
      } else {
        DrawLine(window, a, b, sf::Color(255, 255, 255, 128), false, bounce);
      }
    }
  }
}

Eigen::Vector2d DrawUtil::FromPix(const sf::RenderTarget& window, const sf::Vector2i& p) {
  return Eigen::Vector2d(double(p.x) - window.getSize().x*0.5, double(p.y) - window.getSize().y*0.5) / scale + center;
}

double DrawUtil::SmoothBounce(double t, double trigger_t, double a) {
  return std::max(1.0 + a*(1.0 - t)*std::exp(10*(trigger_t - t))*(1.0 - std::exp(trigger_t - t)), 0.0);
}

double DrawUtil::Snappy(double t) {
  return t*(t*(t - 1.0) + 1.0);
}
