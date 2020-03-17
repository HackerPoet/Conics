#define NOGDI
#include "Animations.h"
#include "DrawUtil.h"
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <iostream>
#include <random>
#include <sstream>
#include <cassert>
#include <fstream>

#ifdef RECORDING_QUALITY
static const int start_w = 1920;
static const int start_h = 1080;
static const bool start_fullscreen = true;
static const int render_scale = 2;
#else
static const int start_w = 1280;
static const int start_h = 720;
static const bool start_fullscreen = false;
static const int render_scale = 1;
#endif

static int cur_anim = -1;
static sf::Vector2i mouse_pos;
static bool mouse_pressed = false;
static int mouse_select = -1;
static const double pt_smoothing = 0.8;

static bool(*ANIM_ARRAY[])(sf::RenderTarget& window) = {
  Animations::RandomConics,
  Animations::FivePointSolver,
  Animations::FiveTangentSolver,
  Animations::FourPointSolver,
  Animations::FourTangentSolver,
  Animations::ThreePointSolver,
  Animations::ThreeTangentSolver,
  Animations::DualLines,
  Animations::EllipseConstruction,
  Animations::SkewAxes,
  Animations::HardestProblem1,
  Animations::HardestProblem2,
  Animations::HardestProblem3,
};
static const int NUM_ANIMS = sizeof(ANIM_ARRAY) / sizeof(ANIM_ARRAY[0]);

static void ActivatePoint(const sf::RenderTarget& window) {
  const Eigen::Vector2d p = DrawUtil::FromPix(window, mouse_pos * render_scale);
  for (int i = 0; i < 5; ++i) {
    const double d = (Animations::moveable_points[i] - p).norm();
    if (d * DrawUtil::scale < 10.0 * render_scale) {
      mouse_select = i;
      return;
    }
  }
  mouse_select = -1;
}

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
// windows main
INT WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PSTR lpCmdLine, INT nCmdShow) {
#else
int main(int argc, char *argv[]) {
#endif
  //Get the screen size
  sf::VideoMode screenSize = sf::VideoMode::getDesktopMode();
  screenSize = sf::VideoMode(start_w, start_h, screenSize.bitsPerPixel);
  DrawUtil::render_scale = float(render_scale) * 0.5f;
  
  //GL settings
  sf::ContextSettings settings;
  settings.depthBits = 24;
  settings.stencilBits = 8;
  settings.antialiasingLevel = 16;
  settings.majorVersion = 2;
  settings.minorVersion = 0;

  //Create the window
  sf::RenderWindow window;
  sf::Uint32 window_style = (start_fullscreen ? sf::Style::Fullscreen : sf::Style::Resize | sf::Style::Close);
  window.create(screenSize, "Extraordinary Conics - By CodeParade", window_style, settings);
  window.setFramerateLimit(60);
  window.requestFocus();
  sf::View view = window.getDefaultView();

  //Create the render texture 4 times larger than the window
  sf::RenderTexture renderTexture;
  renderTexture.create(window.getSize().x * render_scale, window.getSize().y * render_scale, settings);
  renderTexture.setSmooth(true);
  renderTexture.setActive(true);

  //Setup OpenGL things
  glHint(GL_POINT_SMOOTH, GL_NICEST);
  glHint(GL_LINE_SMOOTH, GL_NICEST);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_SMOOTH);

  //Create geometry
  DrawUtil::center = Eigen::Vector2d::Zero();
  DrawUtil::scale = 130.0 * render_scale;
  Animations::unfiltered_points[0] = Eigen::Vector2d(0.10f, -1.00f);
  Animations::unfiltered_points[1] = Eigen::Vector2d(-1.20f, 1.40f);
  Animations::unfiltered_points[2] = Eigen::Vector2d(2.50f, 1.00f);
  Animations::unfiltered_points[3] = Eigen::Vector2d(2.00f, 0.10f);
  Animations::unfiltered_points[4] = Eigen::Vector2d(-0.05f, 2.05f);
  for (int i = 0; i < 5; ++i) {
    Animations::moveable_points[i] = Animations::unfiltered_points[i];
  }

  //Main Loop
  bool capture = false;
  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
        break;
      } else if (event.type == sf::Event::KeyPressed) {
        const sf::Keyboard::Key keycode = event.key.code;
        if (keycode == sf::Keyboard::Escape) {
          window.close();
          break;
        } else if (keycode == sf::Keyboard::Space) {
          if (cur_anim > 0) {
            Animations::animate_out = true;
          } else {
            cur_anim += 1;
          }
        } else if (keycode == sf::Keyboard::C) {
          capture = true;
        }
      } else if (event.type == sf::Event::Resized) {
        const sf::FloatRect visibleArea(0, 0, (float)event.size.width, (float)event.size.height);
        window.setView(sf::View(visibleArea));
        renderTexture.create(window.getSize().x * render_scale, window.getSize().y * render_scale, settings);
        renderTexture.setSmooth(true);
        renderTexture.setActive(true);
      } else if (event.type == sf::Event::MouseMoved) {
        mouse_pos = sf::Vector2i(event.mouseMove.x, event.mouseMove.y);
      } else if (event.type == sf::Event::MouseButtonPressed) {
        mouse_pos = sf::Vector2i(event.mouseButton.x, event.mouseButton.y);
        mouse_pressed = true;
        ActivatePoint(renderTexture);
      } else if (event.type == sf::Event::MouseButtonReleased) {
        mouse_pos = sf::Vector2i(event.mouseButton.x, event.mouseButton.y);
        mouse_pressed = false;
        mouse_select = -1;
      }
    }

    //Move active point
    if (mouse_select >= 0) {
      Animations::unfiltered_points[mouse_select] = DrawUtil::FromPix(renderTexture, mouse_pos * render_scale);
    }

    //Filter points
    for (int i = 0; i < 5; ++i) {
      Animations::moveable_points[i] *= pt_smoothing;
      Animations::moveable_points[i] += Animations::unfiltered_points[i] * (1.0 - pt_smoothing);
    }

    //Draw the background
    renderTexture.setActive(true);
    Animations::Background(renderTexture, cur_anim >= 0);

    //Draw the foreground
    if (cur_anim > 0 && cur_anim <= NUM_ANIMS && ANIM_ARRAY[cur_anim - 1](renderTexture)) {
      Animations::animate_out = false;
      cur_anim += 1;
    }

    //Finish drawing to the texture
    renderTexture.display();

    //Draw texture to window
    window.setActive(true);
    const sf::Texture& texture = renderTexture.getTexture();
    sf::Sprite sprite(texture);
    sprite.setScale(1.0f / float(render_scale), 1.0f / float(render_scale));
    window.draw(sprite);

    //Take a single screen-shot
    if (capture) {
      texture.copyToImage().saveToFile("sshot.png");
      capture = false;
    }

    //Flip the screen buffer
    window.display();
    Animations::frame += 1;
  }

  return 0;
}
