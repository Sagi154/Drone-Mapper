#include "MapRenderer.h"

#include <SFML/Config.hpp>
#include <SFML/Graphics.hpp>

namespace dmap::viz {

void MapRenderer::openWindow() {
#if SFML_VERSION_MAJOR >= 3
  sf::RenderWindow window(sf::VideoMode(sf::Vector2u(800, 600)), "DroneMapper visualizer");
#else
  sf::RenderWindow window(sf::VideoMode(800, 600), "DroneMapper visualizer");
#endif
  window.close();
}

}  // namespace dmap::viz
