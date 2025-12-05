#include <pluginlib/class_loader.hpp>
#include <polygon_base/regular_polygon.hpp>

#include <iostream>
#include <iomanip>

int main(int argc, char **argv) {
  (void) argc;
  (void) argv;

  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("polygon_base", "polygon_base::RegularPolygon");

  try {
    auto triangle = poly_loader.createSharedInstance("awesome_triangle");
    triangle->initialize(10.0);

    auto square = poly_loader.createSharedInstance("polygon_plugins::Square");
    square->initialize(10.0);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Triangle area: " <<  triangle->area() << '\n';
    std::cout << "Square area: " <<  square->area() << '\n';
  } catch(pluginlib::PluginlibException &ex) {
    std::cerr << "The plugin failed to load for some reason. Error: " << ex.what() << '\n';
  }

  return 0;
}
