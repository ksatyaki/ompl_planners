#include "srscp/mc_reeds_shepp_car_planner.hpp"

#include <mrpt/math/CPolygon.h>

int main(int argn, char* args[]) {

  mrpt::math::CPolygon footprint;
  footprint.AddVertex(0.5,0.5);
  footprint.AddVertex(0.5,-0.5);
  footprint.AddVertex(-0.5,0.5);
  footprint.AddVertex(-0.5,-0.5);

  srscp::MultipleCirclesReedsSheppCarPlanner planner(args[1], std::atof(args[2]), 0.25, footprint);

  std::vector <srscp::State> path;
  planner.plan({2.0, 2.0, 0.0}, {2.0, 38.0, 0.0}, 0.4, 2.0, path);

  for(auto p : path) {
    printf("\nX: %lf, Y: %lf, Theta: %lf", p.x, p.y, p.theta);
  }

  return 0;
  

}
