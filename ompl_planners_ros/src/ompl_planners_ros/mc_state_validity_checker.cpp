#include "ompl_planners_ros/mc_state_validity_checker.hpp"

namespace ompl_planners_ros {
bool MultipleCircleStateValidityChecker::isValid(const ob::State *state) const {
  if (!no_map_) {
    const ob::SE2StateSpace::StateType *s =
        state->as<ob::SE2StateSpace::StateType>();
    float x, y, theta, value;
    for (size_t i = 0; i < x_coords_.size(); i++) {
      theta = (float)s->getYaw();
      x = x_coords_[i] * cos(theta) - y_coords_[i] * sin(theta);
      y = x_coords_[i] * sin(theta) + y_coords_[i] * cos(theta);
      x += (float)s->getX();
      y += (float)s->getY();

      value = grid_map_.computeClearance(x, y, radius_);
      if (value < radius_) {
        return false;
      }
    }
  }
  return true;
}
}
