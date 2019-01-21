#include "ompl_planners_ros/simple_state_validity_checker.hpp"

bool SimpleStateValidityChecker::isValid(const ob::State *state) const {
  const ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
  float x=(float)s->getX(), y=(float)s->getY();
  float value = gridmap.computeClearance(x, y, radius);
  return (value >= radius);
}
