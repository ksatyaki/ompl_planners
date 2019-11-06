//
// Created by ksatyaki on 2019-10-30.
//

#include "ompl/mod/objectives/DTWOptimizationObjective.h"
namespace ompl {
namespace mod {

DTWOptimizationObjective::DTWOptimizationObjective(
    const ompl::base::SpaceInformationPtr &si,
    const whytemap_ros::WHyTeMap &whytemap, double wd, double wq, double wc,
    double maxvs)
    : ompl::mod::MoDOptimizationObjective(si, wd, wq, wc, MapType::WHyTeMap),
      max_vehicle_speed(maxvs), whytemap(whytemap) {
  description_ = "DownTheWHyTe Cost";

  // Setup a default cost-to-go heuristic:
  setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
}

ompl::base::Cost DTWOptimizationObjective::stateCost(
    const ompl::base::State *s) const {
  return ompl::base::Cost(0.0);
}

ompl::base::Cost DTWOptimizationObjective::motionCostHeuristic(
    const ompl::base::State *s1, const ompl::base::State *s2) const {
  return motionCost(s1, s2);
}

ompl::base::Cost ompl::mod::DTWOptimizationObjective::motionCost(
    const ompl::base::State *s1, const ompl::base::State *s2) const {
  auto space = si_->getStateSpace();
  // 1. Declare the intermediate states.
  std::vector<ompl::base::State *> intermediate_states;

  // 2. How many segments do we want. Each segment should be approximately the
  // size of resolution.
  unsigned int numSegments = space->validSegmentCount(s1, s2);

  // 3. Get intermediate states.
  si_->getMotionStates(s1, s2, intermediate_states, numSegments - 1, true,
                       true);

  double total_cost = 0.0;
  this->last_cost_.cost_d_ = 0.0;
  this->last_cost_.cost_q_ = 0.0;
  this->last_cost_.cost_c_ = 0.0;

  for (unsigned int i = 0; i < intermediate_states.size() - 1; i++) {
    std::array<double, 3> state_a{
        *space->getValueAddressAtIndex(intermediate_states[i], 0),
        *space->getValueAddressAtIndex(intermediate_states[i], 1),
        *space->getValueAddressAtIndex(intermediate_states[i], 2)};
    std::array<double, 3> state_b{
        *space->getValueAddressAtIndex(intermediate_states[i + 1], 0),
        *space->getValueAddressAtIndex(intermediate_states[i + 1], 1),
        *space->getValueAddressAtIndex(intermediate_states[i + 1], 2)};

    double dot = cos(state_b[2] / 2.0) * cos(state_a[2] / 2.0) +
                 sin(state_b[2] / 2.0) * sin(state_a[2] / 2.0);

    // 4a. Compute Euclidean distance.
    double cost_d =
        si_->distance(intermediate_states[i], intermediate_states[i + 1]);

    // 4b. Compute the quaternion distance.
    double cost_q = (1.0 - dot*dot);

    double alpha = atan2(state_b[1] - state_a[1], state_b[0] - state_a[0]);

    double x = state_b[0];
    double y = state_b[1];

    double cost_c = whytemap.getLikelihood(this->timestamp, x, y, alpha, max_vehicle_speed);


    total_cost += (weight_d_ * cost_d) + (weight_q_ * cost_q) +
                  (weight_c_ * cost_c);
    this->last_cost_.cost_c_ += cost_c;
    this->last_cost_.cost_d_ += cost_d;
    this->last_cost_.cost_q_ += cost_q;
    si_->freeState(intermediate_states[i]);
  }
  si_->freeState(intermediate_states[intermediate_states.size() - 1]);
  return ompl::base::Cost(total_cost);
}


} // namespace mod
} // namespace ompl
