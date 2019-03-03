#include <memory>

#include "ompl/mod/objectives/DTCOptimizationObjective.h"

#include <ros/console.h>

ompl::mod::DTCOptimizationObjective::DTCOptimizationObjective(
    const ompl::base::SpaceInformationPtr &si,
    const std::string &cliffmap_filename, float wd, float wq, float wc,
    float maxvs)
    : ompl::base::OptimizationObjective(si),
      weight_d(wd),
      weight_q(wq),
      weight_c(wc),
      max_vehicle_speed(maxvs),
      cliffmap(cliffmap_filename, true) {
  description_ = "DownTheCLiFF Cost";

  // Setup a default cost-to-go heuristics:
  setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
}

ompl::base::Cost ompl::mod::DTCOptimizationObjective::stateCost(
    const ompl::base::State *s) const {
  return this->identityCost();
}

ompl::base::Cost ompl::mod::DTCOptimizationObjective::motionCost(
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
    double this_distance =
        si_->distance(intermediate_states[i], intermediate_states[i + 1]);

    // 4b. Compute the quaternion distance.
    double q_dist = 1.0 - fabs(dot);

    double cliffcost = 0.0;
    Eigen::Vector2d V;
    V[0] = atan2(state_b[1] - state_a[1], state_b[0] - state_a[0]);
    V[1] = max_vehicle_speed;

    double x = state_b[0];
    double y = state_b[1];
    const cliffmap_ros::CLiFFMapLocation &cl = cliffmap(x, y);
    double trust = cl.p * cl.q;

    for (const auto &dist : cl.distributions) {
      Eigen::Matrix2d Sigma;
      std::array<double, 4> sigma_array = dist.getCovariance();
      Sigma(0, 0) = sigma_array[0];
      Sigma(0, 1) = sigma_array[1];
      Sigma(1, 0) = sigma_array[2];
      Sigma(1, 1) = sigma_array[3];
      Eigen::Vector2d myu;
      myu[0] = atan2(sin(dist.getMeanHeading()), cos(dist.getMeanHeading()));
      myu[1] = dist.getMeanSpeed();

      double inc_cost = 0.0;
      if (Sigma.determinant() < 1e-8 && Sigma.determinant() > -1e-8)
        inc_cost = 10000.00;
      else
        inc_cost =
            sqrt((V - myu).transpose() * Sigma.inverse() * (V - myu)) * trust;

      if (inc_cost < 0.0) {
        printf("WHAT THE HOLY?!");
      }

      cliffcost += inc_cost;

      if (std::isnan(cliffcost)) {
        std::cout << "Sigma: " << Sigma << std::endl;
        std::cout << "V: " << V << std::endl;
        std::cout << "myu: " << myu << std::endl;
        std::cout << "Trust: " << trust << std::endl;
        std::cout << "Incremental: " << inc_cost << std::endl;
        printf("cliffcost: %lf\n", cliffcost);
        printf("_________________________\n");
        std::cout << "SHIT!" << std::endl;
      }
    }

    total_cost += (weight_d * this_distance) + (weight_q * q_dist) +
                  (cliffcost * weight_c);
    si_->freeState(intermediate_states[i]);
  }
  si_->freeState(intermediate_states[intermediate_states.size() - 1]);
  return ompl::base::Cost(total_cost);
}

ompl::base::Cost ompl::mod::DTCOptimizationObjective::motionCostHeuristic(
    const ompl::base::State *s1, const ompl::base::State *s2) const {
  return motionCost(s1, s2);
}
