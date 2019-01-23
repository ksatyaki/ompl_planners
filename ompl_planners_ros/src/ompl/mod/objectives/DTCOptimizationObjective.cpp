#include <memory>
#include <Eigen/Core>

#include "ompl/mod/objectives/DTCOptimizationObjective.h"

ompl::mod::DTCOptimizationObjective::DTCOptimizationObjective(
    const ompl::base::SpaceInformationPtr &si)
    : ompl::base::OptimizationObjective(si) {
  description_ = "Path Length";

  // Setup a default cost-to-go heuristics:
  setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
}

ompl::base::Cost ompl::mod::DTCOptimizationObjective::stateCost(
    const ompl::base::State *s) const {
  return identityCost();
}

ompl::base::Cost ompl::mod::DTCOptimizationObjective::motionCost(
    const ompl::base::State *s1, const ompl::base::State *s2) const {
  // 1. Compute distance between the previous and current state.
  double this_distance_sq = si_->distance(s1, s2);

  // 2. Compute the quaternion distance.
  double q_dist = pow(1.0 - fabs(quaternionDistance(s1, s2)), 2);

  // Total distance for now.
  // TODO: Weight should be a parameter.
  double distance_cost = sqrt(this_distance_sq) + 10.0 * sqrt(q_dist);

  double cliffcost = 0.0;
  Eigen::Vector2d V;
  V[0] = state_curr->state_vars[2];
  // TODO: make the intended speed a parameters.
  V[1] = 1.0;  // sqrt(this_distance_sq) / this_time;

  double x = state_curr->state_vars[0];
  double y = state_curr->state_vars[1];
  double trust = (*cliffmap)(x, y).p * (*cliffmap)(x, y).q;
  for (const auto &dist : (*cliffmap)(x, y).distributions) {
    Eigen::Matrix2d Sigma;
    std::array<double, 4> sigma_array = dist.getCovariance();
    Sigma(0, 0) = sigma_array[0];
    Sigma(0, 1) = sigma_array[1];
    Sigma(1, 0) = sigma_array[2];
    Sigma(1, 1) = sigma_array[3];
    Eigen::Vector2d myu;
    myu[0] = dist.getMeanHeading();
    myu[1] = dist.getMeanSpeed();

    double inc_cost = 0.0;
    if (Sigma.determinant() < 1e-4 && Sigma.determinant() > -1e-4)
      inc_cost = 10000.00;
    else
      inc_cost =
          sqrt((V - myu).transpose() * Sigma.inverse() * (V - myu)) * trust;

    if (inc_cost < 0.0) {
      printf("WHAT THE HOLY?!");
    }
    // if (inc_cost > 10000.00)
    //  inc_cost = 10000.00f;

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
  // 3. Add the cliffcost
  if (only_distance_cost) {
    total_cost = total_cost + distance_cost;
  } else {
    total_cost = total_cost + distance_cost + (cliffcost / 5.0);
  }

  return total_cost;
}

ompl::base::Cost ompl::mod::DTCOptimizationObjective::motionCostHeuristic(
    const ompl::base::State *s1, const ompl::base::State *s2) const {
  return motionCost(s1, s2);
}
