#include <memory>

#include "ompl/mod/objectives/DTCOptimizationObjective.h"

ompl::mod::DTCOptimizationObjective::DTCOptimizationObjective(
    const ompl::base::SpaceInformationPtr &si)
    : ompl::base::OptimizationObjective(si) {
  description_ = "DownTheCLiFF Cost";

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

  if (!getStateAsArray) {
    std::cerr << "You should set-up a function to get the State as an array "
                 "before you start planning.\n";
  }
  std::array<double, 3> state_prev = getStateAsArray(s1);
  std::array<double, 3> state_curr = getStateAsArray(s2);

  double dot = cos(state_curr[2] / 2.0) * cos(state_prev[2] / 2.0) +
               sin(state_curr[2] / 2.0) * sin(state_prev[2] / 2.0);

  // 2. Compute the quaternion distance.
  double q_dist = pow(1.0 - fabs(dot), 2);

  // Total distance for now.
  // TODO: Weight should be a parameter.
  double distance_cost = weight_d * sqrt(this_distance_sq) + weight_q * sqrt(q_dist);

  double cliffcost = 0.0;
  Eigen::Vector2d V;
  V[0] = state_curr[2];
  // TODO: make the intended speed a parameters.
  V[1] = max_vehicle_speed;

  double x = state_curr[0];
  double y = state_curr[1];
  const cliffmap_ros::CLiFFMapLocation& cl = (*cliffmap)(x,y);
  double trust = cl.p * cl.q;

  for (const auto &dist : cl.distributions) {
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

  // TODO: Make the weight a parameter.
  return ompl::base::Cost(distance_cost + (cliffcost * weight_c));
}

ompl::base::Cost ompl::mod::DTCOptimizationObjective::motionCostHeuristic(
    const ompl::base::State *s1, const ompl::base::State *s2) const {
  return motionCost(s1, s2);
}
