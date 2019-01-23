#include <ompl_planners_ros/mc_reeds_shepp_car_planner.hpp>

std::array<double, 3> getState(const ompl::base::State *s) {
  auto state_concrete = s->as<ompl::base::ReedsSheppStateSpace::StateType>();

  std::array<double, 3> return_val{
      state_concrete->getX(), state_concrete->getY(), state_concrete->getYaw()};
  return return_val;
}

namespace ompl_planners_ros {

MultipleCirclesReedsSheppCarPlanner::MultipleCirclesReedsSheppCarPlanner(
    const PlannerParameters &planner_params,
    const VehicleParameters &vehicle_params,
    const nav_msgs::OccupancyGridConstPtr &occ_map_ptr)
    : no_map_(false),
      vehicle_params_(vehicle_params),
      planner_params_(planner_params) {
  grid_map_.setSize(
      occ_map_ptr->info.origin.position.x,
      occ_map_ptr->info.origin.position.x +
          (occ_map_ptr->info.width * occ_map_ptr->info.resolution),
      occ_map_ptr->info.origin.position.y,
      occ_map_ptr->info.origin.position.y +
          (occ_map_ptr->info.height * occ_map_ptr->info.resolution),
      occ_map_ptr->info.resolution);

  for (int h = 0; h < occ_map_ptr->info.height; h++) {
    for (int w = 0; w < occ_map_ptr->info.width; w++) {
      float value = -1.0f;
      const int8_t &occ_map_value =
          occ_map_ptr->data[w + h * occ_map_ptr->info.width];
      if (occ_map_value <= 100 || occ_map_value >= 0) {
        value = 1.0f - (float)occ_map_value / 100.0f;
      } else if (occ_map_value == -1) {
        value = -1.0f;
      } else {
        std::cerr << "[ERROR]: Converting nav_msgs::OccupancyGrid to "
                     "mrpt::maps::COccupancyGridMap2D. Saw an unknown value in "
                     "data: "
                  << occ_map_value << "\n";
      }
      grid_map_.setCell(w, h, value);
    }
  }

  // Setup the og::SimpleSetup;
  ob::StateSpacePtr space(
      new ob::ReedsSheppStateSpace(vehicle_params_.turning_radius));
  ss = boost::make_shared<og::SimpleSetup>(space);

  ob::RealVectorBounds bounds(2);
  if (!no_map_) {
    bounds.low[0] = grid_map_.getXMin();
    bounds.low[1] = grid_map_.getYMin();
    bounds.high[0] = grid_map_.getXMax();
    bounds.high[1] = grid_map_.getYMax();
  } else {
    bounds.low[0] = -10000;
    bounds.low[1] = -10000;
    bounds.high[0] = 10000;
    bounds.high[1] = 10000;
  }

  space->as<ob::SE2StateSpace>()->setBounds(bounds);
  std::cout << "Bounds are [(" << bounds.low[0] << "," << bounds.low[1] << "),("
            << bounds.high[0] << "," << bounds.high[1] << ")]" << std::endl;

  // State Validity Checker
  ob::SpaceInformationPtr si(ss->getSpaceInformation());

  // Get the vertices as a vector of Xs and Ys.
  std::vector<double> x_coords_, y_coords_;
  vehicle_params_.footprint.getAllVertices(x_coords_, y_coords_);

  si->setStateValidityChecker(
      ob::StateValidityCheckerPtr(new MultipleCircleStateValidityChecker(
          si, grid_map_, vehicle_params_.inflation_radius, x_coords_,
          y_coords_)));
  ss->getSpaceInformation()->setStateValidityCheckingResolution(0.005);

  ob::OptimizationObjectivePtr DTCCostObjective(
      new ompl::mod::DTCOptimizationObjective(si));

  ompl::mod::DTCOptimizationObjective *tmp_ptr =
      dynamic_cast<ompl::mod::DTCOptimizationObjective *>(
          DTCCostObjective.get());
  tmp_ptr->initCLiFFMap(planner_params_.cliffmap_filename);
  tmp_ptr->initDTCWeights(planner_params_.weight_d, planner_params_.weight_q,
                          planner_params_.weight_c,
                          vehicle_params_.max_vehicle_speed);
  tmp_ptr->setGetStateAsArrayFunction(&getState);

  ss->setOptimizationObjective(DTCCostObjective);
  // Choose the planner.
  auto planner(boost::make_shared<og::RRTstar>(si));
  ss->setPlanner(planner);

  ss->setup();
  ss->print();

  std::cout << "**********************************************************\n";
  std::cout << "*** MCRSCP has successfully setup the planning problem ***\n";
  std::cout << "**********************************************************\n";
}

bool MultipleCirclesReedsSheppCarPlanner::plan(
    const geometry_msgs::Pose &startState, const geometry_msgs::Pose &goalState,
    geometry_msgs::PoseArray *path) {
  double pLen = 0.0;

  ob::ScopedState<> start(ss->getSpaceInformation()->getStateSpace()),
      goal(ss->getSpaceInformation()->getStateSpace());

  // set the start and goal states
  start[0] = startState.position.x;
  start[1] = startState.position.y;
  start[2] = atan2(startState.orientation.z, startState.orientation.w);

  goal[0] = goalState.position.x;
  goal[1] = goalState.position.y;
  goal[2] = atan2(goalState.orientation.z, goalState.orientation.w);
  ss->setStartAndGoalStates(start, goal);

  printf("\n\x1b[34m*************************************************");
  printf("\n\x1b[34mStart state: (%lf, %lf %lf)", start[0], start[1], start[2]);
  printf("\n\x1b[34mGoal state: (%lf, %lf %lf)", goal[0], goal[1], goal[2]);
  printf("\n\x1b[34m*************************************************");

  ob::PlannerStatus solved =
      ss->getPlanner()->solve(planner_params_.planning_time);

  if (solved) {
    std::cout << "Found solution:" << std::endl;
    ss->simplifySolution();
    og::PathGeometric pth = ss->getSolutionPath();
    pLen = pth.length();
    int numInterpolationPoints =
        ((double)pLen) / planner_params_.path_resolution;
    if (numInterpolationPoints > 0) pth.interpolate(numInterpolationPoints);

    std::vector<ob::State *> states = pth.getStates();
    std::vector<double> reals;

    path->poses.clear();
    path->poses.resize(states.size());

    for (unsigned i = 0; i < states.size(); i++) {
      ss->getStateSpace()->copyToReals(reals, states[i]);
      path->poses[i].position.x = reals[0];
      path->poses[i].position.y = reals[1];
      path->poses[i].orientation.z = sin(reals[2] / 2);
      path->poses[i].orientation.w = cos(reals[2] / 2);
    }
    ss->clearStartStates();
    ss->clear();
    return 1;
  }
  std::cout << "WARNING: No solution found" << std::endl;
  return 0;
}
}  // namespace ompl_planners_ros
