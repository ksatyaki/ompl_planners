#include <srscp/mc_reeds_shepp_car_planner.hpp>

namespace srscp {

MultipleCirclesReedsSheppCarPlanner::MultipleCirclesReedsSheppCarPlanner(
    const char *mapFileName, double mapResolution, double robotRadius,
    const mrpt::math::CPolygon &footprint) {

  if (mapFileName != NULL) {
    grid_map_.loadFromBitmapFile(mapFileName, (float)mapResolution, 0.0f, 0.0f);
    std::cout << "Loaded map (1) " << mapFileName << std::endl;
  } else {
    std::cout << "Using empty map" << std::endl;
    no_map_ = true;
  }

  // Initialize footprint points
  footprint.getAllVertices(x_coords_, y_coords_);

  //
  robot_radius_ = robotRadius;
}

MultipleCirclesReedsSheppCarPlanner::MultipleCirclesReedsSheppCarPlanner(
    const nav_msgs::OccupancyGridConstPtr &occ_map_ptr, double robotRadius,
    const mrpt::math::CPolygon &footprint) {

  grid_map_.setSize(
      occ_map_ptr->info.origin.position.x,
      occ_map_ptr->info.origin.position.x +
          (occ_map_ptr->info.width * occ_map_ptr->info.resolution),
      occ_map_ptr->info.origin.position.y,
      occ_map_ptr->info.origin.position.y +
          (occ_map_ptr->info.height * occ_map_ptr->info.resolution),
      occ_map_ptr->info.resolution);

  // std::cout << "X: " << grid_map_.getXMin() << std::endl
  //           << "Y: " << grid_map_.getYMin() << std::endl
  //           << "Resolution: " << grid_map_.getResolution() << std::endl
  //           << "XMax: " << grid_map_.getXMax() << std::endl
  //           << "YMax: " << grid_map_.getYMax() << std::endl;

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

  //grid_map_.saveAsBitmapFile("ok.png");

  // Initialize footprint points
  footprint.getAllVertices(x_coords_, y_coords_);

  //
  robot_radius_ = robotRadius;
}

bool MultipleCirclesReedsSheppCarPlanner::plan(const State &startState,
                                               const State &goalState,
                                               double distanceBetweenPathPoints,
                                               double turningRadius,
                                               std::vector<State> &path) {

  double pLen = 0.0;
  int numInterpolationPoints = 0;
  ob::StateSpacePtr space(new ob::ReedsSheppStateSpace(turningRadius));

  ob::ScopedState<> start(space), goal(space);
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

  // define a simple setup class
  og::SimpleSetup ss(space);

  // set state validity checking for this space
  ob::SpaceInformationPtr si(ss.getSpaceInformation());
  // if (!grid_map_.isEmpty()) {
  si->setStateValidityChecker(
      ob::StateValidityCheckerPtr(new MultipleCircleStateValidityChecker(
          si, grid_map_, robot_radius_, x_coords_, y_coords_)));

  //} else {
  // si->setStateValidityChecker(ob::StateValidityCheckerPtr(
  //     new MultipleCircleStateValidityChecker(si)));
  // std::cout << "No validity checker!" << std::endl;
  //}

  ob::PlannerPtr planner(new og::RRTConnect(si));
  // ob::PlannerPtr planner(new og::RRTstar(si));
  // ob::PlannerPtr planner(new og::TRRT(si));
  // ob::PlannerPtr planner(new og::SST(si));
  // ob::PlannerPtr planner(new og::LBTRRT(si));
  // ob::PlannerPtr planner(new og::PRMstar(si));
  // ob::PlannerPtr planner(new og::SPARS(si));
  // ob::PlannerPtr planner(new og::pRRT(si));
  // ob::PlannerPtr planner(new og::LazyRRT(si));
  ss.setPlanner(planner);

  // set the start and goal states
  start[0] = startState.x;
  start[1] = startState.y;
  start[2] = startState.theta;
  goal[0] = goalState.x;
  goal[1] = goalState.y;
  goal[2] = goalState.theta;
  ss.setStartAndGoalStates(start, goal);

  // this call is optional, but we put it in to get more output information
  ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
  ss.setup();
  ss.print();

  // attempt to solve the problem within 30 seconds of planning time
  ob::PlannerStatus solved = ss.solve(30.0);

  if (solved) {
    std::cout << "Found solution:" << std::endl;
    ss.simplifySolution();
    og::PathGeometric pth = ss.getSolutionPath();
    pLen = pth.length();
    numInterpolationPoints = ((double)pLen) / distanceBetweenPathPoints;
    if (numInterpolationPoints > 0)
      pth.interpolate(numInterpolationPoints);

    std::vector<ob::State *> states = pth.getStates();
    std::vector<double> reals;

    path.resize(states.size());

    for (unsigned i = 0; i < states.size(); i++) {
      space->copyToReals(reals, states[i]);
      path[i].x = reals[0];
      path[i].y = reals[1];
      path[i].theta = reals[2];
    }
    return 1;
  }
  std::cout << "No solution found" << std::endl;
  return 0;
}
}
