#include <ompl/mod/objectives/IntensityMapOptimizationObjective.h>

#include <Eigen/Dense>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <cmath>

namespace ompl {
namespace mod {

IntensityMap::IntensityMap(const IntensityMap &intensityMap) {
  this->rows_ = intensityMap.rows_;
  this->columns_ = intensityMap.columns_;
  this->x_max_ = intensityMap.x_max_;
  this->y_max_ = intensityMap.y_max_;
  this->x_min_ = intensityMap.x_min_;
  this->cell_size_ = intensityMap.cell_size_;
  this->values_ = intensityMap.values_;
}

void IntensityMap::readFromXML(const std::string &fileName) {
  using boost::property_tree::ptree;
  ptree pTree;

  boost::property_tree::read_xml(fileName, pTree);

  this->x_min_ = pTree.get<double>("map.parameters.x_min");
  this->y_min_ = pTree.get<double>("map.parameters.y_min");
  this->x_max_ = pTree.get<double>("map.parameters.x_max");
  this->y_max_ = pTree.get<double>("map.parameters.y_max");
  this->cell_size_ = pTree.get<double>("map.parameters.cell_size");
  this->rows_ = ((this->y_max_ - this->y_min_) / this->cell_size_) + 1;
  this->columns_ = ((this->x_max_ - this->x_min_) / this->cell_size_) + 1;

  this->values_.resize(this->rows_ * this->columns_);

  for (const auto &cell : pTree.get_child("map.cells")) {
    if (cell.second.get<size_t>("row") * this->columns_ +
        cell.second.get<size_t>("col") < this->rows_ * this->columns_) {
      this->values_[cell.second.get<size_t>("row") * this->columns_ +
                    cell.second.get<size_t>("col")] =
          cell.second.get<double>("value");
    }
    else {
      printf("OOPS!\n");
    }
  }
}

IntensityMapOptimizationObjective::IntensityMapOptimizationObjective(
    const ompl::base::SpaceInformationPtr &si, const std::string &file_name,
    double wd, double wq, double wc)
    : ompl::mod::MoDOptimizationObjective(si, wd, wq, wc,
                                          MapType::IntensityMap) {
  this->intensity_map_ = IntensityMap(file_name);
  description_ = "Intensity Cost";
  // Setup a default cost-to-go heuristic:
  setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
}

ompl::base::Cost IntensityMapOptimizationObjective::stateCost(
    const ompl::base::State *s) const {
  return ompl::base::Cost(0.0);
}

ompl::base::Cost IntensityMapOptimizationObjective::motionCostHeuristic(
    const ompl::base::State *s1, const ompl::base::State *s2) const {
  return motionCost(s1, s2);
}

ompl::base::Cost IntensityMapOptimizationObjective::motionCost(
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

    double cost_d =
        si_->distance(intermediate_states[i], intermediate_states[i + 1]);
    double cost_q = (1.0 - dot * dot);
    double cost_c = intensity_map_(state_b[0], state_b[1]);

    total_cost +=
        (weight_d_ * cost_d) + (weight_q_ * cost_q) + (weight_c_ * cost_c);
    this->last_cost_.cost_c_ += cost_c;
    this->last_cost_.cost_d_ += cost_d;
    this->last_cost_.cost_q_ += cost_q;
    si_->freeState(intermediate_states[i]);
  }
  si_->freeState(intermediate_states[intermediate_states.size() - 1]);
  return ompl::base::Cost(total_cost);
}
}  // namespace mod
}  // namespace ompl