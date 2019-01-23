#include <array>
#include <functional>
#include <Eigen/Dense>

#include <cliffmap_ros/cliffmap.hpp>

#include "ompl/base/OptimizationObjective.h"


namespace ompl {

namespace mod {

class DTCOptimizationObjective : public ompl::base::OptimizationObjective {

  double weight_d;
  double weight_c;
  double weight_q;
  double max_vehicle_speed;

  std::function<std::array<double,3>(const ompl::base::State *)> getStateAsArray;

  std::shared_ptr<cliffmap_ros::CLiFFMap> cliffmap;

 public:
  DTCOptimizationObjective(const ompl::base::SpaceInformationPtr &si);

  inline void initCLiFFMap(const std::string cliffmap_filename) {
    cliffmap = std::make_shared<cliffmap_ros::CLiFFMap>();
    cliffmap->readFromXML(cliffmap_filename);
    cliffmap->organizeAsGrid();

    std::cout
        << ("Read a cliffmap XML organized as a grid @ %lf m/cell resolution.",
            cliffmap->getResolution());
  }

  inline void initDTCWeights(float wd, float wq, float wc, float maxvs) {
    this->weight_c = wc;
    this->weight_d = wd;
    this->weight_q = wq;
    this->max_vehicle_speed = maxvs;
  }
  void setGetStateAsArrayFunction(
      std::function<std::array<double,3>(const ompl::base::State *)> func) {
    getStateAsArray = func;
  }

  ompl::base::Cost stateCost(const ompl::base::State *s) const override;

  ompl::base::Cost motionCost(const ompl::base::State *s1,
                              const ompl::base::State *s2) const override;

  ompl::base::Cost motionCostHeuristic(
      const ompl::base::State *s1, const ompl::base::State *s2) const override;
};
}
}
