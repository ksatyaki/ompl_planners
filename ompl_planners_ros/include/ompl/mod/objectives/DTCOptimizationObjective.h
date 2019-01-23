#include <array>
#include <functional>

#include "ompl/base/OptimizationObjective.h"

#include <cliffmap_ros/cliffmap.hpp>

namespace ompl {

namespace mod {

class DTCOptimizationObjective : public ompl::base::OptimizationObjective {
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
