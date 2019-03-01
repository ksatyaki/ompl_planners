#include <Eigen/Dense>
#include <array>
#include <functional>

#include <cliffmap_ros/cliffmap.hpp>

#include "ompl/base/OptimizationObjective.h"

namespace ompl {

namespace mod {

/**
 * The optimization objective class for DownTheCLiFF cost.
 * This is a multi-optimization objective but doens't derive from the
 * corresponding OMPL class.
 */
class DTCOptimizationObjective : public ompl::base::OptimizationObjective {
  /// The weight associated with Euclidean distance cost.
  double weight_d;

  /// The weight associated with quaternion distance cost.
  double weight_q;

  /// The weight associated with Down-The-CLiFF cost.
  double weight_c;

  /// Maximum vehicle speed used in the computation of Down-The-CLiFF cost.
  double max_vehicle_speed;

  /// A function pointer to get the state as a std::array.
  std::function<std::array<double, 3>(const ompl::base::State *)>
      getStateAsArray;

  /// A std smart pointer to the CLiFFMap.
  std::shared_ptr<cliffmap_ros::CLiFFMap> cliffmap;

 public:
  /**
   * Constructor
   * @param si SpaceInformationPtr that we get from the problem setup.
   */
  DTCOptimizationObjective(const ompl::base::SpaceInformationPtr &si);

  /**
   * Initialize CLiFF-map
   * @param cliffmap_filename File name of the CLiFF-map xml.
   */
  inline void initCLiFFMap(const std::string cliffmap_filename) {
    cliffmap = std::make_shared<cliffmap_ros::CLiFFMap>();
    cliffmap->readFromXML(cliffmap_filename);
    cliffmap->organizeAsGrid();

    std::cout
        << ("Read a cliffmap XML organized as a grid @ %lf m/cell resolution.",
            cliffmap->getResolution());
  }

  /**
   * Initialize the DTC weights.
   * @param wd Euclidean distance weight.
   * @param wq Quaternion distance weight.
   * @param wc DTC cost weight.
   * @param maxvs Maximum vehicle speed.
   */
  inline void initDTCWeights(float wd, float wq, float wc, float maxvs) {
    this->weight_c = wc;
    this->weight_d = wd;
    this->weight_q = wq;
    this->max_vehicle_speed = maxvs;
  }

  /**
   * Set the function that returns the state as a std::array. This is required!
   * @param func A function that takes ompl::base::State* and dynamic_casts it
   * to the appropriate state and then returns the state as a std::array.
   */
  void setGetStateAsArrayFunction(
      std::function<std::array<double, 3>(const ompl::base::State *)> func) {
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
