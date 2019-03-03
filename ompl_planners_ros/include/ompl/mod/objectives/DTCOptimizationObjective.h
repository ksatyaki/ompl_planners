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

  /// A std smart pointer to the CLiFFMap.
  cliffmap_ros::CLiFFMap cliffmap;

 public:
  /**
   * Constructor
   * @param si SpaceInformationPtr that we get from the problem setup.
   */
  DTCOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                           const std::string &cliffmap_filename, float wd,
                           float wq, float wc, float maxvs);

  virtual bool isSymmetric() const override { return false; }

  ompl::base::Cost stateCost(const ompl::base::State *s) const override;

  ompl::base::Cost motionCost(const ompl::base::State *s1,
                              const ompl::base::State *s2) const override;

  ompl::base::Cost motionCostHeuristic(
      const ompl::base::State *s1, const ompl::base::State *s2) const override;
};
}
}
