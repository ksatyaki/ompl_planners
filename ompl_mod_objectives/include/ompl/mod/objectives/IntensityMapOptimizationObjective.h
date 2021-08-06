/*
 *   Copyright (c) 2019 Chittaranjan Srinivas Swaminathan
 *   This file is part of ompl_mod_objectives.
 *
 *   ompl_mod_objectives is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ompl_mod_objectives is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with ompl_planners_ros.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <ompl/mod/objectives/MoDOptimizationObjective.h>

#include <Eigen/Dense>
#include <array>
#include <functional>

namespace ompl {
namespace mod {

class IntensityMap {
  double x_max_;
  double y_max_;
  double x_min_;
  double y_min_;
  size_t rows_;
  size_t columns_;
  double cell_size_;
  std::vector<double> values_;

  void readFromXML(const std::string &fileName);

 public:
  inline IntensityMap(const std::string &fileName) {
    this->readFromXML(fileName);
  }

  inline double getXMax() const { return x_max_; }
  inline double getYMax() const { return y_max_; }
  inline double getXMin() const { return x_min_; }
  inline double getYMin() const { return y_min_; }
  inline size_t getRows() { return rows_; }
  inline size_t getColumns() { return columns_; }
  inline double getCellSize() { return cell_size_; }

  IntensityMap(const IntensityMap &intensityMap);
  virtual ~IntensityMap() = default;

  inline double operator()(double x, double y) const {
    size_t row = size_t(std::floor(y - this->y_min_) / this->cell_size_);
    size_t col = size_t(std::floor(x - this->x_min_) / this->cell_size_);
    std::cout << "Row, Col: " << row << ", " << col << std::endl;
    return this->values_[row * this->columns_ + col];
  }
};

class IntensityMapOptimizationObjective : public MoDOptimizationObjective {
 protected:
  IntensityMap intensity_map_;

 public:
  IntensityMapOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                                    const std::string &file_name, double wd,
                                    double wq, double wc);

  virtual ~IntensityMapOptimizationObjective() = default;

  virtual inline bool isSymmetric() const override { return false; }

  ompl::base::Cost stateCost(const ompl::base::State *s) const override;

  ompl::base::Cost motionCost(const ompl::base::State *s1,
                              const ompl::base::State *s2) const override;

  ompl::base::Cost motionCostHeuristic(
      const ompl::base::State *s1, const ompl::base::State *s2) const override;
};

typedef std::shared_ptr<IntensityMapOptimizationObjective>
    IntensityMapOptimizationObjectivePtr;

}  // namespace mod
}  // namespace ompl