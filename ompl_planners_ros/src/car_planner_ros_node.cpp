/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   This file is part of ompl_planners_ros.
 *
 *   ompl_planners_ros is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ompl_planners_ros is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with ompl_planners_ros.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <fstream>

#include <ros/console.h>
#include <ros/ros.h>

#include <costmap_2d/footprint.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <mrpt/math/CPolygon.h>

#include "ompl_planners_ros/mc_reeds_shepp_car_planner.hpp"
#include "ompl_planners_ros/visualization.hpp"

#include "ompl/mod/objectives/DTCOptimizationObjective.h"
#include "ompl/mod/objectives/UpstreamCriterionOptimizationObjective.h"

#include <cliffmap_ros/cliffmap.hpp>
#include <gmmtmap_ros/gmmtmap.hpp>
#include <stefmap_ros/stefmap.hpp>

class CarPlannerROSNode {
protected:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ompl_planners_ros::PlannerParameters pp;
  ompl_planners_ros::VehicleParameters vp;

  std::shared_ptr<stefmap_ros::STeFMapClient> stefmap_client;
  std::shared_ptr<cliffmap_ros::CLiFFMapClient> cliffmap_client;
  std::shared_ptr<gmmtmap_ros::GMMTMapClient> gmmtmap_client;
  ros::ServiceClient map_client;

  ros::Subscriber save_path_sub;

  geometry_msgs::Pose2DPtr start_pose;
  geometry_msgs::Pose2DPtr goal_pose;
  boost::shared_ptr<std::vector<geometry_msgs::Pose2D>> poses_ptr;
  boost::shared_ptr<ompl_planners_ros::MultipleCirclesReedsSheppCarPlanner>
      planner;

public:
  CarPlannerROSNode() : nh("~") {
    map_client = private_nh.serviceClient<nav_msgs::GetMap>("static_map");
    map_client.waitForExistence();

    nav_msgs::GetMap get_map_;
    if (!map_client.call(get_map_)) {
      ROS_FATAL("Failed to call the map server for map!");
    }

    ROS_INFO_STREAM("[PLANNER]: Map received!");

    std::string mod_type = "CLiFF-map";

    nh.getParam("planner/weight_d", pp.weight_d);
    nh.getParam("planner/weight_c", pp.weight_c);
    nh.getParam("planner/weight_q", pp.weight_q);
    nh.getParam("planner/planning_time", pp.planning_time);
    nh.getParam("planner/path_resolution", pp.path_resolution);
    nh.getParam("planner/mod_type", mod_type);
    nh.getParam("planner/publish_viz_markers", pp.publish_viz_markers);
    nh.getParam("vehile/inflation_radius", vp.inflation_radius);
    nh.getParam("vehicle/turning_radius", vp.turning_radius);

    std::vector<geometry_msgs::Point> ftprnt =
        costmap_2d::makeFootprintFromParams(nh);

    for (const auto &ftprntpt : ftprnt) {
      ROS_INFO_STREAM(std::fixed << std::setprecision(2) << "\x1b[34m("
                                 << ftprntpt.x << "," << ftprntpt.y << ")");
      vp.footprint.AddVertex(ftprntpt.x, ftprntpt.y);
    }

    int shitwidth = 31;

    ROS_INFO_STREAM("*** *********************************** ***");
    ROS_INFO_STREAM("*** wd: " << std::setw(shitwidth) << pp.weight_d
                               << " ***");
    ROS_INFO_STREAM("*** wq: " << std::setw(shitwidth) << pp.weight_q
                               << " ***");
    ROS_INFO_STREAM("*** wc: " << std::setw(shitwidth) << pp.weight_c
                               << " ***");
    ROS_INFO_STREAM("*** Publish viz markers? " << std::setw(shitwidth - 17)
                                                << pp.publish_viz_markers
                                                << " ***");
    ROS_INFO_STREAM("*** Planning time: " << std::setw(shitwidth - 11)
                                          << pp.planning_time << " ***");
    ROS_INFO_STREAM("*** Path resolution: " << std::setw(shitwidth - 13)
                                            << pp.path_resolution << " ***");
    ROS_INFO_STREAM("*** MoD type: " << std::setw(shitwidth - 6)
                                     << mod_type.c_str() << " ***");
    ROS_INFO_STREAM("*** Turning radius: " << std::setw(shitwidth - 12)
                                           << vp.turning_radius << " ***");
    ROS_INFO_STREAM("*** Inflation radius: " << std::setw(shitwidth - 14)
                                             << vp.inflation_radius << " ***");
    ROS_INFO_STREAM("*** Check footprint manually." << std::setw(shitwidth - 17)
                                                    << " ***");
    ROS_INFO_STREAM("*** *********************************** ***");

    nav_msgs::OccupancyGridPtr occ_map =
        nav_msgs::OccupancyGridPtr(new nav_msgs::OccupancyGrid);
    *occ_map = get_map_.response.map;
    planner = boost::make_shared<
        ompl_planners_ros::MultipleCirclesReedsSheppCarPlanner>(pp, vp,
                                                                occ_map);
    planner->ss->getProblemDefinition()->setIntermediateSolutionCallback(
        boost::bind(&CarPlannerROSNode::solutionCallback, this, _1, _2, _3));

    if (mod_type == "CLiFF-map" or mod_type == "CLiFFMap") {
      ROS_INFO_STREAM("\x1b[34mCLiFF-map planning is activated.");

      cliffmap_client = std::make_shared<cliffmap_ros::CLiFFMapClient>();

      ob::OptimizationObjectivePtr DTCCostObjective(
          new ompl::mod::DTCOptimizationObjective(
              planner->ss->getSpaceInformation(), cliffmap_client->get(),
              pp.weight_d, pp.weight_q, pp.weight_c, vp.max_vehicle_speed));

      planner->ss->setOptimizationObjective(DTCCostObjective);

    } else if (mod_type == "STeF-map" or mod_type == "STeFMap") {
      ROS_INFO_STREAM("\x1b[34mSTeF-map planning is activated.");

      stefmap_client = std::make_shared<stefmap_ros::STeFMapClient>();

      ob::OptimizationObjectivePtr UCOO(
          new ompl::mod::UpstreamCriterionOptimizationObjective(
              planner->ss->getSpaceInformation(),
              stefmap_client->get(1352265000.0, 2),
              pp.weight_d, pp.weight_q, pp.weight_c));

      planner->ss->setOptimizationObjective(UCOO);

    } else if (mod_type == "GMMT-map" or mod_type == "GMMTMap") {
      ROS_INFO_STREAM("\x1b[34mGMMT-map planning is activated.");

      gmmtmap_client = std::make_shared<gmmtmap_ros::GMMTMapClient>();

      ob::OptimizationObjectivePtr UCOO(
          new ompl::mod::UpstreamCriterionOptimizationObjective(
              planner->ss->getSpaceInformation(), gmmtmap_client->get(),
              pp.weight_d, pp.weight_q, pp.weight_c));
      planner->ss->setOptimizationObjective(UCOO);
    } else {
      ROS_WARN_STREAM("Wrong or missing 'mod_type' parameter. Not using MoDs "
                      "for planning.");
    }

    save_path_sub = this->private_nh.subscribe(
        "save_path", 1, &CarPlannerROSNode::savePathCallback, this);
  }

  virtual ~CarPlannerROSNode() = default;

  void savePathCallback(const std_msgs::String &msg) {
    ROS_INFO_STREAM("\x1b[34mSaving path to file: " << msg.data.c_str());
    std::ofstream ofile(msg.data.c_str(), std::ios::out);
    if (!ofile) {
      ROS_ERROR_STREAM("Couldn't open paths file!");
      return;
    }

    for (const auto &pose : *poses_ptr) {
      ofile << pose.x << ", " << pose.y << ", " << pose.theta << std::endl;
    }

    ofile.close();
    ROS_INFO_STREAM("Successfully saved path to file: " << msg.data.c_str());
  }

  void solutionCallback(
      const ompl::base::Planner *planner,
      const std::vector<const ompl::base::State *> &solution_states,
      const ompl::base::Cost cost) {
    ROS_INFO_STREAM("New solution found with cost: \x1b[34m"
                    << std::fixed << std::setprecision(2) << cost.value());
  }

  void callback_fn(const geometry_msgs::PoseStampedConstPtr &msg, bool start) {
    if (start) {
      start_pose = geometry_msgs::Pose2DPtr(new geometry_msgs::Pose2D);
      start_pose->x = msg->pose.position.x;
      start_pose->y = msg->pose.position.y;
      start_pose->theta =
          2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);
    } else {
      if (!start_pose) {
        ROS_INFO_STREAM("\x1b[93mUse RViz to select the start pose first.");
        return;
      }
      // Reset previous solution path.
      poses_ptr.reset();
      poses_ptr = boost::shared_ptr<std::vector<geometry_msgs::Pose2D>>(
          new std::vector<geometry_msgs::Pose2D>);

      // Assign new goal.
      goal_pose = geometry_msgs::Pose2DPtr(new geometry_msgs::Pose2D);
      goal_pose->x = msg->pose.position.x;
      goal_pose->y = msg->pose.position.y;
      goal_pose->theta =
          2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);

      // Plan.
      planner->plan(*start_pose, *goal_pose, poses_ptr.get());
      ROS_INFO_STREAM("\x1b[34mPLANNING COMPLETE!");
    }
  }
};

int main(int argn, char *args[]) {
  ros::init(argn, args, "rscp_node");

  CarPlannerROSNode cprn;
  ros::NodeHandle nhandle;

  ros::Subscriber goalSub = nhandle.subscribe<geometry_msgs::PoseStamped>(
      "goal", 1,
      boost::bind(&CarPlannerROSNode::callback_fn, &cprn, _1, false));

  ros::Subscriber startSub = nhandle.subscribe<geometry_msgs::PoseStamped>(
      "start", 1,
      boost::bind(&CarPlannerROSNode::callback_fn, &cprn, _1, true));

  ros::spin();

  return 0;
}
