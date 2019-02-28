/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   This file is part of ompl_planners_ros.
 *
 *   ompl_planners_ros is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ompl_planners_ros is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with ompl_planners_ros.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <ros/console.h>
#include <ros/ros.h>

#include <costmap_2d/footprint.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>

#include <mrpt/math/CPolygon.h>
#include <boost/function.hpp>

#include "ompl_planners_ros/mc_reeds_shepp_car_planner.hpp"
#include "ompl_planners_ros/visualization.hpp"

class CarPlannerROSNode {
 protected:
  ros::NodeHandle nh;
  ros::NodeHandle nhandle;

  ompl_planners_ros::PlannerParameters pp;
  ompl_planners_ros::VehicleParameters vp;

  ros::Publisher path_pub;
  ros::ServiceClient map_client;

  geometry_msgs::PosePtr start_pose;
  geometry_msgs::PoseArrayPtr poses_ptr;
  boost::shared_ptr<ompl_planners_ros::MultipleCirclesReedsSheppCarPlanner>
      planner;

 public:
  CarPlannerROSNode() : nh("~") {
    path_pub = nh.advertise<geometry_msgs::PoseArray>("path", 1);
    map_client = nhandle.serviceClient<nav_msgs::GetMap>("static_map");

    map_client.waitForExistence();

    nav_msgs::GetMap get_map_;
    if (!map_client.call(get_map_)) {
      ROS_FATAL("Failed to call the map server for map!");
    }

    ROS_INFO_STREAM("\x1b[34m"
                    << "[PLANNER]: Map received!");

    nh.getParam("planner/weight_d", pp.weight_d);
    nh.getParam("planner/weight_c", pp.weight_c);
    nh.getParam("planner/weight_q", pp.weight_q);
    nh.getParam("planner/planning_time", pp.planning_time);
    nh.getParam("planner/path_resolution", pp.path_resolution);
    nh.getParam("planner/cliffmap_filename", pp.cliffmap_filename);
    nh.getParam("planner/publish_viz_markers", pp.publish_viz_markers);

    nh.getParam("vehile/inflation_radius", vp.inflation_radius);
    nh.getParam("vehicle/turning_radius", vp.turning_radius);

    std::vector<geometry_msgs::Point> ftprnt =
        costmap_2d::makeFootprintFromParams(nh);

    for (const auto& ftprntpt : ftprnt) {
      ROS_INFO("><>< %lf, %lf ><><", ftprntpt.x, ftprntpt.y);
      vp.footprint.AddVertex(ftprntpt.x, ftprntpt.y);
    }

    ROS_INFO_STREAM(
        "\x1b[34m" << std::endl
                   << "*** ***************************************" << std::endl
                   << "*** wd: " << pp.weight_d << std::endl
                   << "*** wq: " << pp.weight_q << std::endl
                   << "*** wc: " << pp.weight_c << std::endl
                   << "*** Publish viz markers? " << pp.publish_viz_markers
                   << std::endl
                   << "*** Planning time: " << pp.planning_time << std::endl
                   << "*** Path resolution: " << pp.path_resolution << std::endl
                   << "*** Cliffmap file: " << pp.cliffmap_filename << std::endl
                   << "*** Turning radius: " << vp.turning_radius << std::endl
                   << "*** Inflation radius: " << vp.inflation_radius
                   << std::endl
                   << "*** Check footprint manually." << std::endl
                   << "*** ***************************************");

    nav_msgs::OccupancyGridPtr occ_map =
        nav_msgs::OccupancyGridPtr(new nav_msgs::OccupancyGrid);
    *occ_map = get_map_.response.map;
    planner = boost::make_shared<
        ompl_planners_ros::MultipleCirclesReedsSheppCarPlanner>(pp, vp,
                                                                occ_map);
    //planner->ss->getProblemDefinition()->setIntermediateSolutionCallback(
    //    boost::bind(&CarPlannerROSNode::solutionCallback, this, _1, _2, _3));
  }

  virtual ~CarPlannerROSNode(){};

  void solutionCallback(
      const ompl::base::Planner* planner,
      const std::vector<const ompl::base::State*>& solution_states,
      const ompl::base::Cost cost) {
    ROS_INFO_STREAM("\x1b[34m"
                    << "New solution found with cost: " << cost.value());
  }

  void callback_fn2(const geometry_msgs::PoseStampedConstPtr& start) {
    start_pose = geometry_msgs::PosePtr(new geometry_msgs::Pose);
    *start_pose = start->pose;
  }

  void callback_fn(const geometry_msgs::PoseStampedConstPtr& goal) {
    if (!start_pose) {
      ROS_INFO_STREAM("\x1b[93mUse RViz to select the start pose first.");
      return;
    }

    poses_ptr.reset();
    poses_ptr = geometry_msgs::PoseArrayPtr(new geometry_msgs::PoseArray);
    poses_ptr->header.frame_id = "map";
    poses_ptr->header.stamp = ros::Time::now();

    planner->plan(*start_pose, goal->pose, poses_ptr.get());

    std::cout << std::endl;
    ROS_INFO_STREAM("\x1b[34m[CALLBACK]: PLANNING COMPLETE!");
    ROS_INFO_STREAM(
        "\x1b[34m[CALLBACK]: Cost: " << planner->ss->getPlanner()
                                            ->as<ompl::geometric::RRTstar>()
                                            ->getBestCost());
    std::cout << std::endl;
    fflush(stdout);

    return;
  }
};

int main(int argn, char* args[]) {
  ros::init(argn, args, "rscp_node");

  CarPlannerROSNode cprn;
  ros::NodeHandle nhandle;

  ros::Subscriber goalSub = nhandle.subscribe<geometry_msgs::PoseStamped>(
      "goal", 1, &CarPlannerROSNode::callback_fn, &cprn);

  ros::Subscriber startSub = nhandle.subscribe<geometry_msgs::PoseStamped>(
      "start", 1, &CarPlannerROSNode::callback_fn2, &cprn);

  ros::spin();

  return 0;
}
