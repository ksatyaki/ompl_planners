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

void callback_fn(
    const geometry_msgs::PoseStampedConstPtr& goal,
    ompl_planners_ros::MultipleCirclesReedsSheppCarPlanner& planner,
    geometry_msgs::PoseArray* poses) {
  geometry_msgs::Pose startPose;
  startPose.position.x = 2.5;
  startPose.position.y = 9.0;
  startPose.orientation.w = cos(-M_PI / 4);
  startPose.orientation.z = sin(-M_PI / 4);

  planner.plan(startPose, goal->pose, poses);

  std::cout << std::endl;
  ROS_INFO_STREAM("\x1b[34m"
                  << "PLANNING COMPLETE!");
  std::cout << std::endl;

  return;
}

void saveGraphCallback(
    const std_msgs::EmptyConstPtr& empty,
    ompl_planners_ros::MultipleCirclesReedsSheppCarPlanner& planner) {
  ompl::base::PlannerData data(planner.ss->getSpaceInformation());
  planner.ss->getPlanner()->getPlannerData(data);
  std::fstream file_stream("/home/ksatyaki/graph.xml", std::ios_base::out);
  data.printGraphML(file_stream);
  ROS_INFO("Saved as xml.");
  return;
}

int main(int argn, char* args[]) {
  ros::init(argn, args, "rscp_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle nhandle;

  ros::Publisher path_pub = nh.advertise<geometry_msgs::PoseArray>("/path", 1);

  ros::ServiceClient map_client =
      nhandle.serviceClient<nav_msgs::GetMap>("static_map");
  map_client.waitForExistence();
  nav_msgs::GetMap get_map_;
  if (!map_client.call(get_map_)) {
    ROS_FATAL("Failed to call the map server for map!");
  }
  ROS_INFO_STREAM("\x1b[34m"
                  << "[PLANNER]: Map received!");

  ompl_planners_ros::PlannerParameters pp;
  ompl_planners_ros::VehicleParameters vp;

  nh.getParam("planner/weight_d", pp.weight_d);
  nh.getParam("planner/weight_c", pp.weight_c);
  nh.getParam("planner/weight_q", pp.weight_q);
  nh.getParam("planner/planning_time", pp.planning_time);
  nh.getParam("planner/path_resolution", pp.path_resolution);
  nh.getParam("planner/cliffmap_filename", pp.cliffmap_filename);
  nh.getParam("vehile/inflation_radius", vp.inflation_radius);
  nh.getParam("vehicle/turning_radius", vp.turning_radius);

  std::vector<geometry_msgs::Point> ftprnt =
      costmap_2d::makeFootprintFromParams(nh);

  for (const auto& ftprntpt : ftprnt) {
    ROS_INFO("><>< %lf, %lf ><><", ftprntpt.x, ftprntpt.y);
    vp.footprint.AddVertex(ftprntpt.x, ftprntpt.y);
  }

  ROS_INFO_STREAM("\x1b[34m"
                  << "[PLANNER]: Parameters obtained!");
  ROS_INFO_STREAM(
      "\x1b[34m" << std::endl
                 << "*** ***************************************" << std::endl
                 << "*** wd: " << pp.weight_d << std::endl
                 << "*** wq: " << pp.weight_q << std::endl
                 << "*** wc: " << pp.weight_c << std::endl
                 << "*** Planning time: " << pp.planning_time << std::endl
                 << "*** Path resolution: " << pp.path_resolution << std::endl
                 << "*** Cliffmap file: " << pp.cliffmap_filename << std::endl
                 << "*** Turning radius: " << vp.turning_radius << std::endl
                 << "*** Inflation radius: " << vp.inflation_radius << std::endl
                 << "*** Check footprint manually." << std::endl
                 << "*** ***************************************");

  nav_msgs::OccupancyGridPtr occ_map =
      nav_msgs::OccupancyGridPtr(new nav_msgs::OccupancyGrid);
  *occ_map = get_map_.response.map;
  ompl_planners_ros::MultipleCirclesReedsSheppCarPlanner planner(pp, vp,
                                                                 occ_map);

  geometry_msgs::PoseArray poses;
  poses.header.frame_id = "map";
  poses.header.stamp = ros::Time::now();

  ros::Subscriber goalSub = nhandle.subscribe<geometry_msgs::PoseStamped>(
      "goal", 1, boost::bind(&callback_fn, _1, planner, &poses));

  ros::Subscriber saveSub = nhandle.subscribe<std_msgs::Empty>(
      "saveit", 1, boost::bind(&saveGraphCallback, _1, planner));

  ros::Rate rate(5);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    if (not poses.poses.empty()) {
      path_pub.publish(poses);
    }
  }

  return 0;
}
