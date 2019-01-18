#include <ros/console.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>

#include <boost/function.hpp>
#include <mrpt/math/CPolygon.h>

#include "srscp/mc_reeds_shepp_car_planner.hpp"

void callback_fn(const geometry_msgs::PoseStampedConstPtr& goal,
                 srscp::MultipleCirclesReedsSheppCarPlanner& planner,
                 std::vector<srscp::State>& path,
                 geometry_msgs::PoseArrayPtr& poses) {
  planner.plan({2.0, 2.0, 0.0},
               {goal->pose.position.x, goal->pose.position.y,
                2 * atan2(goal->pose.orientation.z, goal->pose.orientation.w)},
               path, 0.5);

  std::cout << std::endl;
  ROS_INFO_STREAM("\x1b[34m"
                  << "Planning Complete!");
  std::cout << std::endl;

  for (auto p : path) {
    geometry_msgs::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.orientation.z = sin(p.theta / 2);
    pose.orientation.w = cos(p.theta / 2);
    poses->poses.push_back(pose);
  }
  return;
}

void saveGraphCallback(const std_msgs::EmptyConstPtr& empty, srscp::MultipleCirclesReedsSheppCarPlanner& planner) {
	ompl::base::PlannerData data(planner.ss->getSpaceInformation());
	planner.ss->getPlanner()->getPlannerData(data);
	std::fstream file_stream("/home/ksatyaki/graph.xml", std::ios_base::out);
	data.printGraphML(file_stream);
	ROS_INFO("Saved as xml.");
	return;
}

int main(int argn, char* args[]) {
  ros::init(argn, args, "rscp_node");
  ros::NodeHandle nh;

  ros::Publisher path_pub = nh.advertise<geometry_msgs::PoseArray>("/path", 1);

  ros::ServiceClient map_client =
      nh.serviceClient<nav_msgs::GetMap>("static_map");
  map_client.waitForExistence();
  nav_msgs::GetMap get_map_;
  if (!map_client.call(get_map_)) {
    ROS_FATAL("Failed to call the map server for map!");
  }
  ROS_INFO("Got a map!");

  mrpt::math::CPolygon footprint;
  footprint.AddVertex(0.5, 0.5);
  footprint.AddVertex(0.5, -0.5);
  footprint.AddVertex(-0.5, 0.5);
  footprint.AddVertex(-0.5, -0.5);

  // srscp::MultipleCirclesReedsSheppCarPlanner planner(args[1],
  // std::atof(args[2]), 0.25, footprint);
  nav_msgs::OccupancyGridPtr occ_map =
      nav_msgs::OccupancyGridPtr(new nav_msgs::OccupancyGrid);
  *occ_map = get_map_.response.map;
  srscp::MultipleCirclesReedsSheppCarPlanner planner(occ_map, 0.25, footprint, 0.25);

  std::vector<srscp::State> path;

  geometry_msgs::PoseArrayPtr poses = geometry_msgs::PoseArrayPtr(new geometry_msgs::PoseArray);
  poses->header.frame_id = "map";
  poses->header.stamp = ros::Time::now();

  ros::Subscriber goalSub = nh.subscribe<geometry_msgs::PoseStamped>(
      "goal", 1, boost::bind(&callback_fn, _1, planner, path, poses));

  ros::Subscriber saveSub = nh.subscribe<std_msgs::Empty>(
        "saveit", 1, boost::bind(&saveGraphCallback, _1, planner));

  ros::Rate rate(5);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
    if (not poses->poses.empty()) path_pub.publish(poses);
  }

  return 0;
}
