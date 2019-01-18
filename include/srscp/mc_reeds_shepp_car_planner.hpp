#include "srscp/mc_state_validity_checker.hpp"

#include <boost/math/constants/constants.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>

#include <mrpt/math/CPolygon.h>
#include <nav_msgs/OccupancyGrid.h>


namespace mm = mrpt::maps;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace srscp {

struct State {
	double x;
	double y;
	double theta;
};

class MultipleCirclesReedsSheppCarPlanner {
	mm::COccupancyGridMap2D grid_map_;
	std::vector<double> x_coords_;
	std::vector<double> y_coords_;
	bool no_map_;
	double robot_radius_;

public:
	boost::shared_ptr<og::SimpleSetup> ss;
	MultipleCirclesReedsSheppCarPlanner(const char *mapFileName,
			double mapResolution, double robotRadius,
			const mrpt::math::CPolygon &footprint);
	MultipleCirclesReedsSheppCarPlanner(
			const nav_msgs::OccupancyGridConstPtr &occ_map_ptr,
			double robotRadius, const mrpt::math::CPolygon &footprint,
			double turningRadius);

	~MultipleCirclesReedsSheppCarPlanner() {
	}

	bool plan(const State &startState, const State &goalState,
			std::vector<State> &path,
			double distanceBetweenPathPoints = 0.5);
};
}
