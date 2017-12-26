/*********************************************************************
 *
 *********************************************************************/

#ifndef FORMATION_COST_FUNCTION_H_
#define FORMATION_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/map_grid.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

#include <math.h>

namespace base_local_planner {

/**
 * when scoring a trajectory according to the values in mapgrid, we can take
 *return the value of the last point (if no of the earlier points were in
 * return collision), the sum for all points, or the product of all (non-zero) points
 */
enum CostType { mLast, mSum, mProduct};


class FormationCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
  FormationCostFunction(tf::TransformListener* tf, CostType aggregationType = mLast, double xshift = 0.0, double yshift = 0.0);

  ~FormationCostFunction() {}

  /**
   * set line segments on the grid with distance 0, resets the grid
   */
  void setTargetPoses(geometry_msgs::PoseStamped target_poses, bool gotDesiredPosition);

  bool prepare() {return true;}

  double scoreTrajectory(Trajectory &traj);

  void setXShift(double xshift) {xshift_ = xshift;}
  void setYShift(double yshift) {yshift_ = yshift;}
  bool tranformPointToGlobal(geometry_msgs::Point& obs);

private:
  geometry_msgs::PoseStamped target_poses_;
  CostType aggregationType_;
  /// xshift and yshift allow scoring for different
  // ooints of robots than center, like fron or back
  // this can help with alignment or keeping specific
  // wheels on tracks both default to 0
  double xshift_;
  double yshift_;

  tf::TransformListener* tfListener_;
  std::string nameSpace_;
  bool gotDesiredPosition_;
};

} /* namespace base_local_planner */
#endif /* FORMATION_COST_FUNCTION_H_ */
