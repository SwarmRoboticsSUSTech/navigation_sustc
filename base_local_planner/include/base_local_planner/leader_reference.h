#ifndef LEADER_RERENCE_H_
#define LEADER_RERENCE_H_

#include <base_local_planner/trajectory_cost_function.h>

#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/map_grid.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace base_local_planner {

/**
 */
enum Formation { Line, Wedge, Column, Diamond};

/**
 */
class LeaderRerence {
public:
  LeaderRerence(tf::TransformListener* tf, Formation pF, double pDS);
  ~LeaderRerence() {}
  bool getDesiredPosition(geometry_msgs::PoseStamped& desiredPosition);

protected:
  Formation f;
  double desiredSpaceing;

private:
  bool getRobotPose(geometry_msgs::PoseStamped& robot1_pose) const;
  tf::TransformListener* tfListener_;
};

} /* namespace base_local_planner */
#endif /* LEADER_RERENCE_H_ */
