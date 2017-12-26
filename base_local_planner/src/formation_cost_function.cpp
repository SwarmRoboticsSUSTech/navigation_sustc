/*********************************************************************
 *
 *********************************************************************/

#include <base_local_planner/formation_cost_function.h>

namespace base_local_planner {

FormationCostFunction::FormationCostFunction(tf::TransformListener* tf, CostType aggregationType, double xshift, double yshift):
  aggregationType_(aggregationType),
  xshift_(xshift),
  yshift_(yshift)
{
  tfListener_ = tf;
  ros::NodeHandle nh;
  nameSpace_ = nh.getNamespace();
  nameSpace_ = nameSpace_.substr(1, nameSpace_.length() - 1);
//  ROS_INFO("FormationCostFunction namespace: %s", nameSpace_.c_str());
}

void FormationCostFunction::setTargetPoses(geometry_msgs::PoseStamped target_poses, bool gotDesiredPosition)
{
  target_poses_ = target_poses;
  gotDesiredPosition_ = gotDesiredPosition;
}

bool FormationCostFunction::tranformPointToGlobal(geometry_msgs::Point& obs)
{
  std::string new_global_frame = "/map";
  ros::Time transform_time = ros::Time::now();
  std::string tf_error;
  std::string global_frame = nameSpace_ + "_odom";
  double tf_tolerance = 0.1;

  if (!tfListener_->waitForTransform(new_global_frame, global_frame, transform_time, ros::Duration(tf_tolerance),
                            ros::Duration(0.01), &tf_error))
  {
    ROS_ERROR("Transform between %s and %s with tolerance %.2f failed: %s.", new_global_frame.c_str(),
              global_frame.c_str(), tf_tolerance, tf_error.c_str());
    return false;
  }

  try
  {
    geometry_msgs::PointStamped origin;
    origin.header.frame_id = global_frame;
    origin.header.stamp = transform_time;
    origin.point = obs;

    // we need to transform the origin of the observation to the new global frame
    tfListener_->transformPoint(new_global_frame, origin, origin);
    obs = origin.point;
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("TF Error attempting to transform an observation from %s to %s: %s", global_frame.c_str(),
              new_global_frame.c_str(), ex.what());
    return false;
  }
  return true;
}

double FormationCostFunction::scoreTrajectory(Trajectory &traj) {
  double cost = 0.0;
  double px, py, pth;
  double gx, gy;
  double grid_dist;
  geometry_msgs::Point trajPoint;

  gx = target_poses_.pose.position.x;
  gy = target_poses_.pose.position.y;

  if(!gotDesiredPosition_)
  {
//    ROS_INFO("FormationCostFunction 0");
    return 0;
  }

  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
    traj.getPoint(i, px, py, pth);
    trajPoint.x = px;
    trajPoint.y = py;
    trajPoint.z = 0;

//  transform coordination from /odom to /map
    for(int count = 1; count < 5; count++)
    {
      if(tranformPointToGlobal(trajPoint))
      {
        break;
      }
    }
    px = trajPoint.x;
    py = trajPoint.y;

    if(i == 0)
    {
//      ROS_INFO("traj px: %f, py: %f", px, py);
//      ROS_INFO("target_poses gx: %f, gy : %f", gx, gy);
//      ROS_INFO("FormationCostFunction namespace: %s", nameSpace_.c_str());
    }

    grid_dist = sqrt(pow((px - gx), 2) + pow((py + gy), 2));

    switch( aggregationType_ ) {
    case mLast:
      cost = grid_dist;
      break;
    case mSum:
      cost += grid_dist;
      break;
    case mProduct:
      if (cost > 0) {
        cost *= grid_dist;
      }
      break;
    }
  }
//  ROS_INFO("FormationCost %f", cost);
  return cost;
}

} /* namespace base_local_planner */
