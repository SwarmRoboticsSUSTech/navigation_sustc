#include <base_local_planner/leader_reference.h>
#include <stdexcept>

namespace base_local_planner {

LeaderRerence::LeaderRerence(tf::TransformListener* tf, Formation pF, double pDS)
{
  tfListener_ = tf;
  f = pF;
  desiredSpaceing = pDS;
}

bool LeaderRerence::getRobotPose(geometry_msgs::PoseStamped& robot1_pose) const
{
  std::string global_frame = "/map";
  std::string robot1_base_frame = "/robot1_base_footprint";
  tf::Stamped<tf::Pose> global_pose;
  global_pose.setIdentity();
  tf::Stamped < tf::Pose > robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = robot1_base_frame;
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

//  ROS_INFO("In LeaderRerence::getRobotPose!!");

  // get the global pose of the robot
  try
  {
//    ROS_INFO(" getting robot pose");
    tfListener_->transformPose(global_frame, robot_pose, global_pose);
  }
  catch (tf::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }

  double transform_tolerance = 0.1;
  // check global_pose timeout
  if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.stamp_.toSec(), transform_tolerance);
    return false;
  }

  robot1_pose.pose.position.x = global_pose.getOrigin().getX();
  robot1_pose.pose.position.y = global_pose.getOrigin().getY();
  robot1_pose.pose.position.z = global_pose.getOrigin().getZ();
  robot1_pose.pose.orientation.x = global_pose.getRotation().getX();
  robot1_pose.pose.orientation.y = global_pose.getRotation().getY();
  robot1_pose.pose.orientation.z = global_pose.getRotation().getZ();
  robot1_pose.pose.orientation.w = global_pose.getRotation().getW();
  robot1_pose.header.stamp = global_pose.stamp_;
  robot1_pose.header.frame_id = robot1_base_frame;
//  tf::pointStampedTFToMsg(global_pose, robot1_pose);
  return true;
}

bool LeaderRerence::getDesiredPosition(geometry_msgs::PoseStamped& desiredPosition )
{
  geometry_msgs::PoseStamped leaderPos;

//  can't get the pose os leader
  if(!getRobotPose(leaderPos))
  {
//    ROS_INFO("had got robot pose, count = %d", count);
//    ROS_INFO("getDesiredPosition:out of while");
    return false;
  }

  desiredPosition = leaderPos;
  ros::NodeHandle nh;

  std::string mNameSpace = nh.getNamespace();
  mNameSpace = mNameSpace.substr(2, mNameSpace.length() - 2);

  if(mNameSpace == "robot1")
  {
    return false;
  }

  else if(f == Column)
  {
    if(mNameSpace == "robot2")
    {
      desiredPosition.pose.position.y = leaderPos.pose.position.y - 2 * desiredSpaceing;
    }
    else if(mNameSpace == "robot3")
    {
      desiredPosition.pose.position.y = leaderPos.pose.position.y - desiredSpaceing;
    }
    else if(mNameSpace == "robot4")
    {
      desiredPosition.pose.position.y = leaderPos.pose.position.y - 3 * desiredSpaceing;
    }
  }
  else if(f == Diamond)
  {
    if(mNameSpace == "robot2")
    {
      desiredPosition.pose.position.y = leaderPos.pose.position.y - desiredSpaceing;
      desiredPosition.pose.position.x = leaderPos.pose.position.x + desiredSpaceing;
    }
    else if(mNameSpace == "robot3")
    {
      desiredPosition.pose.position.y = leaderPos.pose.position.y - desiredSpaceing;
      desiredPosition.pose.position.x = leaderPos.pose.position.x - desiredSpaceing;
    }
    else if(mNameSpace == "robot4")
    {
      desiredPosition.pose.position.y = leaderPos.pose.position.y - 2 * desiredSpaceing;
    }
  }
  else if(f == Line)
  {
    if(mNameSpace == "robot2")
    {
      desiredPosition.pose.position.x = leaderPos.pose.position.x + desiredSpaceing;
    }
    else if(mNameSpace == "robot3")
    {
      desiredPosition.pose.position.x = leaderPos.pose.position.x - desiredSpaceing;
    }
    else if(mNameSpace == "robot4")
    {
      desiredPosition.pose.position.x = leaderPos.pose.position.x - 2 * desiredSpaceing;
    }
  }
  else if(f == Wedge)
  {
    if(mNameSpace == "robot2")
    {
      desiredPosition.pose.position.y = leaderPos.pose.position.y - desiredSpaceing;
      desiredPosition.pose.position.x = leaderPos.pose.position.x + desiredSpaceing;
    }
    else if(mNameSpace == "robot3")
    {
      desiredPosition.pose.position.x = leaderPos.pose.position.x - desiredSpaceing;
    }
    else if(mNameSpace == "robot4")
    {
      desiredPosition.pose.position.y = leaderPos.pose.position.y - desiredSpaceing;
      desiredPosition.pose.position.x = leaderPos.pose.position.x - 2 * desiredSpaceing;
    }
  }
  return true;
}

} /* namespace base_local_planner */
