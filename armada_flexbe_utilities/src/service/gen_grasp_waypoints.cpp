#include "ros/ros.h"
#include "armada_flexbe_utilities/GenGraspWaypoints.h"

/**
 * Generate a list of grasp waypoint sets.
 *
 * Given a list of grasp target candidates, generate a set of waypoint poses (pre-approach, target pose, post-retreat) for each candidate.
 *
 * @param[in] req gpd_ros/GraspConfigList Container of grasp target candidates generated by GPD algorithm.
 * @param[out] res armada_flexbe_utilities/GraspPosesList Container of sets of pose waypoints for grasp target candidates.
 * @return Bool Service completion result.
 */
bool executeCB(armada_flexbe_utilities::GenGraspWaypoints::Request  &req,
         armada_flexbe_utilities::GenGraspWaypoints::Response &res)
{
  // use list in function for simplicicy unless we know the size of things
  unsigned long n = req.grasp_msg_list.grasps.size();
  armada_flexbe_utilities::GraspPoses grasp_pose_arr[n];
  for (unsigned long i = 0; i < n; ++i) {
    // do something here
  }

  armada_flexbe_utilities::GraspPosesList grasp_poses_list;
  res.grasp_poses_list = grasp_poses_list;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gen_grasp_waypoints_service");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("gen_grasp_waypoints", executeCB);
  ROS_INFO("Ready to generate grasping waypoints.");
  ros::spin();

  return 0;
}
