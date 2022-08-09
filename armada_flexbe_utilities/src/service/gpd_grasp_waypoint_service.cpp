/*
 * This code uses a modification of the implemetation presented at:
 * https://gist.github.com/tkelestemur/60401be131344dae98671b95d46060f8 for using GPD
 *
 * Please refer to the gist provided for more information
 *
 */

#include "ros/ros.h"
#include "armada_flexbe_utilities/GPDGraspWaypoints.h"
#include "armada_flexbe_utilities/GraspPoses.h"
#include "armada_flexbe_utilities/GraspPosesList.h"
#include <gpd_ros/GraspConfigList.h>
#include <gpd_ros/GraspConfig.h>
#include <tf/transform_listener.h>

using namespace std;

class GraspWaypointservice
{
protected:

  ros::ServiceServer graspWaypointService;
  double gripper_offset;
  double approach_dist;
  double retreat_dist;

  string global_frame;
  string robot_frame;

public:

  /**
   * Class Constructor.
   *
   * Constructor for SetGripperService class.
   *
   * @param[in] nh A ROS NodeHandle object.
   */
  GraspWaypointservice(ros::NodeHandle nh)
  {
    graspWaypointService = nh.advertiseService("calculate_grasp_waypoints", &GraspWaypointservice::calculateGraspWaypoints, this);
    nh.getParam("/end_effector/gripper_offset", gripper_offset);
    nh.getParam("/end_effector/approach_dist", approach_dist);
    nh.getParam("/end_effector/retreat_dist", retreat_dist);
    nh.getParam("/reference_frame/global_frame", global_frame);
    nh.getParam("/reference_frame/robot_frame", robot_frame);
  }

  /**
   * Generate a list of grasp waypoint sets (pre, target and post poses) to be used with the GPD (grasp pose detection) algorithm.
   *
   * Given a list of grasp target candidates, generate a set of waypoint poses (pre-approach, target pose, post-retreat) for each candidate.
   *
   * @param[in] req gpd_ros/GraspConfigList Container of grasp target candidates generated by GPD algorithm.
   * @param[out] res armada_flexbe_utilities/GraspPosesList Container of sets of pose waypoints for grasp target candidates.
   * @return Bool Service completion result.
   */
  bool calculateGraspWaypoints(armada_flexbe_utilities::GPDGraspWaypoints::Request &req,
                               armada_flexbe_utilities::GPDGraspWaypoints::Response &res)
  {
    ROS_WARN("Executing GPDGraspWaypoints Service");

    std::vector<armada_flexbe_utilities::GraspPoses> grasp_poses_vect;
    armada_flexbe_utilities::GraspPosesList msg;

    unsigned long candidates_list_size = req.grasp_msg_list.grasps.size();
    for (unsigned long i = 0; i < candidates_list_size; ++i) {
      res.grasp_poses_list.poses.push_back(calculateGraspPoses(req.grasp_msg_list.grasps[i]));
    }

    ROS_WARN("Finished GPDGraspWaypoints Service");
    return true;
  }

  /**
   * Generate grasp picking poses from a grasp candidate.
   *
   * Given a grasp candidate, generate a set of poses (pre-grasp, target pose, post-grasp).
   * Source Gist: https://gist.github.com/tkelestemur/60401be131344dae98671b95d46060f8
   *
   * @param[in] req gpd_ros/GraspConfig A grasp candidate generated by GPD algorithm.
   * @param[in] grasp_offset double Distance from planning end point to actual gripper TCP.
   * @param[in] pregrasp_dist double Distance from target grasp position to start approaching.
   * @param[in] postgrasp_dist double Distance from target grasp position to retreat to after grasp.
   * @return armada_flexbe_utilities::GraspPoses A set of poses for picking.
   */
  armada_flexbe_utilities::GraspPoses calculateGraspPoses(gpd_ros::GraspConfig grasp_msg)
  {
    armada_flexbe_utilities::GraspPoses grasp_poses;

    tf::Matrix3x3 rot_matrix_grasp_base(-grasp_msg.axis.x, grasp_msg.binormal.x, grasp_msg.approach.x,
                                        -grasp_msg.axis.y, grasp_msg.binormal.y, grasp_msg.approach.y,
                                        -grasp_msg.axis.z, grasp_msg.binormal.z, grasp_msg.approach.z);

    tf::Vector3 tr_grasp_base(grasp_msg.position.x, grasp_msg.position.y, grasp_msg.position.z);
    tf::Transform tf_grasp_base(rot_matrix_grasp_base, tr_grasp_base);
    tf::StampedTransform tf_base_odom;

    try {
      tf::TransformListener listener;
      listener.waitForTransform(global_frame, robot_frame, ros::Time::now(), ros::Duration(3.0) );
      listener.lookupTransform(global_frame, robot_frame, ros::Time::now(), tf_base_odom);
    } catch (tf::TransformException err) {
      ROS_ERROR("%s", err.what());
    }

    tf::Transform tf_grasp_odom_(tf::Quaternion(0, 0, -M_PI/4 - M_PI/16, 1), tf::Vector3(0, 0, gripper_offset));
    tf::Transform tf_grasp_odom = tf_base_odom * tf_grasp_base * tf_grasp_odom_;
    tf::poseTFToMsg(tf_grasp_odom, grasp_poses.target);

    tf::Transform tf_pregrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, approach_dist));
    tf::Transform tf_pregrasp_odom = tf_grasp_odom * tf_pregrasp_odom_;
    tf::poseTFToMsg(tf_pregrasp_odom, grasp_poses.pre);

    tf::Transform tf_aftergrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, retreat_dist));
    tf::Transform tf_aftergrasp_odom = tf_grasp_odom * tf_aftergrasp_odom_;
    tf::poseTFToMsg(tf_aftergrasp_odom, grasp_poses.post);

    return grasp_poses;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gen_grasp_waypoints_service");
  ros::NodeHandle nh;

  GraspWaypointservice graspWaypointService = GraspWaypointservice(nh);
  ROS_WARN("gen_grasp_waypoints_service Ready.");
  ros::spin();

  return 0;
}
