#include <ros/ros.h>
#include "armada_flexbe_utilities/GetPointCloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

/**
 * Add current camera pointcloud msg to array of pointcloud messages.
 *
 * Given a topic and existing array of PointCloud2 messages, add another PointCloud2 message from the given topic to the array of messages.
 *
 * @param[in] req string String containing desired camera message topic.
 * @param[out] res sensor_msgs/PointCloud2 PointCloud2 message.
 * @return Bool Service completion result.
 */
bool getPointCloud(armada_flexbe_utilities::GetPointCloud::Request &req,
                   armada_flexbe_utilities::GetPointCloud::Response &res)
{
  ROS_WARN("Executing GetPointCloud Service");
  ros::Duration timeout(5);

  // subscribe to the pointcloud topic for just one message
  // this is a one-line subscriber, it doesn't need a distinct callback but it's nothing special
  sensor_msgs::PointCloud2ConstPtr pointcloud2_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(req.camera_topic, timeout);

  // -------------------------------------- now we have a message --------------------------------------

  // create a pcl::pointcloud object
  pcl::PointCloud<pcl::PointXYZRGB> temp_transform_cloud;
  // turn sensor_msgs pointcloud into pcl pointcloud that we can work with (now it is workable data but no longer a message)
  fromROSMsg(*pointcloud2_msg, temp_transform_cloud);

  // -------------------------------------- now we have a piece of data we created from the message --------------------------------------

  // now we can do whatever we want with the PointXYZRGB object we have populated
  // what we want to do here is transform it into the correct frame
  ros::Time stamp = ros::Time(0);
  tf::StampedTransform transform;
  tf::TransformListener listener;
  pcl_conversions::toPCL(stamp, temp_transform_cloud.header.stamp);
  try
  {
    listener.waitForTransform("base_link", temp_transform_cloud.header.frame_id, stamp, ros::Duration(5.0));
    listener.lookupTransform("base_link", temp_transform_cloud.header.frame_id, stamp, transform);
  } catch (tf::TransformException err)
  {
    ROS_ERROR("%s", err.what());
  }
  pcl_ros::transformPointCloud("base_link", temp_transform_cloud, temp_transform_cloud, listener);

  // -------------------------------------- we have transformed our new data into something useful

  // create a new sensor_msgs pointcloud object that we can send (this is a message type)
  sensor_msgs::PointCloud2 transformed_pointcloud_msg;
  // turn the transformed pointcloud back into a message
  toROSMsg(temp_transform_cloud, transformed_pointcloud_msg);

  // -------------------------------------- we turned our data back into a message we can send somewhere else for further processing --------------------------------------

  // return the message as a response to the service request (so it can be sent out by someone else on some topic)
  res.cloud_out = transformed_pointcloud_msg;
  ROS_WARN("Finished GetPointCloud Service");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_pointcloud_service");
  ros::NodeHandle nh;

  ros::ServiceServer getPointCloudService = nh.advertiseService("get_pointcloud", getPointCloud);
  ROS_WARN("get_pointcloud_service Ready.");
  ros::spin();

  return 0;
}
