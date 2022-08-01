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

using namespace pcl;

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
  sensor_msgs::PointCloud2ConstPtr pointcloud2_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(req.camera_topic, timeout);

  PointCloud<PointXYZRGB> temp_transform_cloud;
  fromROSMsg(*pointcloud2_msg, temp_transform_cloud);

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

  int sum_x = 0;
  int sum_y = 0;
  int sum_z = 0;

  int size = temp_transform_cloud.points.size();
  for (i=0;i<size;i++) {
    sum_x += temp_transform_cloud.points[i].x;
    sum_y += temp_transform_cloud.points[i].y;
    sum_z += temp_transform_cloud.points[i].z;
  }

  int avg_x = sum_x / i;
  int avg_y = sum_y / i;
  int avg_z = sum_y / i;

  ROS_INFO_STREAM("avg x point position: " << avg_x);
  ROS_INFO_STREAM("avg y point position: " << avg_y);
  ROS_INFO_STREAM("avg z point position: " << avg_z);

  pcl_ros::transformPointCloud("base_link", temp_transform_cloud, temp_transform_cloud, listener);

  sensor_msgs::PointCloud2 transformed_pointcloud_msg;
  toROSMsg(temp_transform_cloud, transformed_pointcloud_msg);

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
