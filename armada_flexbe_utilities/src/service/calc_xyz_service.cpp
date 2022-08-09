#include <ros/ros.h>
#include "armada_flexbe_utilities/CalculateXYZ.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <float.h>


//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/passthrough.h>

using namespace pcl;


/**
 * Perform Calculation (planar) for object XYZ on a PointCloud2 message.
 *
 * Given a PointCloud2 message, perform Calculation (planar) for object XYZ  and provide the resulting PointCloud2 message.
 * More information about pcl filters at: https://pcl.readthedocs.io/projects/tutorials/en/master/#
 *
 * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
 * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
 * @return Bool Service completion result.
 */
bool calculateXYZ(armada_flexbe_utilities::CalculateXYZ::Request &req,
                     armada_flexbe_utilities::CalculateXYZ::Response &res)
{
  ROS_WARN("Executing Calculation Service for object's XYZ");
  PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);
  fromROSMsg(req.cloud_in, *temp_cloud);

  ros::Time stamp = ros::Time(0);
  tf::StampedTransform transform;
  tf::TransformListener listener;
  pcl_conversions::toPCL(stamp, temp_cloud->header.stamp);
  try
  {
    listener.waitForTransform("base_link", temp_cloud->header.frame_id, stamp, ros::Duration(5.0));
    listener.lookupTransform("base_link", temp_cloud->header.frame_id, stamp, transform);
  } catch (tf::TransformException err)
  {
    ROS_ERROR("%s", err.what());
  }
  pcl_ros::transformPointCloud("base_link", *temp_cloud, *temp_cloud, listener);

  unsigned long size = temp_cloud->points.size();
  ROS_INFO_STREAM("size: " << size);

  double sum_x = 0;
  double sum_y = 0;
  double sum_z = 0;
  unsigned long i = 0;

//  double x_max = temp_cloud->points[0].x;
//  double x_min = temp_cloud->points[size - 1].x;
//  double x_mid = (x_max - x_min) + x_min;

//  double y_max = temp_cloud->points[0].y;
//  double y_min = temp_cloud->points[size - 1].y;
//  double y_mid = (y_max - y_min) + y_min;

//  double z_max = temp_cloud->points[0].z;
//  double z_min = temp_cloud->points[size - 1].z;
//  double z_mid = (z_max - z_min) + z_min;

  std::vector<double> x_vals;

    double x_max = DBL_MIN;
    double x_min = DBL_MAX;
    double y_max = DBL_MIN;
    double y_min = DBL_MAX;
    double z_max = DBL_MIN;
    double z_min = DBL_MAX;

  for (i;i<size;i++) {
    if (temp_cloud->points[i].x > x_max)
      x_max = temp_cloud->points[i].x;
    if (temp_cloud->points[i].x < x_min)
      x_min = temp_cloud->points[i].x;
  }

  ROS_INFO_STREAM("x_max: " << x_max);
  ROS_INFO_STREAM("x_min: " << x_min);
  double x_mid = (x_max - x_min) / 2 + x_min;
  ROS_INFO_STREAM("x_middle: " << x_mid);




  for (i = 0 ;i<size;i++) {
    if (temp_cloud->points[i].y > y_max)
      y_max = temp_cloud->points[i].y;
    if (temp_cloud->points[i].y < y_min)
      y_min = temp_cloud->points[i].y;
  }

  ROS_INFO_STREAM("y_max: " << y_max);
  ROS_INFO_STREAM("y_min: " << y_min);
  // we need to adjust for the simulated camera _optical_frame offset (+0.02m)
  double y_mid = (y_max - y_min) / 2 + y_min + 0.01;
  ROS_INFO_STREAM("y_middle: " << y_mid);




  for (i = 0;i<size;i++) {

//    ROS_INFO_STREAM("z_max: " << temp_cloud->points[i].z);
    if (temp_cloud->points[i].z > z_max)
      z_max = temp_cloud->points[i].z;
    if (temp_cloud->points[i].z < z_min)
      z_min = temp_cloud->points[i].z;

  }

  ROS_INFO_STREAM("z_max: " << z_max);
  ROS_INFO_STREAM("z_min: " << z_min);
  double z_mid = (z_max - z_min) / 2 + z_min;
  ROS_INFO_STREAM("z_middle: " << z_mid);


  toROSMsg(*temp_cloud, res.cloud_out);
  ROS_WARN("Finishing CalculateXYZ Service");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calc_xyz_service");
  ros::NodeHandle nh;

  ros::ServiceServer CalculateXYZService = nh.advertiseService("calculationXYZ", calculateXYZ);
  ROS_WARN("calculation_xyz_service Ready.");
  ros::spin();

  return 0;
}
