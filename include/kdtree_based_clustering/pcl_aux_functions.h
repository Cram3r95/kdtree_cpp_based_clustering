#ifndef PCL_AUX_FUNCTIONS_H
#define PCL_AUX_FUNCTIONS_H

#include <ros/ros.h>
#include <cstdlib>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <string>
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <chrono>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "kdtree.h"

int main(int argc, char **argv);

void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

pcl::PointCloud<pcl::PointXYZI>::Ptr sensor2pcl(const sensor_msgs::PointCloud2::ConstPtr& msg);

pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceThreshold);

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clustering(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clusterTolerance, int minSize, int maxSize, float maxVolume, float minVolume, float maxWidth, float maxLength, float maxHeight);

void proximity(std::vector<std::vector<float>> points, int id_point, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol);

std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> boundingBoxes(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters);

std::pair<pcl::PointXYZ, pcl::PointXYZ> boundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster);

void showBbRviz(std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> boundingBoxes);

Eigen::Quaterniond QuaternionRot(Eigen::Vector3d x1, Eigen::Vector3d y1, Eigen::Vector3d z1, Eigen::Vector3d x2, Eigen::Vector3d y2, Eigen::Vector3d z2);

#endif