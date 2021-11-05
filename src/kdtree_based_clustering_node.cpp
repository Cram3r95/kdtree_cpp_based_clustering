#include "pcl_aux_functions.h"

ros::Publisher marker_pub;
//geometry_msgs::Pose vehicle_pose;
//bool first_callback = true;

int main(int argc, char **argv)
{
  // function required before using any other ROS based functions
  ros::init(argc, argv, "kdtree_based_clustering_node");

  // access point to ROS
  ros::NodeHandle nh;
  
  // Subscribers

  ros::Subscriber point_cloud = nh.subscribe("/t4ac/perception/sensors/lidar", 1, pclCallback);
  // ros::Subscriber point_cloud = nh.subscribe("/velodyne_points", 1, pclCallback);

  // Publishers
  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/t4ac/perception/detection/lidar/3D_lidar_obstacles_markers", 1);

  // function that calls each callback
  ros::spin();

  return 0;
}

// this function is called every time the pcl topic is updated
void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  //system("clear");

  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud = sensor2pcl(msg);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_point_cloud = filterCloud(point_cloud, 0.3, Eigen::Vector4f (-40,-5,-10,1), Eigen::Vector4f (50,17,10,1));

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = segmentPlane(filtered_point_cloud, 50, 0.3);

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = clustering(segmentCloud.first, 1.4, 20, 150, 10, 1, 6, 6, 4);

  std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> bBoxes = boundingBoxes(clusters);

  showBbRviz(bBoxes);

  std::cout << std::endl;
}

// convert sensor_msgs::PointCloud2 structure to a pcl::PointClouds structure
pcl::PointCloud<pcl::PointXYZI>::Ptr sensor2pcl(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  std::cout << "Creating the point cloud: ";
  auto startTime = std::chrono::steady_clock::now();
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // iteration over every point of the pcl structure use by ROS
  for (int i=0; i<(int)(msg->width); i++){
    pcl::PointXYZI point;
    // iteration over x, y, z and intensity of every point
    for (int j=0; j<4; j++){
      uint32_t foo_ = (uint32_t)((msg->data[i*16+j*4+3] << 24) | (msg->data[i*16+j*4+2] << 16) | (msg->data[i*16+j*4+1] << 8) | (msg->data[i*16+j*4] << 0));
      float foo;
      std::memcpy(&foo, &foo_, sizeof(float));
      // fill the point
      switch(j){
        case 0:
          point.x = foo;
          break;
        case 1:
          point.y = foo;
          break;
        case 2:
          point.z = foo;
          break;
        case 3:
          point.intensity = foo;
          break;
      }
    }
    point_cloud->push_back(point);
  }
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << elapsedTime.count() << " milliseconds, " << point_cloud->points.size() << " points" << std::endl;
  return point_cloud;
}

// filter the point cloud based on the region of interest and the voxel size
pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
  std::cout << "Filtering point cloud: ";
  auto startTime = std::chrono::steady_clock::now();

  // filter based on voxels
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZI>);
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloudFiltered);

  // filter based on region of interest
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRegion (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::CropBox<pcl::PointXYZI> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloudFiltered);
  region.filter(*cloudRegion);
/*
  std::vector<int> indices;
  pcl::CropBox<pcl::PointXYZI> roof(true);
  roof.setMin(Eigen::Vector4f(-2.0, -2.0, -3.0, 1.0));
  roof.setMax(Eigen::Vector4f(3.0, 2.0, 3.0, 1.0));
  roof.setInputCloud(cloudRegion);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers {new pcl::PointIndices()};
  for(int point : indices){
    inliers->indices.push_back(point);
  }

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloudRegion);
*/
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << elapsedTime.count() << " milliseconds, " << cloudRegion->points.size() << " points" << std::endl;

  return cloudRegion;
}

// apply ransac to segment into floor and 
std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceThreshold)
{
  std::cout << "Applying ransac-3D: ";
  auto startTime = std::chrono::steady_clock::now();

  std::unordered_set<int> inliersResult;
	srand(time(NULL));
    
  // iterate the number of iteration
	for(int i=0; i<maxIterations; i++){
    std::unordered_set<int> inliers;

    // get 3 random point from the cloud and create the 3D-plane
    while(inliers.size() < 3){
		  inliers.insert(rand()%(cloud->points.size()));
    }

    float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

    float a, b, c, d;
		a = ((y2 - y1)*(z3 - z1)) - ((z2 - z1)*(y3 - y1));
		b = ((z2 - z1)*(x3 - x1)) - ((x2 - x1)*(z3 - z1));
		c = ((x2 - x1)*(y3 - y1)) - ((y2 - y1)*(x3 - x1));
		d = -(a*x1 + b*y1 + c*z1);

    // get the points that lie on the plane within the distance threshold
		for(int index = 0; index < (int) cloud->points.size(); index++){
			if(inliers.count(index) > 0) continue;
            
			pcl::PointXYZI point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			float distance = fabs(a*x4 + b*y4 + c*z4 + d)/sqrt(a*a + b*b + c*c);

			if(distance <= distanceThreshold){
				inliers.insert(index);
			}
		}

    // store the results of the plane on which lies the most points
		if(inliers.size()>inliersResult.size()){
			inliersResult = inliers;
		}
	}

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

  // divide the floor and the rest in 2 different point clouds
  for (int index = 0; index < (int) cloud->points.size(); index++){
    pcl::PointXYZI point = cloud->points[index];
    if(inliersResult.count(index)){
      cloudInliers->points.push_back(point);
    }else{
      cloudOutliers->points.push_back(point);
    }
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << elapsedTime.count() << " milliseconds, " << cloudInliers->points.size() << " points" << std::endl;

  return std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> (cloudOutliers,cloudInliers);
}

// cluster the point cloud according to the kdtree algorithm
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clustering(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clusterTolerance, int minSize, int maxSize, float maxVolume, float minVolume, float maxWidth, float maxLength, float maxHeight)
{
  std::cout << "Clustering the point cloud: ";
  auto startTime = std::chrono::steady_clock::now();

  // create the vector where the clusters will be stored
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

  // insert each point in the kdtree
  std::vector<std::vector<float>> vpoints;
  KdTree* tree = new KdTree;
  for(int i=0; i<(int)cloud->points.size(); i++){
    pcl::PointXYZI point = cloud->points[i];
    vpoints.push_back(std::vector<float> {point.x, point.y, point.z});
    tree->insert(std::vector<float> {point.x, point.y, point.z}, i);
  }

  // add to the vector of clusters the point cloud of nearby points that store a number of points between the maximum and the minumum selected,
  // are within the maximum length, width and height, and have a volume smaller than the selected maximum.
  std::vector<std::vector<int>> intclusters;
	std::vector<bool> processed(vpoints.size(), false);
	for (int i=0; i<(int)vpoints.size(); i++){
		if(processed[i]==false){
			std::vector<int> vcluster;
			proximity(vpoints, i, vcluster, processed, tree, clusterTolerance);
      
      if((int)vcluster.size() >= minSize && (int)vcluster.size() <= maxSize){
			  intclusters.push_back(vcluster);
      }
		}
	}

  for (std::vector<int> clusterIndex : intclusters){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster (new pcl::PointCloud<pcl::PointXYZI>);
    for (int index : clusterIndex){
      cloudCluster->points.push_back(cloud->points[index]);
    }
    cloudCluster->width = cloudCluster->points.size();
    cloudCluster->height = 1;
    cloudCluster->is_dense = true;

    std::pair<pcl::PointXYZ, pcl::PointXYZ> bb = boundingBox(cloudCluster);
    float width = abs(bb.first.x - bb.second.x);
    float length = abs(bb.first.y - bb.second.y);
    float height = abs(bb.first.z - bb.second.z);
    float volume = width*length*height;
    if(width < maxWidth && length < maxLength && height < maxHeight && volume < maxVolume && volume > minVolume){
      clusters.push_back(cloudCluster);
    }
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << elapsedTime.count() << " milliseconds, " << clusters.size() << " clusters" << std::endl;

  return clusters;
}

// add points to a cluster if it is within distance tolerance
void proximity(std::vector<std::vector<float>> points, int id_point, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[id_point] = true;
	cluster.push_back(id_point);
	for (int id_near_point : tree->search(points[id_point], distanceTol)){
		if(processed[id_near_point] == false){
			proximity(points, id_near_point, cluster, processed, tree, distanceTol);
		}
	}
}

// create all the bounding boxes of the clusters
std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> boundingBoxes(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters)
{
  std::cout << "Creating bounding boxes: ";
  auto startTime = std::chrono::steady_clock::now();

  std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> boundingBoxes;

  for(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>::iterator cluster = clusters.begin(); cluster != clusters.end(); ++cluster) {
    boundingBoxes.push_back(boundingBox(*cluster));
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << elapsedTime.count() << " milliseconds" << std::endl;

  return boundingBoxes;
}

// create maximum and the minimum points of the cluster bounding box
std::pair<pcl::PointXYZ, pcl::PointXYZ> boundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster)
{
  std::pair<pcl::PointXYZ, pcl::PointXYZ> box;
  pcl::PointXYZ minPoint, maxPoint;
  
  std::vector<float> xs,ys,zs;
  for(int i = 0; i < (int)cluster->points.size(); i++){
    xs.push_back(cluster->points[i].x);
    ys.push_back(cluster->points[i].y);
    zs.push_back(cluster->points[i].z);
  }

  minPoint.x = *min_element(xs.begin(), xs.end());
  minPoint.y = *min_element(ys.begin(), ys.end());
  minPoint.z = *min_element(zs.begin(), zs.end());

  maxPoint.x = *max_element(xs.begin(), xs.end());
  maxPoint.y = *max_element(ys.begin(), ys.end());
  maxPoint.z = *max_element(zs.begin(), zs.end());

  box = std::make_pair(minPoint, maxPoint);
  return box;
}

// publish and create the MarkerArray that contains the boxes
void showBbRviz(std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> boundingBoxes)
{
  // delete all previous cubes
  visualization_msgs::MarkerArray marker_bbs;
  visualization_msgs::Marker marker_bb;
  marker_bb.type = visualization_msgs::Marker::CUBE;
  marker_bb.action = visualization_msgs::Marker::DELETEALL;
  marker_bbs.markers.push_back(marker_bb);
  marker_pub.publish(marker_bbs);

  marker_bbs = visualization_msgs::MarkerArray();

  int i = 0;
  for(std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>::iterator boundingBox = boundingBoxes.begin(); boundingBox != boundingBoxes.end(); ++boundingBox) {
    visualization_msgs::Marker marker_bb;

    marker_bb.header.frame_id = "ego_vehicle/lidar/lidar1";
    // marker_bb.header.frame_id = "lidar_vlp16";
    marker_bb.header.stamp = ros::Time::now();
    marker_bb.id = i;

    marker_bb.type = visualization_msgs::Marker::CUBE;
    marker_bb.action = visualization_msgs::Marker::ADD;

    marker_bb.pose.position.x = ((*boundingBox).first.x + (*boundingBox).second.x) / 2;
    marker_bb.pose.position.y = ((*boundingBox).first.y + (*boundingBox).second.y) / 2;
    marker_bb.pose.position.z = ((*boundingBox).first.z + (*boundingBox).second.z) / 2;

    marker_bb.pose.orientation.x = 0.0;
    marker_bb.pose.orientation.y = 0.0;
    marker_bb.pose.orientation.z = 0.0;
    marker_bb.pose.orientation.w = 0.0;

    marker_bb.scale.x = abs((*boundingBox).first.x - (*boundingBox).second.x);
    marker_bb.scale.y = abs((*boundingBox).first.y - (*boundingBox).second.y);
    marker_bb.scale.z = abs((*boundingBox).first.z - (*boundingBox).second.z);

    marker_bb.color.r = 1.0;
    marker_bb.color.g = 1.0;
    marker_bb.color.b = 1.0;
    marker_bb.color.a = 0.8;

    marker_bb.lifetime = ros::Duration();

    marker_bbs.markers.push_back(marker_bb);

    i++;
  }

  marker_pub.publish(marker_bbs);
}