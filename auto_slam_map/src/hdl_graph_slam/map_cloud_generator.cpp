#include <hdl_graph_slam/map_cloud_generator.hpp>

#include <pcl/octree/octree_search.h>

namespace hdl_graph_slam {

MapCloudGenerator::MapCloudGenerator() {
}

MapCloudGenerator::~MapCloudGenerator() {

}

//11.27
pcl::PointCloud<MapCloudGenerator::PointT>::Ptr MapCloudGenerator::generate(const std::vector<KeyFrameSnapshot::Ptr>& keyframes,
                                                    const std::vector<KeyFrame::Ptr>& clipped_keyframes , double resolution, bool mode) const {
//pcl::PointCloud<MapCloudGenerator::PointT>::Ptr MapCloudGenerator::generate(const std::vector<KeyFrameSnapshot::Ptr>& keyframes, double resolution) const {
  if(keyframes.empty()) {
    std::cerr << "warning: keyframes empty!!" << std::endl;
    return nullptr;
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  cloud->reserve(keyframes.front()->cloud->size() * keyframes.size());

  pcl::PointCloud<PointT>::Ptr clipped_cloud(new pcl::PointCloud<PointT>());//11.27
  clipped_cloud->reserve(clipped_keyframes.front()->cloud->size() * clipped_keyframes.size());//11.27

  int num=0;//11.27
  for(const auto& keyframe : keyframes) {
    Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>();
    if(mode){//11.27
      for(const auto& src_pt : keyframe->cloud->points) {
        PointT dst_pt;
        dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
        dst_pt.intensity = src_pt.intensity;
        cloud->push_back(dst_pt);
      }
    }
    else{//11.27
      for(const auto& src_pt : clipped_keyframes[num++]->cloud->points) {
        PointT dst_pt;
        dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
        dst_pt.intensity = src_pt.intensity;
        clipped_cloud->push_back(dst_pt);
      }
    }
  }

  pcl::octree::OctreePointCloud<PointT> octree(resolution);//11.27
  if(mode){
    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = false;
  
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud(); 
  }
  else{//11.27
    clipped_cloud->width = clipped_cloud->size();
    clipped_cloud->height = 1;
    clipped_cloud->is_dense = false;

    octree.setInputCloud(clipped_cloud);
    octree.addPointsFromInputCloud();
  }
  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  octree.getOccupiedVoxelCenters(filtered->points);

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;

  return filtered;
}

}
