#ifndef HDL_GLOBAL_LOCALIZATION_RANSAC_POSE_ESTIMATION_HPP
#define HDL_GLOBAL_LOCALIZATION_RANSAC_POSE_ESTIMATION_HPP

#include <atomic>
#include <vector>
#include <random>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include <hdl_global_localization/global_localization_results.hpp>
#include <hdl_global_localization/ransac/voxelset.hpp>
#include <hdl_global_localization/ransac/matching_cost_evaluater.hpp>

namespace hdl_global_localization {

template <typename FeatureT>
class RansacPoseEstimation {
public:
  RansacPoseEstimation(ros::NodeHandle& private_nh);

  void set_target(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target, typename pcl::PointCloud<FeatureT>::ConstPtr target_features);
  void set_source(pcl::PointCloud<pcl::PointXYZ>::ConstPtr source, typename pcl::PointCloud<FeatureT>::ConstPtr source_features);

  GlobalLocalizationResults estimate();

private:
  void select_samples(std::mt19937& mt, const std::vector<std::vector<int>>& similar_features, std::vector<int>& samples, std::vector<int>& correspondences) const;

private:
  ros::NodeHandle& private_nh;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr target;
  typename pcl::PointCloud<FeatureT>::ConstPtr target_features;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr source;
  typename pcl::PointCloud<FeatureT>::ConstPtr source_features;

  typename pcl::KdTreeFLANN<FeatureT>::Ptr feature_tree;
  std::unique_ptr<MatchingCostEvaluater> evaluater;
};

}  // namespace hdl_global_localization

#endif