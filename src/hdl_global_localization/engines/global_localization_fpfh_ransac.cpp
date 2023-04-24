#include <hdl_global_localization/engines/global_localization_fpfh_ransac.hpp>

#include <spdlog/spdlog.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/impl/kdtree.hpp>

#include <hdl_global_localization/ransac/ransac_pose_estimation.hpp>

namespace hdl_global_localization {

GlobalLocalizationEngineFPFH_RANSAC::GlobalLocalizationEngineFPFH_RANSAC(const GlobalLocalizationEngineFPFH_RANSACParams& params) : params(params) {}

GlobalLocalizationEngineFPFH_RANSAC::~GlobalLocalizationEngineFPFH_RANSAC() {}

pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr GlobalLocalizationEngineFPFH_RANSAC::extract_fpfh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  double normal_estimation_radius = params.normal_estimation_radius;
  double search_radius = params.search_radius;

  spdlog::info("Normal Estimation: Radius({})", normal_estimation_radius);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;
  nest.setRadiusSearch(normal_estimation_radius);
  nest.setInputCloud(cloud);
  nest.compute(*normals);

  spdlog::info("FPFH Extraction: Search Radius({})", search_radius);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
  fest.setRadiusSearch(search_radius);
  fest.setInputCloud(cloud);
  fest.setInputNormals(normals);
  fest.compute(*features);

  return features;
}

void GlobalLocalizationEngineFPFH_RANSAC::set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  global_map = cloud;
  global_map_features = extract_fpfh(cloud);

  ransac.reset(new RansacPoseEstimation<pcl::FPFHSignature33>(params.ransac_params));
  ransac->set_target(global_map, global_map_features);
}

GlobalLocalizationResults GlobalLocalizationEngineFPFH_RANSAC::query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) {
  pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr cloud_features = extract_fpfh(cloud);

  ransac->set_source(cloud, cloud_features);
  auto results = ransac->estimate();

  return results.sort(max_num_candidates);
}

}  // namespace hdl_global_localization