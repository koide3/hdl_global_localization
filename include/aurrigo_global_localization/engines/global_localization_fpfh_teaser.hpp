/**
 * @brief Global pose estimation with FPFH features and Teaser++
 * @ref Yang et al., "TEASER: Fast and Certifiable Point Cloud Registration", T-RO, 2020
 * @ref https://github.com/MIT-SPARK/TEASER-plusplus
 */
#ifndef AURRIGO_GLOBAL_LOCALIZATION_FPFH_TEASER_HPP
#define AURRIGO_GLOBAL_LOCALIZATION_FPFH_TEASER_HPP

#include <aurrigo_global_localization/ransac/matching_cost_evaluater.hpp>
#include <aurrigo_global_localization/engines/global_localization_fpfh_ransac.hpp>

namespace aurrigo_global_localization {

class GlobalLocalizationEngineFPFH_Teaser : public GlobalLocalizationEngineFPFH_RANSAC {
public:
  GlobalLocalizationEngineFPFH_Teaser(ros::NodeHandle& private_nh);
  virtual ~GlobalLocalizationEngineFPFH_Teaser() override;

  virtual void set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) override;
  virtual GlobalLocalizationResults query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) override;

private:
  using GlobalLocalizationEngineFPFH_RANSAC::private_nh;

  std::unique_ptr<MatchingCostEvaluater> evaluater;
};

}  // namespace aurrigo_global_localization

#endif