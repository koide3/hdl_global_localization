#include <hdl_global_localization/engines/global_localization_bbs.hpp>

#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <hdl_global_localization/util/config.hpp>
#include <hdl_global_localization/bbs/bbs_localization.hpp>
#include <hdl_global_localization/bbs/occupancy_gridmap.hpp>

namespace hdl_global_localization {

GlobalLocalizationBBSParams::GlobalLocalizationBBSParams() {
  const Config config(GlobalConfig::get_config_path("config_bbs"));

  bbs_params.max_range = config.param<double>("bbs", "max_range", 15.0);
  bbs_params.min_tx = config.param<double>("bbs", "min_tx", -50.0);
  bbs_params.max_tx = config.param<double>("bbs", "max_tx", 50.0);
  bbs_params.min_ty = config.param<double>("bbs", "min_ty", -50.0);
  bbs_params.max_ty = config.param<double>("bbs", "max_ty", 50.0);
  bbs_params.min_theta = config.param<double>("bbs", "min_theta", -3.15);
  bbs_params.max_theta = config.param<double>("bbs", "max_theta", 3.15);

  map_min_z = config.param<double>("bbs", "map_min_z", 2.0);
  map_max_z = config.param<double>("bbs", "map_max_z", 2.4);

  map_width = config.param<int>("bbs", "map_width", 512);
  map_height = config.param<int>("bbs", "map_height", 1024);
  map_resolution = config.param<double>("bbs", "map_resolution", 0.5);

  map_pyramid_level = config.param<int>("bbs", "map_pyramid_level", 6);
  max_points_per_cell = config.param<int>("bbs", "max_points_per_cell", 5);

  scan_min_z = config.param<double>("bbs", "scan_min_z", -0.2);
  scan_max_z = config.param<double>("bbs", "scan_max_z", -0.2);
}

GlobalLocalizationBBSParams::~GlobalLocalizationBBSParams() {}

GlobalLocalizationBBS::GlobalLocalizationBBS(rclcpp::Node& node, const GlobalLocalizationBBSParams& params) : params(params) {
  auto gridmap_qos = rclcpp::SystemDefaultsQoS();
  gridmap_qos.get_rmw_qos_profile().depth = 1;
  gridmap_qos.get_rmw_qos_profile().reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  gridmap_qos.get_rmw_qos_profile().durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  gridmap_pub = node.create_publisher<nav_msgs::msg::OccupancyGrid>("bbs/gridmap", gridmap_qos);

  map_slice_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("bbs/map_slice", rclcpp::SensorDataQoS());
  scan_slice_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("bbs/scan_slice", rclcpp::SensorDataQoS());
}

GlobalLocalizationBBS ::~GlobalLocalizationBBS() {}

void GlobalLocalizationBBS::set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  // params.max_range = private_nh.param<double>("bbs/max_range", 15.0);
  // params.min_tx = private_nh.param<double>("bbs/min_tx", -10.0);
  // params.max_tx = private_nh.param<double>("bbs/max_tx", 10.0);
  // params.min_ty = private_nh.param<double>("bbs/min_ty", -10.0);
  // params.max_ty = private_nh.param<double>("bbs/max_ty", 10.0);
  // params.min_theta = private_nh.param<double>("bbs/min_theta", -3.15);
  // params.max_theta = private_nh.param<double>("bbs/max_theta", 3.15);
  bbs.reset(new BBSLocalization(params.bbs_params));

  double map_min_z = params.map_min_z;
  double map_max_z = params.map_max_z;
  auto map_2d = slice(*cloud, map_min_z, map_max_z);
  spdlog::info("Set Map {} points", map_2d.size());

  if (map_2d.size() < 128) {
    spdlog::warn("Num points in the sliced map is too small!!");
    spdlog::warn("Change the slice range parameters!!");
  }

  int map_width = params.map_width;
  int map_height = params.map_height;
  double map_resolution = params.map_resolution;
  int map_pyramid_level = params.map_pyramid_level;
  int max_points_per_cell = params.max_points_per_cell;
  bbs->set_map(map_2d, map_resolution, map_width, map_height, map_pyramid_level, max_points_per_cell);

  auto map_3d = unslice(map_2d);
  map_3d->header.frame_id = "map";

  auto map_slice_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*map_3d, *map_slice_msg);

  map_slice_pub->publish(*map_slice_msg);
  gridmap_pub->publish(*bbs->gridmap()->to_rosmsg());
}

GlobalLocalizationResults GlobalLocalizationBBS::query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) {
  double scan_min_z = params.scan_min_z;
  double scan_max_z = params.scan_max_z;
  auto scan_2d = slice(*cloud, scan_min_z, scan_max_z);

  std::vector<GlobalLocalizationResult::Ptr> results;

  spdlog::info("Query {} points", scan_2d.size());
  if (scan_2d.size() < 32) {
    spdlog::warn("Num points in the sliced scan is too small!!");
    spdlog::warn("Change the slice range parameters!!");
    return GlobalLocalizationResults(results);
  }

  double best_score = 0.0;
  auto trans_2d = bbs->localize(scan_2d, 0.0, &best_score);
  if (trans_2d == boost::none) {
    return GlobalLocalizationResults(results);
  }

  if (scan_slice_pub->get_subscription_count()) {
    auto scan_3d = unslice(scan_2d);
    scan_3d->header = cloud->header;

    auto scan_slice_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*scan_3d, *scan_slice_msg);
    scan_slice_pub->publish(*scan_slice_msg);
  }

  Eigen::Isometry3f trans_3d = Eigen::Isometry3f::Identity();
  trans_3d.linear().block<2, 2>(0, 0) = trans_2d->linear();
  trans_3d.translation().head<2>() = trans_2d->translation();

  results.resize(1);
  results[0].reset(new GlobalLocalizationResult(best_score, best_score, trans_3d));

  return GlobalLocalizationResults(results);
}

GlobalLocalizationBBS::Points2D GlobalLocalizationBBS::slice(const pcl::PointCloud<pcl::PointXYZ>& cloud, double min_z, double max_z) const {
  Points2D points_2d;
  points_2d.reserve(cloud.size());
  for (int i = 0; i < cloud.size(); i++) {
    if (min_z < cloud.at(i).z && cloud.at(i).z < max_z) {
      points_2d.push_back(cloud.at(i).getVector3fMap().head<2>());
    }
  }
  return points_2d;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GlobalLocalizationBBS::unslice(const Points2D& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->resize(points.size());
  for (int i = 0; i < points.size(); i++) {
    cloud->at(i).getVector3fMap().head<2>() = points[i];
    cloud->at(i).z = 0.0f;
  }

  return cloud;
}
}  // namespace hdl_global_localization