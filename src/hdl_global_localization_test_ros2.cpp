#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/auto_io.h>

#include <rclcpp/rclcpp.hpp>
#include <hdl_global_localization/srv/set_global_map.hpp>
#include <hdl_global_localization/srv/set_global_localization_engine.hpp>
#include <hdl_global_localization/srv/query_global_localization.hpp>

namespace hdl_global_localization {

class TestNode : public rclcpp::Node {
public:
  TestNode(rclcpp::NodeOptions& options) : rclcpp::Node("hdl_global_localization_test", options) {
    const std::string global_map_path = "/home/koide/datasets/hdl_localization/hdl_400_map.pcd";

    global_map = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::io::load(global_map_path, *global_map);
    RCLCPP_INFO_STREAM(this->get_logger(), "Global map=" << global_map->size() << " points");
  }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;
};

}  // namespace hdl_global_localization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<hdl_global_localization::TestNode>(options);
  rclcpp::spin(node);

  return 0;
}