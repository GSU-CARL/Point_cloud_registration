#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
constexpr const char * kDefaultOutputDir = MY_POINT_REG_SOURCE_DIR "/output_test_file";
}  // namespace

// "load + preprocess" stage of the point-cloud merge pipeline
// (see POINTCLOUD_MERGE_PLAN.md). File in, file out: this node does nothing
// on startup, declares its parameters, and does all its work inside the
// `voxelize` Trigger callback so a human decides when it runs.
class VoxelNode : public rclcpp::Node
{
public:
  VoxelNode()
  : Node("voxel_node")
  {
    this->declare_parameter<std::string>("input_pcd_path", "");
    this->declare_parameter<std::string>("output_pcd_path", "");
    this->declare_parameter<double>("leaf_size", 0.5);
    this->declare_parameter<bool>("remove_outliers", false);
    this->declare_parameter<int>("sor_mean_k", 50);
    this->declare_parameter<double>("sor_stddev_mul_thresh", 1.0);

    service_ = this->create_service<std_srvs::srv::Trigger>(
      "voxelize",
      std::bind(
        &VoxelNode::handleVoxelize, this, std::placeholders::_1,
        std::placeholders::_2));
  }

private:
  void handleVoxelize(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const std::string input_pcd_path =
      this->get_parameter("input_pcd_path").as_string();
    const std::string output_pcd_path =
      this->get_parameter("output_pcd_path").as_string();
    const double leaf_size = this->get_parameter("leaf_size").as_double();
    const bool remove_outliers =
      this->get_parameter("remove_outliers").as_bool();
    const int sor_mean_k = this->get_parameter("sor_mean_k").as_int();
    const double sor_stddev_mul_thresh =
      this->get_parameter("sor_stddev_mul_thresh").as_double();

    if (input_pcd_path.empty()) {
      response->success = false;
      response->message = "'input_pcd_path' parameter must be set.";
      return;
    }

    std::string resolved_output_pcd_path = output_pcd_path;
    if (resolved_output_pcd_path.empty()) {
      const std::filesystem::path input_path(input_pcd_path);
      const std::filesystem::path output_dir(kDefaultOutputDir);
      std::filesystem::create_directories(output_dir);
      resolved_output_pcd_path =
        (output_dir / (input_path.stem().string() + "_voxelized.pcd")).string();
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_pcd_path, *cloud) == -1) {
      response->success = false;
      response->message = "Failed to load input PCD file: " + input_pcd_path;
      return;
    }

    if (cloud->empty()) {
      response->success = false;
      response->message = "Input PCD file is empty: " + input_pcd_path;
      return;
    }
    const std::size_t loaded_count = cloud->size();

    pcl::PointCloud<pcl::PointXYZI>::Ptr voxelized(
      new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*voxelized);
    const std::size_t voxelized_count = voxelized->size();

    pcl::PointCloud<pcl::PointXYZI>::Ptr result = voxelized;
    std::size_t outlier_removed_count = 0;
    if (remove_outliers) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
        new pcl::PointCloud<pcl::PointXYZI>);
      pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
      sor.setInputCloud(voxelized);
      sor.setMeanK(sor_mean_k);
      sor.setStddevMulThresh(sor_stddev_mul_thresh);
      sor.filter(*filtered);
      result = filtered;
      outlier_removed_count = filtered->size();
    }

    if (pcl::io::savePCDFileBinary(resolved_output_pcd_path, *result) != 0) {
      response->success = false;
      response->message =
        "Failed to save output PCD file: " + resolved_output_pcd_path;
      return;
    }

    std::ostringstream msg;
    msg << std::fixed << std::setprecision(6);
    msg << "Loaded " << loaded_count << " points -> voxelized to "
        << voxelized_count << " (leaf_size=" << leaf_size << ")";
    if (remove_outliers) {
      msg << " -> outlier removal (mean_k=" << sor_mean_k
          << ", stddev_mul=" << sor_stddev_mul_thresh << "): "
          << voxelized_count << " -> " << outlier_removed_count;
    }
    msg << " -> " << resolved_output_pcd_path;

    response->success = true;
    response->message = msg.str();
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelNode>());
  rclcpp::shutdown();
  return 0;
}
