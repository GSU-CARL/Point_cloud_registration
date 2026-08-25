#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <pcl/common/io.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
constexpr const char * kDefaultOutputDir = "output_test_file";
}  // namespace

// "estimate normals + FPFH features" stage of the point-cloud merge pipeline
// (see POINTCLOUD_MERGE_PLAN.md, Step 2). File in, file out: this node does
// nothing on startup, declares its parameters, and does all its work inside
// the `estimate_features` Trigger callback so a human decides when it runs.
class FeatureNode : public rclcpp::Node
{
public:
  FeatureNode()
  : Node("feature_node")
  {
    this->declare_parameter<std::string>("input_pcd_path", "");
    this->declare_parameter<std::string>("output_normals_pcd_path", "");
    this->declare_parameter<std::string>("output_fpfh_pcd_path", "");
    this->declare_parameter<double>("normal_radius", 0.5);
    this->declare_parameter<double>("fpfh_radius", 1.0);
    this->declare_parameter<int>("num_threads", 4);
    // PCL orients each normal toward a viewpoint that defaults to the origin.
    // For world-frame scans that origin sits *in* the ground plane, so for
    // ground points the flip direction is decided by numerical noise - roughly
    // half of them come out pointing down. FPFH is built from angles between
    // normals, so that turns half the descriptors into the sign-flipped
    // version of the other half, and the source/target clouds flip different
    // points. Registration then matches noise. Flipping to a single global
    // direction instead makes the descriptors comparable across clouds.
    this->declare_parameter<bool>("orient_normals_up", true);

    service_ = this->create_service<std_srvs::srv::Trigger>(
      "estimate_features",
      std::bind(
        &FeatureNode::handleEstimateFeatures, this, std::placeholders::_1,
        std::placeholders::_2));
  }

private:
  void handleEstimateFeatures(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const std::string input_pcd_path =
      this->get_parameter("input_pcd_path").as_string();
    const std::string output_normals_pcd_path =
      this->get_parameter("output_normals_pcd_path").as_string();
    const std::string output_fpfh_pcd_path =
      this->get_parameter("output_fpfh_pcd_path").as_string();
    const double normal_radius =
      this->get_parameter("normal_radius").as_double();
    const double fpfh_radius =
      this->get_parameter("fpfh_radius").as_double();
    const int num_threads = this->get_parameter("num_threads").as_int();
    const bool orient_normals_up =
      this->get_parameter("orient_normals_up").as_bool();

    if (input_pcd_path.empty()) {
      response->success = false;
      response->message = "'input_pcd_path' parameter must be set.";
      return;
    }

    std::string resolved_output_normals_pcd_path = output_normals_pcd_path;
    std::string resolved_output_fpfh_pcd_path = output_fpfh_pcd_path;
    if (resolved_output_normals_pcd_path.empty() ||
      resolved_output_fpfh_pcd_path.empty())
    {
      const std::filesystem::path input_path(input_pcd_path);
      const std::filesystem::path output_dir(kDefaultOutputDir);
      std::filesystem::create_directories(output_dir);
      if (resolved_output_normals_pcd_path.empty()) {
        resolved_output_normals_pcd_path =
          (output_dir / (input_path.stem().string() + "_normals.pcd")).string();
      }
      if (resolved_output_fpfh_pcd_path.empty()) {
        resolved_output_fpfh_pcd_path =
          (output_dir / (input_path.stem().string() + "_fpfh.pcd")).string();
      }
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

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>);

    pcl::PointCloud<pcl::Normal>::Ptr normals(
      new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(normal_radius);
    normal_estimation.setNumberOfThreads(num_threads);
    normal_estimation.compute(*normals);

    // Re-orient to +Z (see `orient_normals_up` above). Normals estimated by
    // PCA are sign-ambiguous, so flipping the whole cloud to one hemisphere
    // costs nothing geometrically and makes FPFH comparable between the source
    // and target clouds. Non-finite normals (radius search found too few
    // neighbours) are left alone - FPFH already ignores them.
    std::size_t flipped_count = 0;
    if (orient_normals_up) {
      for (auto & normal : normals->points) {
        if (!std::isfinite(normal.normal_z)) {
          continue;
        }
        if (normal.normal_z < 0.0f) {
          normal.normal_x = -normal.normal_x;
          normal.normal_y = -normal.normal_y;
          normal.normal_z = -normal.normal_z;
          ++flipped_count;
        }
      }
    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(
      new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    if (pcl::io::savePCDFileBinary(
        resolved_output_normals_pcd_path, *cloud_with_normals) != 0)
    {
      response->success = false;
      response->message =
        "Failed to save output normals PCD file: " +
        resolved_output_normals_pcd_path;
      return;
    }

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_features(
      new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::FPFHEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33>
    fpfh_estimation;
    fpfh_estimation.setInputCloud(cloud);
    fpfh_estimation.setInputNormals(normals);
    fpfh_estimation.setSearchMethod(tree);
    fpfh_estimation.setRadiusSearch(fpfh_radius);
    fpfh_estimation.setNumberOfThreads(num_threads);
    fpfh_estimation.compute(*fpfh_features);

    if (pcl::io::savePCDFileBinary(
        resolved_output_fpfh_pcd_path, *fpfh_features) != 0)
    {
      response->success = false;
      response->message =
        "Failed to save output FPFH PCD file: " + resolved_output_fpfh_pcd_path;
      return;
    }

    std::ostringstream msg;
    msg << std::fixed << std::setprecision(6);
    msg << "Loaded " << loaded_count << " points -> normals (radius="
        << normal_radius << ", orient_normals_up="
        << (orient_normals_up ? "true" : "false");
    if (orient_normals_up) {
      msg << ", flipped " << flipped_count << " to +Z";
    }
    msg << ") -> " << resolved_output_normals_pcd_path
        << " -> FPFH (radius=" << fpfh_radius << ") -> "
        << resolved_output_fpfh_pcd_path;

    response->success = true;
    response->message = msg.str();
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeatureNode>());
  rclcpp::shutdown();
  return 0;
}
