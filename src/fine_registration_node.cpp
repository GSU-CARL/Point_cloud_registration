#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/search/kdtree.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
constexpr const char * kDefaultOutputDir = "output_test_file";

// Reads a 4x4 transform written in the same plain-text, space-separated,
// row-major layout coarse_registration_node writes (see
// POINTCLOUD_MERGE_PLAN.md's Open3D cross-check note). Returns false (with
// `error_message` set) on any missing/short/malformed file.
bool readTransform(
  const std::string & path, Eigen::Matrix4f & transform,
  std::string & error_message)
{
  std::ifstream transform_file(path);
  if (!transform_file.is_open()) {
    error_message = "Failed to open initial guess transform file: " + path;
    return false;
  }
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      if (!(transform_file >> transform(row, col))) {
        error_message =
          "Initial guess transform file does not contain a 4x4 matrix: " +
          path;
        return false;
      }
    }
  }
  // operator>> parses "nan" and "inf" without complaint, and a degenerate
  // SAC-IA sample can produce either. Seeding GICP with one yields a garbage
  // transform that still reports a plausible fitness score, so reject it here.
  if (!transform.allFinite()) {
    error_message =
      "Initial guess transform file contains non-finite values: " + path;
    return false;
  }
  if (!transform.row(3).transpose().isApprox(
      Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f), 1e-4f))
  {
    error_message =
      "Initial guess transform file is not an affine 4x4 transform (bottom "
      "row is not [0 0 0 1]): " + path;
    return false;
  }
  return true;
}
}  // namespace

// "fine registration" stage of the point-cloud merge pipeline (see
// POINTCLOUD_MERGE_PLAN.md, Step 4). File in, file out: this node does
// nothing on startup, declares its parameters, and does all its work inside
// the `register_fine` Trigger callback so a human decides when it runs.
//
// Consumes the plain source/target clouds (same pcl::PointXYZI convention as
// voxel_node) plus an optional initial-guess transform - either
// coarse_registration_node's output_transform_path, or a MAVROS
// odometry-derived relative pose once that is available (see the plan's
// scope note) - and runs GICP to produce the fine-aligned transform.
class FineRegistrationNode : public rclcpp::Node
{
public:
  FineRegistrationNode()
  : Node("fine_registration_node")
  {
    this->declare_parameter<std::string>("source_pcd_path", "");
    this->declare_parameter<std::string>("target_pcd_path", "");
    this->declare_parameter<std::string>("initial_guess_transform_path", "");
    this->declare_parameter<std::string>("output_aligned_pcd_path", "");
    this->declare_parameter<std::string>("output_transform_path", "");
    // One GICP pass per entry, in order, each seeded with the previous pass's
    // result. A single loose gate lets a cloud whose points are mostly one big
    // ground plane free-slide in XY/yaw until it maximises ground-on-ground
    // overlap - it reports a *better* fitness score while being visibly more
    // wrong. Starting loose enough to catch the real offset and then tightening
    // pins the solution on structure instead. Pass a single-element array for
    // the old single-pass behaviour.
    this->declare_parameter<std::vector<double>>(
      "max_correspondence_distances", {0.5, 0.3, 0.2});
    this->declare_parameter<double>("transformation_epsilon", 1e-8);
    this->declare_parameter<double>("euclidean_fitness_epsilon", 1e-6);
    this->declare_parameter<double>("rotation_epsilon", 2e-3);
    this->declare_parameter<int>("max_iterations", 50);
    this->declare_parameter<int>("correspondence_randomness", 20);
    // Quality gate, 0 disables it. A meaningful threshold depends on the
    // scene's scale and the leaf size the clouds were voxelized at, so there
    // is no useful non-zero default - run once, read the fitness_score out of
    // this node's response, then set this just above a known-good run.
    this->declare_parameter<double>("max_fitness_score", 0.0);

    service_ = this->create_service<std_srvs::srv::Trigger>(
      "register_fine",
      std::bind(
        &FineRegistrationNode::handleRegisterFine, this,
        std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handleRegisterFine(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const std::string source_pcd_path =
      this->get_parameter("source_pcd_path").as_string();
    const std::string target_pcd_path =
      this->get_parameter("target_pcd_path").as_string();
    const std::string initial_guess_transform_path =
      this->get_parameter("initial_guess_transform_path").as_string();
    const std::string output_aligned_pcd_path =
      this->get_parameter("output_aligned_pcd_path").as_string();
    const std::string output_transform_path =
      this->get_parameter("output_transform_path").as_string();
    const std::vector<double> max_correspondence_distances =
      this->get_parameter("max_correspondence_distances").as_double_array();
    const double transformation_epsilon =
      this->get_parameter("transformation_epsilon").as_double();
    const double euclidean_fitness_epsilon =
      this->get_parameter("euclidean_fitness_epsilon").as_double();
    const double rotation_epsilon =
      this->get_parameter("rotation_epsilon").as_double();
    const int max_iterations = this->get_parameter("max_iterations").as_int();
    const int correspondence_randomness =
      this->get_parameter("correspondence_randomness").as_int();
    const double max_fitness_score =
      this->get_parameter("max_fitness_score").as_double();

    if (source_pcd_path.empty() || target_pcd_path.empty()) {
      response->success = false;
      response->message =
        "'source_pcd_path' and 'target_pcd_path' parameters must both be "
        "set.";
      return;
    }

    if (max_correspondence_distances.empty()) {
      response->success = false;
      response->message =
        "'max_correspondence_distances' must contain at least one value.";
      return;
    }
    for (const double distance : max_correspondence_distances) {
      if (distance <= 0.0) {
        std::ostringstream bad;
        bad << std::fixed << std::setprecision(6) <<
          "'max_correspondence_distances' entries must all be greater than 0 "
          "(got " << distance << ").";
        response->success = false;
        response->message = bad.str();
        return;
      }
    }

    std::string resolved_output_aligned_pcd_path = output_aligned_pcd_path;
    std::string resolved_output_transform_path = output_transform_path;
    if (resolved_output_aligned_pcd_path.empty() ||
      resolved_output_transform_path.empty())
    {
      const std::filesystem::path source_path(source_pcd_path);
      const std::filesystem::path output_dir(kDefaultOutputDir);
      std::filesystem::create_directories(output_dir);
      if (resolved_output_aligned_pcd_path.empty()) {
        resolved_output_aligned_pcd_path =
          (output_dir /
          (source_path.stem().string() + "_fine_aligned.pcd")).string();
      }
      if (resolved_output_transform_path.empty()) {
        resolved_output_transform_path =
          (output_dir /
          (source_path.stem().string() + "_fine_transform.txt")).string();
      }
    }

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    if (!initial_guess_transform_path.empty()) {
      std::string transform_error;
      if (!readTransform(
          initial_guess_transform_path, initial_guess, transform_error))
      {
        response->success = false;
        response->message = transform_error;
        return;
      }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr source(
      new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target(
      new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(source_pcd_path, *source) == -1) {
      response->success = false;
      response->message =
        "Failed to load source PCD file: " + source_pcd_path;
      return;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(target_pcd_path, *target) == -1) {
      response->success = false;
      response->message =
        "Failed to load target PCD file: " + target_pcd_path;
      return;
    }

    if (source->empty() || target->empty()) {
      response->success = false;
      response->message = "One or more input PCD files is empty.";
      return;
    }

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setInputSource(source);
    gicp.setInputTarget(target);
    gicp.setTransformationEpsilon(transformation_epsilon);
    gicp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    gicp.setRotationEpsilon(rotation_epsilon);
    gicp.setMaximumIterations(max_iterations);
    gicp.setCorrespondenceRandomness(correspondence_randomness);

    // Counts source points that land within `distance` of a target point under
    // `candidate`. PCL reports neither the correspondence count nor anything
    // else that distinguishes "GICP fitted the clouds" from "GICP found
    // nothing to fit and handed the initial guess straight back" - in that
    // second case hasConverged() is still true and getFitnessScore() still
    // returns a number. An initial guess pointing the wrong way (the inverse
    // of the transform you meant, say) puts the source tens of metres clear of
    // the target, every gate matches nothing, and the node would otherwise
    // report a confident success over an untouched guess.
    pcl::search::KdTree<pcl::PointXYZI> target_tree;
    target_tree.setInputCloud(target);
    const auto count_correspondences =
      [&](const Eigen::Matrix4f & candidate, const double distance) {
        pcl::PointCloud<pcl::PointXYZI> probe;
        pcl::transformPointCloud(*source, probe, candidate);
        std::vector<int> indices(1);
        std::vector<float> squared_distances(1);
        const float squared_limit =
          static_cast<float>(distance) * static_cast<float>(distance);
        std::size_t hits = 0;
        for (const auto & point : probe.points) {
          if (target_tree.nearestKSearch(point, 1, indices, squared_distances) >
            0 && squared_distances[0] <= squared_limit)
          {
            ++hits;
          }
        }
        return hits;
      };

    // Each pass re-seeds from the previous pass's transform, so `transform`
    // accumulates across the schedule and `aligned` holds the last pass's
    // output. getFitnessScore() is reported from the final (tightest) pass
    // only - scores from different correspondence gates are not comparable, so
    // the per-pass scores go into the message rather than the quality gate.
    pcl::PointCloud<pcl::PointXYZI> aligned;
    Eigen::Matrix4f transform = initial_guess;
    bool converged = false;
    double fitness_score = 0.0;
    std::size_t correspondences = 0;
    std::size_t initial_correspondences = 0;
    std::ostringstream passes;
    passes << std::fixed << std::setprecision(6);
    for (std::size_t i = 0; i < max_correspondence_distances.size(); ++i) {
      const double distance = max_correspondence_distances[i];
      if (i == 0) {
        initial_correspondences = count_correspondences(transform, distance);
      }
      gicp.setMaxCorrespondenceDistance(distance);
      gicp.align(aligned, transform);
      transform = gicp.getFinalTransformation();
      converged = gicp.hasConverged();
      fitness_score = gicp.getFitnessScore();
      correspondences = count_correspondences(transform, distance);
      if (i > 0) {
        passes << ", ";
      }
      passes << "d=" << distance << " -> fitness=" << fitness_score
             << ", correspondences=" << correspondences;
    }

    if (pcl::io::savePCDFileBinary(
        resolved_output_aligned_pcd_path, aligned) != 0)
    {
      response->success = false;
      response->message =
        "Failed to save output aligned PCD file: " +
        resolved_output_aligned_pcd_path;
      return;
    }

    std::ofstream transform_file(resolved_output_transform_path);
    if (!transform_file.is_open()) {
      response->success = false;
      response->message =
        "Failed to save output transform file: " +
        resolved_output_transform_path;
      return;
    }
    transform_file << std::fixed << std::setprecision(6);
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        transform_file << transform(row, col);
        if (col < 3) {
          transform_file << " ";
        }
      }
      transform_file << "\n";
    }
    transform_file.close();

    std::ostringstream msg;
    msg << std::fixed << std::setprecision(6);
    msg << "Loaded source (" << source->size() << " pts) and target ("
        << target->size() << " pts) -> GICP ("
        << max_correspondence_distances.size() << " pass(es): " << passes.str()
        << ", max_iterations=" << max_iterations << ", initial_guess="
        << (initial_guess_transform_path.empty() ? "identity" : "file")
        << ") -> converged=" << (converged ? "true" : "false")
        << ", fitness_score=" << fitness_score << ", correspondences="
        << correspondences << "/" << source->size() << " -> "
        << resolved_output_aligned_pcd_path << ", "
        << resolved_output_transform_path;

    // No correspondences means GICP had nothing to work with: the two clouds
    // never came within the first gate of each other, so whatever is in
    // `transform` is the initial guess, not a registration. Overwhelmingly
    // this is a wrong initial guess rather than a wrong gate - most often the
    // inverse of the intended one, which is what reading back the transform
    // from a run whose source and target were swapped gives you.
    if (correspondences == 0) {
      msg << " -> REJECTED: GICP found no correspondences within "
          << max_correspondence_distances.back()
          << " m, so the transform written is the unmodified initial guess. "
          << "Check that 'initial_guess_transform_path' came from a run with "
          << "the same source and target roles (a transform solved in the "
          << "opposite direction puts the clouds further apart, not closer), "
          << "then that the first entry of 'max_correspondence_distances' "
          << "spans the offset that is left.";
      response->success = false;
      response->message = msg.str();
      return;
    }

    // A guess that already matched nothing at the loosest gate is the same
    // failure caught one step earlier, and is worth saying out loud even when
    // GICP later recovered some correspondences.
    if (initial_correspondences == 0) {
      msg << " -> NOTE: the initial guess itself had no correspondences "
        "within " << max_correspondence_distances.front()
          << " m; GICP started from a pose that does not overlap the target.";
    }

    // fitness_score, not `converged`, is the quality gate: PCL's GICP sets
    // converged_ when it merely runs out of iterations (gicp.hpp,
    // `if (nr_iterations_ >= max_iterations_ || delta < 1)`), so the flag is
    // reported for information only. The outputs above stay on disk when the
    // gate rejects them - re-running GICP over million-point clouds just to
    // look at the result is expensive - but the failure stops the pipeline
    // consuming them.
    if (max_fitness_score > 0.0 && fitness_score > max_fitness_score) {
      msg << " -> REJECTED: fitness_score exceeds max_fitness_score ("
          << max_fitness_score
          << "); outputs were written but should not be used.";
      response->success = false;
      response->message = msg.str();
      return;
    }

    response->success = true;
    response->message = msg.str();
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FineRegistrationNode>());
  rclcpp::shutdown();
  return 0;
}
