#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
constexpr const char * kDefaultOutputDir = MY_POINT_REG_SOURCE_DIR "/output_test_file";
constexpr const char * kMethodSacIa = "sac_ia";
constexpr const char * kMethodYawSweep = "yaw_sweep";

// A 2D cell index packed into one integer so occupancy can live in a hash set
// without a custom hash. Cell indices stay far inside 32 bits for any scene
// this pipeline handles (a 2^31-cell axis is 5e8 km at the default 0.25 m).
inline std::int64_t packCell(const int x, const int y)
{
  return (static_cast<std::int64_t>(x) << 32) ^
         static_cast<std::uint32_t>(y);
}

inline int cellIndex(const float value, const double cell)
{
  return static_cast<int>(std::floor(value / cell));
}

struct Point2D
{
  float x;
  float y;
};

// One candidate planar pose, scored as the fraction of occupied source cells
// that land on an occupied target cell.
struct Pose2D
{
  double score = 0.0;
  double yaw = 0.0;
  double tx = 0.0;
  double ty = 0.0;
};

// Keeps the points whose z falls in [min_z, max_z] and drops the z coordinate.
// The band exists to throw away the floor slab and the ceiling: both are large,
// near-horizontal, and identical everywhere, so they contribute no information
// about yaw or about xy translation while dominating every point count.
std::vector<Point2D> extractBand(
  const pcl::PointCloud<pcl::PointXYZI> & cloud, const double min_z,
  const double max_z)
{
  std::vector<Point2D> out;
  out.reserve(cloud.size());
  for (const auto & point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z))
    {
      continue;
    }
    if (point.z >= min_z && point.z <= max_z) {
      out.push_back(Point2D{point.x, point.y});
    }
  }
  return out;
}

// One representative point per occupied cell. Scoring on cells rather than on
// raw points is what keeps a densely-sampled wall from outvoting the rest of
// the scene - the same reason a fitness score computed over raw points lets a
// ground-plane-heavy cloud free-slide.
std::vector<Point2D> dedupeToCells(
  const std::vector<Point2D> & points, const double cell)
{
  std::unordered_set<std::int64_t> seen;
  seen.reserve(points.size() * 2);
  std::vector<Point2D> out;
  out.reserve(points.size());
  for (const auto & point : points) {
    const std::int64_t key =
      packCell(cellIndex(point.x, cell), cellIndex(point.y, cell));
    if (seen.insert(key).second) {
      out.push_back(point);
    }
  }
  return out;
}

// Dense 2D occupancy bitmap, so the refinement stage can test a candidate pose
// with an array index instead of a hash lookup.
class OccupancyGrid
{
public:
  OccupancyGrid(const std::vector<Point2D> & points, const double cell)
  : cell_(cell)
  {
    if (points.empty()) {
      return;
    }
    min_x_ = max_x_ = cellIndex(points.front().x, cell);
    min_y_ = max_y_ = cellIndex(points.front().y, cell);
    for (const auto & point : points) {
      min_x_ = std::min(min_x_, cellIndex(point.x, cell));
      max_x_ = std::max(max_x_, cellIndex(point.x, cell));
      min_y_ = std::min(min_y_, cellIndex(point.y, cell));
      max_y_ = std::max(max_y_, cellIndex(point.y, cell));
    }
    width_ = max_x_ - min_x_ + 1;
    height_ = max_y_ - min_y_ + 1;
    occupied_.assign(
      static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_),
      false);
    for (const auto & point : points) {
      const int x = cellIndex(point.x, cell) - min_x_;
      const int y = cellIndex(point.y, cell) - min_y_;
      occupied_[static_cast<std::size_t>(y) *
        static_cast<std::size_t>(width_) + static_cast<std::size_t>(x)] = true;
    }
  }

  bool test(const float px, const float py) const
  {
    const int x = cellIndex(px, cell_) - min_x_;
    const int y = cellIndex(py, cell_) - min_y_;
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
      return false;
    }
    return occupied_[static_cast<std::size_t>(y) *
             static_cast<std::size_t>(width_) + static_cast<std::size_t>(x)];
  }

private:
  double cell_;
  int min_x_ = 0;
  int max_x_ = 0;
  int min_y_ = 0;
  int max_y_ = 0;
  int width_ = 0;
  int height_ = 0;
  std::vector<bool> occupied_;
};

// Fraction of `source` cells landing on an occupied `grid` cell under the pose.
double scorePose(
  const std::vector<Point2D> & source, const OccupancyGrid & grid,
  const double yaw, const double tx, const double ty)
{
  if (source.empty()) {
    return 0.0;
  }
  const float cos_yaw = static_cast<float>(std::cos(yaw));
  const float sin_yaw = static_cast<float>(std::sin(yaw));
  const float offset_x = static_cast<float>(tx);
  const float offset_y = static_cast<float>(ty);
  std::size_t hits = 0;
  for (const auto & point : source) {
    const float x = cos_yaw * point.x - sin_yaw * point.y + offset_x;
    const float y = sin_yaw * point.x + cos_yaw * point.y + offset_y;
    if (grid.test(x, y)) {
      ++hits;
    }
  }
  return static_cast<double>(hits) / static_cast<double>(source.size());
}
}  // namespace

// "coarse / global registration" stage of the point-cloud merge pipeline
// (see POINTCLOUD_MERGE_PLAN.md, Step 3). File in, file out: this node does
// nothing on startup, declares its parameters, and does all its work inside
// the `register_coarse` Trigger callback so a human decides when it runs.
//
// Estimates a rough source->target transform with no prior pose guess and
// writes it as plain text for Step 4 (GICP) to use as its initial guess, plus
// the transformed source cloud. Two methods, chosen by the `method` parameter:
//
//   yaw_sweep (default) - exhaustive search over (x, y, yaw) scored on 2D
//     occupancy overlap, consuming the voxelized clouds directly. Assumes both
//     clouds are already gravity-aligned and share a floor height, which is
//     true of anything coming off a LiDAR-inertial SLAM stack: the residual
//     offset is then genuinely 3-DOF, and searching 3 DOF exhaustively cannot
//     land in a local minimum.
//   sac_ia - PCL's SampleConsensusInitialAlignment over the normals + FPFH
//     that feature_node produced. Searches all 6 DOF, so it is the method to
//     use when the clouds are *not* gravity-aligned. Note that FPFH describes
//     only a 1 m-ish neighbourhood, so it cannot tell one aisle of a
//     repetitive scene from another; on the warehouse scans in PCD/ it picks a
//     confidently wrong pose (often the 180-degree flip) every time. Check the
//     reported score before trusting it.
class CoarseRegistrationNode : public rclcpp::Node
{
public:
  CoarseRegistrationNode()
  : Node("coarse_registration_node")
  {
    this->declare_parameter<std::string>("method", kMethodYawSweep);

    // yaw_sweep inputs: the voxelized clouds, straight from voxel_node. It
    // needs no descriptors, so a yaw_sweep run does not need feature_node at
    // all.
    this->declare_parameter<std::string>("source_pcd_path", "");
    this->declare_parameter<std::string>("target_pcd_path", "");

    // sac_ia inputs: feature_node's paired normals + FPFH outputs.
    this->declare_parameter<std::string>("source_normals_pcd_path", "");
    this->declare_parameter<std::string>("source_fpfh_pcd_path", "");
    this->declare_parameter<std::string>("target_normals_pcd_path", "");
    this->declare_parameter<std::string>("target_fpfh_pcd_path", "");

    this->declare_parameter<std::string>("output_aligned_pcd_path", "");
    this->declare_parameter<std::string>("output_transform_path", "");

    // --- yaw_sweep ---
    // Height band kept for scoring, in the clouds' own frame. The defaults
    // assume a floor at z ~ 0 and keep waist-to-ceiling structure: walls,
    // racking, furniture. Widen it for a scene with structure outside this
    // band, but never widen it to include the floor - the floor is the thing
    // that makes every scoring metric here lie.
    this->declare_parameter<double>("sweep_min_z", 0.6);
    this->declare_parameter<double>("sweep_max_z", 4.0);
    // Full circle by default. Narrow it when the yaw offset is roughly known;
    // the cost is linear in the number of steps.
    this->declare_parameter<double>("sweep_yaw_min_deg", 0.0);
    this->declare_parameter<double>("sweep_yaw_max_deg", 360.0);
    this->declare_parameter<double>("sweep_yaw_step_deg", 1.0);
    // The search runs coarse-then-fine. The vote pass costs
    // O(source_cells * target_cells) per yaw step, and both counts scale with
    // scene area over vote_cell_size squared, so this is the parameter that
    // sets the runtime - 1.0 m keeps a warehouse-sized pair to a few seconds.
    // The refine pass then re-scores the best candidates at cell_size.
    this->declare_parameter<double>("sweep_vote_cell_size", 1.0);
    this->declare_parameter<double>("sweep_cell_size", 0.25);
    // How many of the vote pass's best poses get refined. More than one
    // because a repetitive scene produces several near-tied candidates one
    // aisle apart, and the tie is broken at the finer resolution.
    this->declare_parameter<int>("sweep_refine_candidates", 8);

    // Quality gate, 0 disables it. The two methods report scores that are not
    // comparable with each other: sac_ia's is a mean squared correspondence
    // distance (lower is better, hence max_fitness_score), yaw_sweep's is an
    // overlap fraction in [0, 1] (higher is better, hence min_overlap_score).
    // A meaningful threshold depends on how much the two clouds really
    // overlap, so there is no useful non-zero default - run once, read the
    // score out of this node's response, then set the gate just inside a
    // known-good run.
    this->declare_parameter<double>("max_fitness_score", 0.0);
    this->declare_parameter<double>("min_overlap_score", 0.0);

    this->declare_parameter<double>("min_sample_distance", 0.05);
    this->declare_parameter<double>("max_correspondence_distance", 1.0);
    this->declare_parameter<int>("nr_iterations", 500);
    this->declare_parameter<int>("number_of_samples", 3);
    this->declare_parameter<int>("correspondence_randomness", 10);

    service_ = this->create_service<std_srvs::srv::Trigger>(
      "register_coarse",
      std::bind(
        &CoarseRegistrationNode::handleRegisterCoarse, this,
        std::placeholders::_1, std::placeholders::_2));
  }

private:
  // Resolves the two output paths, defaulting them under kDefaultOutputDir
  // from `stem_source`'s stem when left empty.
  void resolveOutputPaths(
    const std::string & stem_source, std::string & aligned_path,
    std::string & transform_path) const
  {
    if (!aligned_path.empty() && !transform_path.empty()) {
      return;
    }
    const std::filesystem::path source_path(stem_source);
    const std::filesystem::path output_dir(kDefaultOutputDir);
    std::filesystem::create_directories(output_dir);
    if (aligned_path.empty()) {
      aligned_path =
        (output_dir /
        (source_path.stem().string() + "_coarse_aligned.pcd")).string();
    }
    if (transform_path.empty()) {
      transform_path =
        (output_dir /
        (source_path.stem().string() + "_coarse_transform.txt")).string();
    }
  }

  bool writeTransform(
    const std::string & path, const Eigen::Matrix4f & transform,
    std::string & error_message) const
  {
    std::ofstream transform_file(path);
    if (!transform_file.is_open()) {
      error_message = "Failed to save output transform file: " + path;
      return false;
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
    return true;
  }

  void handleRegisterCoarse(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const std::string method = this->get_parameter("method").as_string();
    if (method == kMethodYawSweep) {
      runYawSweep(response);
      return;
    }
    if (method == kMethodSacIa) {
      runSacIa(response);
      return;
    }
    response->success = false;
    response->message =
      "Unknown 'method' parameter '" + method + "' - expected '" +
      kMethodYawSweep + "' or '" + kMethodSacIa + "'.";
  }

  void runYawSweep(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const std::string source_pcd_path =
      this->get_parameter("source_pcd_path").as_string();
    const std::string target_pcd_path =
      this->get_parameter("target_pcd_path").as_string();
    const double min_z = this->get_parameter("sweep_min_z").as_double();
    const double max_z = this->get_parameter("sweep_max_z").as_double();
    const double yaw_min_deg =
      this->get_parameter("sweep_yaw_min_deg").as_double();
    const double yaw_max_deg =
      this->get_parameter("sweep_yaw_max_deg").as_double();
    const double yaw_step_deg =
      this->get_parameter("sweep_yaw_step_deg").as_double();
    const double vote_cell_size =
      this->get_parameter("sweep_vote_cell_size").as_double();
    const double cell_size = this->get_parameter("sweep_cell_size").as_double();
    const int refine_candidates =
      this->get_parameter("sweep_refine_candidates").as_int();
    const double min_overlap_score =
      this->get_parameter("min_overlap_score").as_double();

    if (source_pcd_path.empty() || target_pcd_path.empty()) {
      response->success = false;
      response->message =
        "'source_pcd_path' and 'target_pcd_path' parameters must both be set "
        "for method '" + std::string(kMethodYawSweep) + "'.";
      return;
    }
    if (max_z <= min_z) {
      response->success = false;
      response->message =
        "'sweep_max_z' must be greater than 'sweep_min_z'.";
      return;
    }
    if (vote_cell_size <= 0.0 || cell_size <= 0.0) {
      response->success = false;
      response->message =
        "'sweep_vote_cell_size' and 'sweep_cell_size' must both be greater "
        "than 0.";
      return;
    }
    if (yaw_step_deg <= 0.0 || yaw_max_deg <= yaw_min_deg) {
      response->success = false;
      response->message =
        "'sweep_yaw_step_deg' must be greater than 0 and 'sweep_yaw_max_deg' "
        "greater than 'sweep_yaw_min_deg'.";
      return;
    }
    if (refine_candidates < 1) {
      response->success = false;
      response->message =
        "'sweep_refine_candidates' must be at least 1.";
      return;
    }

    std::string aligned_path =
      this->get_parameter("output_aligned_pcd_path").as_string();
    std::string transform_path =
      this->get_parameter("output_transform_path").as_string();
    resolveOutputPaths(source_pcd_path, aligned_path, transform_path);

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

    const std::vector<Point2D> source_band = extractBand(*source, min_z, max_z);
    const std::vector<Point2D> target_band = extractBand(*target, min_z, max_z);
    if (source_band.empty() || target_band.empty()) {
      std::ostringstream bad;
      bad << std::fixed << std::setprecision(6) <<
        "No points left after the height band [" << min_z << ", " << max_z <<
        "] (source kept " << source_band.size() << ", target kept " <<
        target_band.size() <<
        ") - check that sweep_min_z/sweep_max_z match the clouds' frame.";
      response->success = false;
      response->message = bad.str();
      return;
    }

    // --- pass 1: vote for the best (yaw, translation) at vote_cell_size ---
    // For one yaw, every (source cell, target cell) pair implies the exact
    // translation that would make them coincide, so accumulating a histogram
    // over those implied translations and taking its peak finds the best
    // translation for that yaw in a single sweep of the pair list.
    const std::vector<Point2D> source_vote =
      dedupeToCells(source_band, vote_cell_size);
    const std::vector<Point2D> target_vote =
      dedupeToCells(target_band, vote_cell_size);

    std::vector<int> target_x;
    std::vector<int> target_y;
    target_x.reserve(target_vote.size());
    target_y.reserve(target_vote.size());
    int target_min_x = cellIndex(target_vote.front().x, vote_cell_size);
    int target_max_x = target_min_x;
    int target_min_y = cellIndex(target_vote.front().y, vote_cell_size);
    int target_max_y = target_min_y;
    for (const auto & point : target_vote) {
      const int x = cellIndex(point.x, vote_cell_size);
      const int y = cellIndex(point.y, vote_cell_size);
      target_x.push_back(x);
      target_y.push_back(y);
      target_min_x = std::min(target_min_x, x);
      target_max_x = std::max(target_max_x, x);
      target_min_y = std::min(target_min_y, y);
      target_max_y = std::max(target_max_y, y);
    }

    const int yaw_steps = static_cast<int>(
      std::floor((yaw_max_deg - yaw_min_deg) / yaw_step_deg));
    std::vector<Pose2D> candidates;
    std::vector<std::uint32_t> votes;
    std::vector<int> rotated_x;
    std::vector<int> rotated_y;

    for (int step = 0; step < yaw_steps; ++step) {
      const double yaw_deg = yaw_min_deg + step * yaw_step_deg;
      const double yaw = yaw_deg * M_PI / 180.0;
      const float cos_yaw = static_cast<float>(std::cos(yaw));
      const float sin_yaw = static_cast<float>(std::sin(yaw));

      rotated_x.clear();
      rotated_y.clear();
      rotated_x.reserve(source_vote.size());
      rotated_y.reserve(source_vote.size());
      int source_min_x = 0;
      int source_max_x = 0;
      int source_min_y = 0;
      int source_max_y = 0;
      for (std::size_t i = 0; i < source_vote.size(); ++i) {
        const float rx = cos_yaw * source_vote[i].x - sin_yaw *
          source_vote[i].y;
        const float ry = sin_yaw * source_vote[i].x + cos_yaw *
          source_vote[i].y;
        const int x = cellIndex(rx, vote_cell_size);
        const int y = cellIndex(ry, vote_cell_size);
        rotated_x.push_back(x);
        rotated_y.push_back(y);
        if (i == 0) {
          source_min_x = source_max_x = x;
          source_min_y = source_max_y = y;
        } else {
          source_min_x = std::min(source_min_x, x);
          source_max_x = std::max(source_max_x, x);
          source_min_y = std::min(source_min_y, y);
          source_max_y = std::max(source_max_y, y);
        }
      }

      // Translation histogram, indexed by cell-space shift. Its span is the
      // sum of the two clouds' spans, so every shift that puts any source cell
      // on any target cell has a bin.
      const int shift_min_x = target_min_x - source_max_x;
      const int shift_min_y = target_min_y - source_max_y;
      const int shift_width = (target_max_x - target_min_x) +
        (source_max_x - source_min_x) + 1;
      const int shift_height = (target_max_y - target_min_y) +
        (source_max_y - source_min_y) + 1;

      votes.assign(
        static_cast<std::size_t>(shift_width) *
        static_cast<std::size_t>(shift_height), 0U);
      for (std::size_t s = 0; s < rotated_x.size(); ++s) {
        for (std::size_t t = 0; t < target_x.size(); ++t) {
          const int dx = target_x[t] - rotated_x[s] - shift_min_x;
          const int dy = target_y[t] - rotated_y[s] - shift_min_y;
          ++votes[static_cast<std::size_t>(dy) *
            static_cast<std::size_t>(shift_width) +
            static_cast<std::size_t>(dx)];
        }
      }

      std::uint32_t best_votes = 0;
      std::size_t best_index = 0;
      for (std::size_t i = 0; i < votes.size(); ++i) {
        if (votes[i] > best_votes) {
          best_votes = votes[i];
          best_index = i;
        }
      }
      if (best_votes == 0) {
        continue;
      }
      const int best_dy = static_cast<int>(best_index / shift_width);
      const int best_dx = static_cast<int>(best_index % shift_width);
      Pose2D pose;
      pose.score = static_cast<double>(best_votes) /
        static_cast<double>(source_vote.size());
      pose.yaw = yaw;
      pose.tx = (best_dx + shift_min_x) * vote_cell_size;
      pose.ty = (best_dy + shift_min_y) * vote_cell_size;
      candidates.push_back(pose);
    }

    if (candidates.empty()) {
      response->success = false;
      response->message =
        "Yaw sweep found no overlapping pose at all - the two clouds share no "
        "structure in the height band, or the band is wrong.";
      return;
    }

    std::sort(
      candidates.begin(), candidates.end(),
      [](const Pose2D & a, const Pose2D & b) {return a.score > b.score;});
    if (candidates.size() > static_cast<std::size_t>(refine_candidates)) {
      candidates.resize(static_cast<std::size_t>(refine_candidates));
    }

    // --- pass 2: refine each surviving candidate at cell_size ---
    // The vote pass can only resolve translation to vote_cell_size and yaw to
    // yaw_step, so each candidate is re-scored over a local neighbourhood at
    // the finer resolution. This is also what separates candidates that the
    // coarse pass left near-tied.
    const std::vector<Point2D> source_fine =
      dedupeToCells(source_band, cell_size);
    const OccupancyGrid target_grid(target_band, cell_size);

    const double yaw_step = yaw_step_deg * M_PI / 180.0;
    const int yaw_refine_steps = 8;
    const int shift_refine_steps =
      std::max(1, static_cast<int>(std::ceil(vote_cell_size / cell_size)));

    Pose2D best;
    for (const auto & candidate : candidates) {
      for (int iy = -yaw_refine_steps; iy <= yaw_refine_steps; ++iy) {
        const double yaw =
          candidate.yaw + (yaw_step * iy) / yaw_refine_steps;
        for (int ix = -shift_refine_steps; ix <= shift_refine_steps; ++ix) {
          for (int jy = -shift_refine_steps; jy <= shift_refine_steps; ++jy) {
            const double tx = candidate.tx + ix * cell_size;
            const double ty = candidate.ty + jy * cell_size;
            const double score =
              scorePose(source_fine, target_grid, yaw, tx, ty);
            if (score > best.score) {
              best.score = score;
              best.yaw = yaw;
              best.tx = tx;
              best.ty = ty;
            }
          }
        }
      }
    }

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    const float cos_yaw = static_cast<float>(std::cos(best.yaw));
    const float sin_yaw = static_cast<float>(std::sin(best.yaw));
    transform(0, 0) = cos_yaw;
    transform(0, 1) = -sin_yaw;
    transform(1, 0) = sin_yaw;
    transform(1, 1) = cos_yaw;
    transform(0, 3) = static_cast<float>(best.tx);
    transform(1, 3) = static_cast<float>(best.ty);

    pcl::PointCloud<pcl::PointXYZI> aligned;
    pcl::transformPointCloud(*source, aligned, transform);
    if (pcl::io::savePCDFileBinary(aligned_path, aligned) != 0) {
      response->success = false;
      response->message =
        "Failed to save output aligned PCD file: " + aligned_path;
      return;
    }
    std::string error_message;
    if (!writeTransform(transform_path, transform, error_message)) {
      response->success = false;
      response->message = error_message;
      return;
    }

    double yaw_deg = best.yaw * 180.0 / M_PI;
    while (yaw_deg > 180.0) {
      yaw_deg -= 360.0;
    }
    while (yaw_deg <= -180.0) {
      yaw_deg += 360.0;
    }

    std::ostringstream msg;
    msg << std::fixed << std::setprecision(6);
    msg << "Loaded source (" << source->size() << " pts) and target ("
        << target->size() << " pts) -> yaw_sweep (height band [" << min_z
        << ", " << max_z << "] kept " << source_band.size() << "/"
        << target_band.size() << " pts -> " << source_vote.size() << "/"
        << target_vote.size() << " vote cells at " << vote_cell_size
        << " m, " << source_fine.size() << " source cells at " << cell_size
        << " m; " << yaw_steps << " yaw steps of " << yaw_step_deg
        << " deg) -> yaw=" << yaw_deg << " deg, t=(" << best.tx << ", "
        << best.ty << "), overlap_score=" << best.score << " -> "
        << aligned_path << ", " << transform_path;

    // The outputs above stay on disk when the gate rejects them - re-running
    // the sweep just to look at the result is expensive - but the failure
    // stops the pipeline consuming them.
    if (min_overlap_score > 0.0 && best.score < min_overlap_score) {
      msg << " -> REJECTED: overlap_score is below min_overlap_score ("
          << min_overlap_score
          << "); outputs were written but should not be used.";
      response->success = false;
      response->message = msg.str();
      return;
    }

    response->success = true;
    response->message = msg.str();
  }

  void runSacIa(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const std::string source_normals_pcd_path =
      this->get_parameter("source_normals_pcd_path").as_string();
    const std::string source_fpfh_pcd_path =
      this->get_parameter("source_fpfh_pcd_path").as_string();
    const std::string target_normals_pcd_path =
      this->get_parameter("target_normals_pcd_path").as_string();
    const std::string target_fpfh_pcd_path =
      this->get_parameter("target_fpfh_pcd_path").as_string();
    const double min_sample_distance =
      this->get_parameter("min_sample_distance").as_double();
    const double max_correspondence_distance =
      this->get_parameter("max_correspondence_distance").as_double();
    const int nr_iterations = this->get_parameter("nr_iterations").as_int();
    const int number_of_samples =
      this->get_parameter("number_of_samples").as_int();
    const int correspondence_randomness =
      this->get_parameter("correspondence_randomness").as_int();
    const double max_fitness_score =
      this->get_parameter("max_fitness_score").as_double();

    if (source_normals_pcd_path.empty() || source_fpfh_pcd_path.empty() ||
      target_normals_pcd_path.empty() || target_fpfh_pcd_path.empty())
    {
      response->success = false;
      response->message =
        "'source_normals_pcd_path', 'source_fpfh_pcd_path', "
        "'target_normals_pcd_path' and 'target_fpfh_pcd_path' parameters "
        "must all be set for method '" + std::string(kMethodSacIa) + "'.";
      return;
    }

    std::string aligned_path =
      this->get_parameter("output_aligned_pcd_path").as_string();
    std::string transform_path =
      this->get_parameter("output_transform_path").as_string();
    resolveOutputPaths(source_normals_pcd_path, aligned_path, transform_path);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_normals(
      new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_normals(
      new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh(
      new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh(
      new pcl::PointCloud<pcl::FPFHSignature33>);

    if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(
        source_normals_pcd_path, *source_normals) == -1)
    {
      response->success = false;
      response->message =
        "Failed to load source normals PCD file: " + source_normals_pcd_path;
      return;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(
        target_normals_pcd_path, *target_normals) == -1)
    {
      response->success = false;
      response->message =
        "Failed to load target normals PCD file: " + target_normals_pcd_path;
      return;
    }
    if (pcl::io::loadPCDFile<pcl::FPFHSignature33>(
        source_fpfh_pcd_path, *source_fpfh) == -1)
    {
      response->success = false;
      response->message =
        "Failed to load source FPFH PCD file: " + source_fpfh_pcd_path;
      return;
    }
    if (pcl::io::loadPCDFile<pcl::FPFHSignature33>(
        target_fpfh_pcd_path, *target_fpfh) == -1)
    {
      response->success = false;
      response->message =
        "Failed to load target FPFH PCD file: " + target_fpfh_pcd_path;
      return;
    }

    if (source_normals->empty() || target_normals->empty() ||
      source_fpfh->empty() || target_fpfh->empty())
    {
      response->success = false;
      response->message = "One or more input PCD files is empty.";
      return;
    }

    if (source_normals->size() != source_fpfh->size()) {
      response->success = false;
      response->message =
        "Source normals/FPFH point count mismatch (" +
        std::to_string(source_normals->size()) + " vs " +
        std::to_string(source_fpfh->size()) +
        ") - they must come from the same feature_node run.";
      return;
    }
    if (target_normals->size() != target_fpfh->size()) {
      response->success = false;
      response->message =
        "Target normals/FPFH point count mismatch (" +
        std::to_string(target_normals->size()) + " vs " +
        std::to_string(target_fpfh->size()) +
        ") - they must come from the same feature_node run.";
      return;
    }

    pcl::SampleConsensusInitialAlignment<
      pcl::PointXYZINormal, pcl::PointXYZINormal, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source_normals);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(target_normals);
    sac_ia.setTargetFeatures(target_fpfh);
    sac_ia.setMinSampleDistance(static_cast<float>(min_sample_distance));
    sac_ia.setMaxCorrespondenceDistance(
      static_cast<float>(max_correspondence_distance));
    sac_ia.setMaximumIterations(nr_iterations);
    sac_ia.setNumberOfSamples(number_of_samples);
    sac_ia.setCorrespondenceRandomness(correspondence_randomness);

    pcl::PointCloud<pcl::PointXYZINormal> aligned;
    sac_ia.align(aligned);

    const bool converged = sac_ia.hasConverged();
    const double fitness_score = sac_ia.getFitnessScore();
    const Eigen::Matrix4f transform = sac_ia.getFinalTransformation();

    if (pcl::io::savePCDFileBinary(aligned_path, aligned) != 0) {
      response->success = false;
      response->message =
        "Failed to save output aligned PCD file: " + aligned_path;
      return;
    }
    std::string error_message;
    if (!writeTransform(transform_path, transform, error_message)) {
      response->success = false;
      response->message = error_message;
      return;
    }

    std::ostringstream msg;
    msg << std::fixed << std::setprecision(6);
    msg << "Loaded source (" << source_normals->size() << " pts) and target ("
        << target_normals->size()
        << " pts) -> SAC-IA (min_sample_distance=" << min_sample_distance
        << ", max_correspondence_distance=" << max_correspondence_distance
        << ", nr_iterations=" << nr_iterations << ") -> converged="
        << (converged ? "true" : "false") << ", fitness_score="
        << fitness_score << " -> " << aligned_path << ", " << transform_path;

    // fitness_score, not `converged`, is the quality gate: with no initial
    // guess PCL's SAC-IA sets converged_ on its first iteration regardless of
    // how bad the result is (ia_ransac.hpp, `if (i_iter == 0 || error <
    // lowest_error)`), so the flag is reported for information only.
    // The outputs above stay on disk when the gate rejects them - re-running
    // SAC-IA over million-point clouds just to look at the result is
    // expensive - but the failure stops the pipeline consuming them.
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
  rclcpp::spin(std::make_shared<CoarseRegistrationNode>());
  rclcpp::shutdown();
  return 0;
}
