# my_point_reg

Point-cloud registration/merge pipeline for two overlapping `.pcd` scans
(e.g. a ground and an aerial LiDAR pass of the same space). Everything is
**file in, file out** — no cloud topics are published or subscribed.

Pipeline: voxel downsample → coarse (global) registration → fine
(GICP) registration → merge → dedup. Each stage is its own node behind a
`std_srvs/srv/Trigger` service; `pipeline_node` runs all of them in order
from one call.

## Prerequisites

- ROS 2 Jazzy (or another rosdep-supported distro — adjust the `source` path below)
- PCL, plus its ROS wrappers:

  ```bash
  sudo apt install libpcl-dev ros-jazzy-pcl-ros ros-jazzy-pcl-conversions
  ```

- Two overlapping `.pcd` scans to register, ideally already gravity-aligned
  (floor near z = 0) — that's what `yaw_sweep` coarse registration assumes.

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select my_point_reg \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Release is not optional in practice — the filters are slow on multi-million-point
clouds in a debug build.

## Quick start: run the whole pipeline

```bash
ros2 launch my_point_reg pipeline.launch.py \
  source_pcd_path:=/absolute/path/to/source.pcd \
  target_pcd_path:=/absolute/path/to/target.pcd \
  leaf_size:=0.1
# in a second terminal:
ros2 service call /run_pipeline std_srvs/srv/Trigger
```

```bash
#example
ros2 launch my_point_reg pipeline.launch.py \
  source_pcd_path:=$(pwd)/PCD/notfull_scan_low.pcd \
  target_pcd_path:=$(pwd)/PCD/full_scan_high.pcd \
  leaf_size:=0.1 \
  output_dir:=output_test_file
```

Use absolute paths — bash doesn't expand `~` after `:=`. `source` is the
cloud that gets transformed; `target` is held fixed. Output (including the
merged, deduplicated cloud) lands in `output_dir` (default
`output_test_file/`, created next to wherever the launch was run from).

The response `message` returns a log of every stage: point counts, fitness
scores, and file paths. A failing stage stops the run and reports which one
and why — partial output stays on disk for inspection.

## Nodes

Service names are relative and unnamespaced. Source is aligned onto target
in every stage; **source is transformed, target never moves.**

| node | service | what it does |
|---|---|---|
| `voxel_node` | `/voxelize` | `VoxelGrid` downsample, optional `StatisticalOutlierRemoval` |
| `feature_node` | `/estimate_features` | normals + FPFH descriptors (only needed by `sac_ia` coarse registration) |
| `coarse_registration_node` | `/register_coarse` | rough global alignment, no prior pose needed. `method:=yaw_sweep` (default) exhaustively searches x/y/yaw over the voxelized clouds — use this for gravity-aligned scans (e.g. off a LiDAR-inertial SLAM stack). `method:=sac_ia` matches FPFH features — use only for scans that are *not* gravity-aligned; it cannot disambiguate a repetitive scene (see Troubleshooting) |
| `fine_registration_node` | `/register_fine` | GICP refinement, seeded by the coarse transform (or identity if skipped) |
| `merge_node` | `/merge` | applies the final transform and concatenates source onto target |
| `dedup_node` | `/deduplicate` | re-runs `VoxelGrid` on the merged cloud to collapse duplicate points in the overlap |
| `pipeline_node` | `/run_pipeline` | drives all of the above in order |

Every node declares all its parameters with defaults; run each with `--ros-args -p name:=value`.
List them with `ros2 param list /<node_name>` once it's running, or read the
top of the corresponding `src/<node>.cpp` — every default lives there.

Running a single stage by hand follows the same shape everywhere:

```bash
ros2 run my_point_reg voxel_node --ros-args -p input_pcd_path:=/path/to/input.pcd
# in a second terminal:
ros2 service call /voxelize std_srvs/srv/Trigger
```

Leave any `output_*_pcd_path`/`output_transform_path` unset and it resolves to
`output_test_file/<input_stem>_<suffix>.pcd` (directory auto-created). Only
run one instance of a given node at a time — they share a fixed node name, so
a second instance competes for the same service.

Coarse and fine registration both write a plain-text 4×4 row-major transform
(space-separated) alongside the aligned cloud; `fine_registration_node` and
`merge_node` read that transform back in. A transform file doesn't record
which direction it was solved in — name it accordingly if you keep more than
one around (e.g. `source_to_target.txt`).

## Troubleshooting

**Registration "converged" but the merge looks wrong.** Ignore the
`converged` flag — PCL sets it almost unconditionally on both SAC-IA and
GICP. Judge by `fitness_score` (lower is better for both coarse and fine) and,
for `yaw_sweep`, `overlap_score` (higher is better). Fitness can still be
misleading on floor-heavy clouds, which free-slide to maximize floor-on-floor
overlap and score *better* while being visibly wrong — crop the floor out
and check whether walls/structure line up before trusting a score.

**`fine_registration_node` reports `correspondences=0`.** GICP found nothing
within its gate and returned the initial guess untouched, despite reporting
`converged=true`. Almost always means the coarse transform was solved in the
wrong direction, or the first correspondence gate is too tight for the
remaining offset.

**Reject a bad registration automatically** with `coarse_max_fitness_score` /
`fine_max_fitness_score` (`pipeline_node`) or `max_fitness_score` (stage
nodes directly) — both default to `0.0` (disabled). Do one baseline run, read
the scores back from the response, and set the thresholds just above them.

**FPFH/SAC-IA picks a plausible-looking wrong answer.** A descriptor built
from a ~1 m neighborhood can't tell one repeated structure (e.g. a warehouse
aisle) from the next, so RANSAC has many equally self-consistent
correspondence sets to pick from. Prefer `yaw_sweep` for gravity-aligned
scans; it searches x/y/yaw exhaustively instead of relying on feature
matching.

**Stray node from an earlier run answers instead of the current one.**
Stage nodes and `pipeline_node` are addressed by name — `ros2 node list`
warns on duplicates. Find and kill strays with `ps -ef | grep my_point_reg`
before launching again.

**Pipeline times out on a stage.** `stage_timeout_sec` (default 1800s) is
the per-stage budget. Raise it, or voxelize harder before registration.
