# Point Cloud Merge Workflow

End-to-end workflow for merging two `.pcd` scans — `PCD/ground.pcd` as the
**source** and `PCD/air.pcd` as the **target**. The source is always the cloud
that gets transformed; the target is held fixed.

Two ways to run it:

- **[One command](#option-a-run-the-whole-pipeline-at-once)** — `pipeline_node`
  drives all six stage nodes in order. Use this normally.
- **[Stage by stage](#option-b-stage-by-stage)** — run each node yourself.
  Use this when tuning a single stage or debugging one that failed.

## Pipeline Overview

```
Source Cloud              Target Cloud
     |                         |
     v                         v
[1 Voxelize]              [2 Voxelize]
     |                         |
     v                         v
 <src>_voxelized.pcd     <tgt>_voxelized.pcd
     |     |                   |     |
     |     |  coarse_method=sac_ia only:
     |     v                   |     v
     | [Features]              | [Features]
     |     |                   |     |
     |     v                   |     v
     | normals + FPFH          | normals + FPFH
     |     |                   |     |
     |     +--------+----------+     |
     |              v                |
     |   [3 Coarse Registration]     |
     |     yaw_sweep (default)       |
     |       reads the voxelized     |
     |       clouds directly         |
     |     sac_ia                    |
     |       reads normals + FPFH    |
     |              |                |
     |              v                |
     |     ..._coarse_transform.txt  |
     |              |                |
     |     (initial guess)           |
     |              v                |
     +----> [4 Fine Registration (GICP)] <----+
                    |
                    v
           ..._fine_transform.txt
                    |
     +--------------+----------------+
     |              v                |
     +--------> [5 Merge] <----------+
                    |
                    v
             ..._merged.pcd
        (target + transformed source)
                    |
                    v
             [6 Deduplicate]
                    |
                    v
           ..._deduplicated.pcd
     (VoxelGrid re-run to collapse
        overlap-region duplicates)
```

Coarse registration, fine registration and the merge all consume the
*voxelized* clouds — the full-resolution originals are only ever read by
steps 1 and 2. Deduplication consumes the merge output.

The default `coarse_method` is **`yaw_sweep`**, which needs no descriptors, so
the two feature stages drop out and a default run is 5 steps plus dedup, 6
total. `coarse_method:=sac_ia` puts the feature stages back for 7 steps plus
dedup, 8 total. See [Choosing a coarse method](#choosing-a-coarse-method).

Plan Step 6 in `POINTCLOUD_MERGE_PLAN.md` (deduplicating the merged cloud) is
implemented as `dedup_node` and runs by default; set `skip_dedup:=true` to
leave the merged cloud as-is — see [Next Steps](#next-steps).

## Choosing a coarse method

The coarse stage exists to hand GICP an initial guess. GICP refines; it cannot
search. Given a pair that is tens of degrees and tens of metres apart, no
correspondence-distance schedule recovers it — which is exactly what
`PCD/ground.pcd` and `PCD/air.pcd` are, at **87° of yaw and 47 m** apart.

| | `yaw_sweep` (default) | `sac_ia` |
|---|---|---|
| searches | (x, y, yaw) exhaustively | all 6 DOF, by random sampling |
| needs feature_node | no | yes |
| assumes | both clouds gravity-aligned, floors at a common height | nothing |
| deterministic | yes | no — re-runs disagree |
| score reported | `overlap_score`, 0–1, **higher is better** | `fitness_score`, **lower is better** |
| gate parameter | `min_overlap_score` | `max_fitness_score` |

**Use `yaw_sweep`** for anything off a LiDAR-inertial SLAM stack. Those clouds
arrive gravity-aligned, so roll, pitch and z are already solved and only three
degrees of freedom are left — and three can be searched exhaustively, which
means no local minimum to fall into.

**Use `sac_ia`** only when the clouds are genuinely not gravity-aligned, and
check the score before believing it. FPFH describes roughly a 1 m
neighbourhood, so in a repetitive scene it cannot tell one aisle from the next;
on the warehouse scans in `PCD/` it returns a confidently wrong pose (usually
the 180° flip) on every attempt, and reports a *better* fitness score for it
than the correct answer scores.

## Prerequisites

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_point_reg \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Option A: run the whole pipeline at once

`pipeline_node` sets each stage node's parameters over the parameter service,
calls its Trigger service, waits, and moves on. The launch file starts one
instance of every stage node plus the orchestrator.

**Terminal 1:**
```bash
ros2 launch my_point_reg pipeline.launch.py \
  source_pcd_path:=/home/fishman/ros2_ws/src/my_point_reg/PCD/ground.pcd \
  target_pcd_path:=/home/fishman/ros2_ws/src/my_point_reg/PCD/air.pcd \
  output_dir:=/home/fishman/ros2_ws/src/my_point_reg/output_test_file \
  leaf_size:=0.1
```

**Terminal 2** — optionally tune anything that isn't a launch argument, then run:
```bash
ros2 param set /pipeline_node fine_max_correspondence_distances "[0.5, 0.3, 0.2]"

ros2 service call /run_pipeline std_srvs/srv/Trigger
```

The call blocks until the whole pipeline finishes (GICP on million-point clouds
takes minutes, and SAC-IA far longer — the per-stage budget is
`stage_timeout_sec`, default 1800 s). The response `message` is the log of
every stage in order, each with its elapsed time, point counts and score.

With the defaults (`skip_coarse:=false`, `coarse_method:=yaw_sweep`,
`skip_dedup:=false`) the feature stages are dropped and the run is 6 steps:

```
1/6 voxelize source (12.3s): Loaded 928016 points -> voxelized to 43022 ...
2/6 voxelize target (8.9s): Loaded 1729598 points -> voxelized to 103799 ...
3/6 coarse registration (6.1s): ... -> yaw_sweep (height band [0.600000, 4.000000] ...
    -> yaw=-87.030000 deg, t=(5.872000, 46.725000), overlap_score=0.71 ...
4/6 fine registration (96.4s): ... GICP (3 pass(es): d=0.500000 -> fitness=..., ...
5/6 merge (0.9s): ... -> merged (146821 pts) -> .../ground_to_air_merged.pcd
6/6 deduplicate (1.2s): Loaded 146821 points -> deduplicated to 132904 (removed
    13917, leaf_size=0.100000, min_points_per_voxel=1) -> .../ground_to_air_deduplicated.pcd
Merged cloud: .../ground_to_air_merged.pcd
Deduplicated cloud: .../ground_to_air_deduplicated.pcd
```

`coarse_method:=sac_ia` puts the two feature stages back and the labels read
`1/8` … `8/8`. `skip_coarse:=true` drops the coarse stage entirely, GICP starts
from identity, and the labels read `1/5` … `5/5` — only safe when the pair is
already known to be near-aligned. `skip_dedup:=true` drops the last step from
whichever count applies (5/7/4 instead of 6/8/5) and the response's final line
is just `Merged cloud: ...` with no deduplicated-cloud line.

If a stage fails the response is `success=False`, containing the log up to that
point followed by `FAILED at <stage>: <the stage's own error message>`.

**Outputs**, all in `output_dir` (default `output_test_file/`, resolved from the
directory you ran `ros2 launch` in):

| file | from |
|---|---|
| `ground_voxelized.pcd`, `air_voxelized.pcd` | steps 1–2 |
| `ground_normals.pcd`, `ground_fpfh.pcd` | features source (`coarse_method:=sac_ia` only) |
| `air_normals.pcd`, `air_fpfh.pcd` | features target (`coarse_method:=sac_ia` only) |
| `ground_to_air_coarse_aligned.pcd`, `ground_to_air_coarse_transform.txt` | coarse registration (unless `skip_coarse:=true`) |
| `ground_to_air_fine_aligned.pcd`, `ground_to_air_fine_transform.txt` | fine registration |
| `ground_to_air_merged.pcd` | merge |
| `ground_to_air_deduplicated.pcd` | deduplicate (unless `skip_dedup:=true`) |

Set `output_merged_pcd_path` to put the final merge somewhere specific. The
deduplicated output always lands at `<pair_stem>_deduplicated.pcd` in
`output_dir` — there is no equivalent override on `pipeline_node` itself (run
`dedup_node` by hand, per [Option B](#option-b-stage-by-stage), for a custom
path).

### Launch arguments vs parameters

Launch arguments: `source_pcd_path`, `target_pcd_path`, `output_dir`,
`leaf_size`, `normal_radius`, `fpfh_radius`.

Everything else is a `pipeline_node` parameter, set with `ros2 param set`
before calling the service. Names that exist on more than one stage are
prefixed:

- sequence: `coarse_method` (default `yaw_sweep`, or `sac_ia` — see
  [Choosing a coarse method](#choosing-a-coarse-method)), `skip_coarse`
  (default false; true drops the coarse stage and starts GICP from identity)
- voxel: `remove_outliers`, `sor_mean_k`, `sor_stddev_mul_thresh`
- features (`sac_ia` only): `feature_num_threads`, `orient_normals_up`
- coarse, `yaw_sweep`: `coarse_sweep_min_z`, `coarse_sweep_max_z`,
  `coarse_sweep_yaw_min_deg`, `coarse_sweep_yaw_max_deg`,
  `coarse_sweep_yaw_step_deg`, `coarse_sweep_vote_cell_size`,
  `coarse_sweep_cell_size`, `coarse_sweep_refine_candidates`,
  `coarse_min_overlap_score`
- coarse, `sac_ia`: `coarse_min_sample_distance`,
  `coarse_max_correspondence_distance`, `coarse_nr_iterations`,
  `coarse_number_of_samples`, `coarse_correspondence_randomness`,
  `coarse_max_fitness_score`
- fine: `fine_max_correspondence_distances` (a list — one GICP pass per entry,
  tightening), `fine_transformation_epsilon`,
  `fine_euclidean_fitness_epsilon`, `fine_rotation_epsilon`,
  `fine_max_iterations`, `fine_correspondence_randomness`,
  `fine_max_fitness_score`
- dedup: `skip_dedup` (default false; true leaves the merged cloud as-is),
  `dedup_leaf_size`, `dedup_min_points_per_voxel`
- orchestration: `stage_timeout_sec`, `discovery_timeout_sec`

`leaf_size` and both timeouts must be greater than 0; `/run_pipeline` rejects
the call before running anything otherwise. `normal_radius` and `fpfh_radius`
are checked the same way when the feature stages actually run
(`coarse_method:=sac_ia`), and `dedup_leaf_size` the same way when the dedup
stage runs (`skip_dedup:=false`, the default); `dedup_min_points_per_voxel`
must be at least 1 whenever it runs.
Source and target must also have different filename stems, since the
per-cloud intermediates are named from the stem alone.

> **One instance of each node.** `pipeline_node` addresses stage nodes by node
> name. A leftover `voxel_node` or `merge_node` from a manual run competes for
> the same name and the same service, and the pipeline may reconfigure one
> process while triggering another. Check with `ros2 node list` (it warns about
> duplicate names) and kill strays before launching.

---

## Option B: stage by stage

Same sequence, run by hand. Each node needs its own terminal (or Ctrl-C and
relaunch between steps — never leave two of the same node running). Paths below
are written out in full; `$P` is `/home/fishman/ros2_ws/src/my_point_reg`.

### Step 1: Voxelize source cloud

```bash
P=/home/fishman/ros2_ws/src/my_point_reg

ros2 run my_point_reg voxel_node --ros-args \
  -p input_pcd_path:="$P/PCD/ground.pcd" \
  -p output_pcd_path:="$P/output_test_file/voxel_ground.pcd" \
  -p leaf_size:=0.1
```
```bash
ros2 service call /voxelize std_srvs/srv/Trigger
```
**Output:** `output_test_file/voxel_ground.pcd`

### Step 2: Voxelize target cloud

```bash
ros2 run my_point_reg voxel_node --ros-args \
  -p input_pcd_path:="$P/PCD/air.pcd" \
  -p output_pcd_path:="$P/output_test_file/voxel_air.pcd" \
  -p leaf_size:=0.1
```
```bash
ros2 service call /voxelize std_srvs/srv/Trigger
```
**Output:** `output_test_file/voxel_air.pcd`

### Steps 3–4: Extract features — `coarse_method:=sac_ia` only

Skip straight to [step 5](#step-5-coarse-registration) if you are using
`yaw_sweep`, which is the default: it scores 2D occupancy overlap on the
voxelized clouds themselves and never reads a normal or a descriptor.

#### Step 3: Extract features from the voxelized source

```bash
ros2 run my_point_reg feature_node --ros-args \
  -p input_pcd_path:="$P/output_test_file/voxel_ground.pcd" \
  -p output_normals_pcd_path:="$P/output_test_file/normal_ground.pcd" \
  -p output_fpfh_pcd_path:="$P/output_test_file/fpfh_ground.pcd"
```
```bash
ros2 service call /estimate_features std_srvs/srv/Trigger
```
**Outputs:** `normal_ground.pcd`, `fpfh_ground.pcd`

#### Step 4: Extract features from the voxelized target

```bash
ros2 run my_point_reg feature_node --ros-args \
  -p input_pcd_path:="$P/output_test_file/voxel_air.pcd" \
  -p output_normals_pcd_path:="$P/output_test_file/normal_air.pcd" \
  -p output_fpfh_pcd_path:="$P/output_test_file/fpfh_air.pcd"
```
```bash
ros2 service call /estimate_features std_srvs/srv/Trigger
```
**Outputs:** `normal_air.pcd`, `fpfh_air.pcd`

### Step 5: Coarse registration

Produces a rough 4×4 transform to seed GICP. Pick one of the two methods —
see [Choosing a coarse method](#choosing-a-coarse-method).

> **Name the outputs after the direction you solved.** A transform is only
> valid for the source/target roles it was solved with, and the file itself
> carries no record of them. Running the stage a second time with the roles
> swapped — an easy thing to do when checking a result — silently overwrites a
> generic `coarse_aligned.txt` with the *inverse* transform, and step 6 will
> consume it without complaint. The names below encode the direction.

#### Method `yaw_sweep` (default)

Takes the two **voxelized** clouds directly; steps 3–4 are not needed.

```bash
ros2 run my_point_reg coarse_registration_node --ros-args \
  -p method:="yaw_sweep" \
  -p source_pcd_path:="$P/output_test_file/voxel_ground.pcd" \
  -p target_pcd_path:="$P/output_test_file/voxel_air.pcd" \
  -p output_aligned_pcd_path:="$P/output_test_file/ground_to_air_coarse.pcd" \
  -p output_transform_path:="$P/output_test_file/ground_to_air_coarse.txt"
```
```bash
ros2 service call /register_coarse std_srvs/srv/Trigger
```

The response reports the pose it settled on and an `overlap_score` in 0–1:

```
-> yaw_sweep (height band [0.600000, 4.000000] kept 467338/1460414 pts
   -> 1183/1974 vote cells at 1.000000 m, 9647 source cells at 0.250000 m;
   360 yaw steps of 1.000000 deg)
-> yaw=-87.030000 deg, t=(5.872000, 46.725000), overlap_score=0.717000
```

**If the height band keeps nothing**, the node fails with the two counts in the
message — `sweep_min_z`/`sweep_max_z` are absolute values in the clouds' own
frame, so they need moving for a scene whose floor is not near z = 0.

#### Method `sac_ia`

Source normals + FPFH against target normals + FPFH; needs steps 3–4 first.

```bash
ros2 run my_point_reg coarse_registration_node --ros-args \
  -p method:="sac_ia" \
  -p source_normals_pcd_path:="$P/output_test_file/normal_ground.pcd" \
  -p source_fpfh_pcd_path:="$P/output_test_file/fpfh_ground.pcd" \
  -p target_normals_pcd_path:="$P/output_test_file/normal_air.pcd" \
  -p target_fpfh_pcd_path:="$P/output_test_file/fpfh_air.pcd" \
  -p output_aligned_pcd_path:="$P/output_test_file/ground_to_air_coarse.pcd" \
  -p output_transform_path:="$P/output_test_file/ground_to_air_coarse.txt"
```
```bash
ros2 service call /register_coarse std_srvs/srv/Trigger
```

**Outputs** (either method): `ground_to_air_coarse.pcd`,
`ground_to_air_coarse.txt` ← initial guess for step 6

#### Sanity-checking a coarse result

Re-running with source and target swapped is a genuinely useful check — two
independent searches should return transforms that are inverses of each other,
and that agreeing is far stronger evidence than either run alone. Just write
the reverse run somewhere else:

```bash
ros2 run my_point_reg coarse_registration_node --ros-args \
  -p method:="yaw_sweep" \
  -p source_pcd_path:="$P/output_test_file/voxel_air.pcd" \
  -p target_pcd_path:="$P/output_test_file/voxel_ground.pcd" \
  -p output_aligned_pcd_path:="$P/output_test_file/air_to_ground_coarse.pcd" \
  -p output_transform_path:="$P/output_test_file/air_to_ground_coarse.txt"
```

The two 4×4s should multiply to the identity. **Do not feed
`air_to_ground_coarse.txt` to a step 6 that registers ground onto air** — a
transform solved in the opposite direction moves the source further from the
target, not closer, and GICP will find no correspondences at all.

Expect the two runs to report *different* `overlap_score`s. The score is the
fraction of **source** points near a target point, so the larger cloud scores
lower against the smaller one purely because more of it lies outside the shared
area. That asymmetry is not a disagreement about the pose.

### Step 6: Fine registration (GICP)

Voxelized source + voxelized target + the coarse transform. **`source_pcd_path`,
`target_pcd_path` and `initial_guess_transform_path` must all describe the same
direction** — here, ground onto air.

```bash
ros2 run my_point_reg fine_registration_node --ros-args \
  -p source_pcd_path:="$P/output_test_file/voxel_ground.pcd" \
  -p target_pcd_path:="$P/output_test_file/voxel_air.pcd" \
  -p initial_guess_transform_path:="$P/output_test_file/ground_to_air_coarse.txt" \
  -p output_aligned_pcd_path:="$P/output_test_file/ground_to_air_fine.pcd" \
  -p output_transform_path:="$P/output_test_file/ground_to_air_fine.txt"
```
```bash
ros2 service call /register_fine std_srvs/srv/Trigger
```
**Outputs:** `ground_to_air_fine.pcd`, `ground_to_air_fine.txt`

The response reports a **`correspondences=<n>/<total>`** count per pass. That
is the number to read first:

- **`correspondences=0`** — the node fails the call. GICP never found the two
  clouds within its gate, so the transform written is your initial guess
  unchanged, not a registration. Nearly always a guess solved in the wrong
  direction; occasionally a first gate too tight for the offset that is left.
- **A count that stays flat while fitness improves** — GICP is polishing a
  small subset. Sound, but the result only constrains the region those points
  cover.

`initial_guess_transform_path` is optional — left empty, GICP starts from
identity, which is only sensible for a pair already known to be near-aligned.
Once MAVROS odometry is available, a relative pose written in the same 4×4
row-major text format can be dropped in here and step 5 skipped entirely (see
`POINTCLOUD_MERGE_PLAN.md`).

### Step 7: Merge clouds

`merge_node` applies `transform_path` to `source_pcd_path` itself, so the two
clouds it takes are the **untransformed** voxelized pair — the same source and
target that went into steps 5 and 6. Handing it an already-aligned cloud
applies the transform a second time.

```bash
ros2 run my_point_reg merge_node --ros-args \
  -p source_pcd_path:="$P/output_test_file/voxel_ground.pcd" \
  -p target_pcd_path:="$P/output_test_file/voxel_air.pcd" \
  -p transform_path:="$P/output_test_file/ground_to_air_fine.txt" \
  -p output_merged_pcd_path:="$P/output_test_file/final_merge.pcd"
```
```bash
ros2 service call /merge std_srvs/srv/Trigger
```
**Output:** `final_merge.pcd` — target plus the transformed source, not yet
deduplicated (that's step 8, next).

A quick check that the merge is sane: its bounding box should be the union of
the target's box and the aligned source's box. If it extends well beyond both
inputs, the transform and the cloud it was applied to disagree.

### Step 8: Deduplicate the merged cloud

Re-runs `pcl::VoxelGrid` (the same filter step 1 uses) over the merge output
to collapse points both scans contributed in the overlapping region.

```bash
ros2 run my_point_reg dedup_node --ros-args \
  -p input_pcd_path:="$P/output_test_file/final_merge.pcd" \
  -p output_pcd_path:="$P/output_test_file/final_merge_deduplicated.pcd" \
  -p leaf_size:=0.1
```
```bash
ros2 service call /deduplicate std_srvs/srv/Trigger
```
**Output:** `final_merge_deduplicated.pcd`

`min_points_per_voxel` (default 1) raises the bar past just collapsing
duplicates — anything above 1 also drops voxels that only ever received a
handful of noise points. `leaf_size` must be greater than 0 and
`min_points_per_voxel` at least 1; the node rejects the call up front
otherwise.

---

## Verify Results

```bash
ls -lh /home/fishman/ros2_ws/src/my_point_reg/output_test_file/

# point count straight out of the PCD header
head -11 /home/fishman/ros2_ws/src/my_point_reg/output_test_file/final_merge.pcd

# the 4x4 transform (row-major, space separated)
cat /home/fishman/ros2_ws/src/my_point_reg/output_test_file/ground_to_air_fine.txt
```

Each stage reports its own score, and the three are **not comparable with each
other**:

| stage | score | direction |
|---|---|---|
| coarse, `yaw_sweep` | `overlap_score`, 0–1 | higher is better |
| coarse, `sac_ia` | `fitness_score`, mean squared correspondence distance | lower is better |
| fine, GICP | `fitness_score`, same units | lower is better |

`converged` is informational only and is almost always `true`: PCL sets it on
SAC-IA's first iteration regardless of quality, and on GICP simply running out
of iterations.

> **GICP fitness does not rank alignments correctly on floor-heavy clouds.**
> A cloud that is mostly ground plane free-slides in XY/yaw until it maximises
> floor-on-floor overlap, and scores *better* for it. On the `ground`/`air`
> pair, an identity-seeded GICP reports fitness **0.514** for a pose that is
> 87° wrong, against **0.366** for the correct one. Judge the result on
> structure — crop the floor out and look at whether walls and racking line up
> — and treat the fitness number as a change detector, not as a verdict.

### Making a bad registration fail the run

Once you know what a good score looks like for your data, set the gates so a
bad registration stops the pipeline instead of silently feeding the next stage
a wrong transform. Note the coarse gate depends on the method:

```bash
# coarse_method:=yaw_sweep — reject when overlap falls BELOW this
ros2 param set /pipeline_node coarse_min_overlap_score 0.5

# coarse_method:=sac_ia — reject when fitness rises ABOVE this
ros2 param set /pipeline_node coarse_max_fitness_score 2.0

ros2 param set /pipeline_node fine_max_fitness_score 0.5
```

All default to `0.0` (disabled) — a meaningful threshold depends on how much
the two clouds really overlap, so do one baseline run first and read the scores
out of the `/run_pipeline` response. Stage by stage (Option B), the same knobs
are `min_overlap_score` / `max_fitness_score` on the registration nodes.

A rejected stage still writes its aligned cloud and transform so you can look
at what went wrong, but the response says `REJECTED` and the pipeline stops
before the next stage consumes them.

To cross-check the transform in Python/Open3D, see the note at the end of
`POINTCLOUD_MERGE_PLAN.md` — the matrix is the same convention Open3D uses, no
conversion needed.

---

## Tuning Parameters

### Voxelization (steps 1–2)
- `leaf_size`: 0.05–0.2 (smaller = more detail, slower; larger = faster, less detail)

### Feature extraction (steps 3–4, `coarse_method:=sac_ia` only)
- `normal_radius`: 0.3–1.0 (search radius for normal estimation)
- `fpfh_radius`: 0.5–2.0 — keep it larger than `normal_radius`
- `orient_normals_up`: leave true. FPFH is built from angles between normals,
  so the two clouds' normals must be oriented the same way for their
  descriptors to be comparable

### Coarse registration, `yaw_sweep` (step 5)
- `sweep_min_z` / `sweep_max_z`: default `0.6` / `4.0`, absolute heights in the
  clouds' own frame. The band must **exclude the floor** — the floor is
  identical everywhere, so it carries no information about yaw or XY while
  swamping every point count. Move the band, don't widen it, for a scene whose
  floor is not near z = 0
- `sweep_vote_cell_size`: default `1.0`. This sets the runtime — the vote pass
  costs O(source cells × target cells) per yaw step and both counts scale with
  scene area over this value squared. Raise it if the sweep is slow, lower it
  only if the scene is small and cluttered
- `sweep_cell_size`: default `0.25`, the resolution the shortlisted candidates
  get re-scored at. Roughly 2–3× `leaf_size` is sensible
- `sweep_yaw_step_deg`: default `1.0` over the full circle. Narrow
  `sweep_yaw_min_deg`/`sweep_yaw_max_deg` when the offset is roughly known —
  cost is linear in the number of steps
- `sweep_refine_candidates`: default `8`. Raise it for a very repetitive scene,
  where several poses one aisle apart come out of the vote pass near-tied

### Coarse registration, `sac_ia` (step 5)
- `min_sample_distance`: 0.02–0.1 (minimum distance between SAC-IA sample points)
- `max_correspondence_distance`: 0.5–2.0 (max distance to match features)
- `nr_iterations`: 100–1000 (more iterations = more robust but slower)

### Fine registration (step 6)
- `max_correspondence_distances`: default `[0.5, 0.3, 0.2]` — one GICP pass per
  entry, each seeded with the previous result. Make the first entry big enough
  to span the offset you expect and the last one roughly the leaf size. A
  single loose value lets a ground-plane-dominated cloud slide in XY/yaw
  while reporting a better fitness score
- `max_iterations`: 30–100 per pass (more = tighter convergence, slower)
- `transformation_epsilon`: 1e-8 (lower = stricter convergence)

### Deduplication (step 8)
- `leaf_size` (`dedup_leaf_size` through `pipeline_node`): 0.05–0.2, same
  role as the voxelization `leaf_size` — collapses points within one voxel
  into one
- `min_points_per_voxel` (`dedup_min_points_per_voxel` through
  `pipeline_node`): default 1 (pure dedup). Raise it to also drop voxels a
  single stray point occupies alone

Through `pipeline_node` these are the prefixed names listed under
[Launch arguments vs parameters](#launch-arguments-vs-parameters).

---

## Next Steps

Plan Step 6 (deduplication) is implemented as `dedup_node` and runs by
default as the pipeline's last step — set `skip_dedup:=true` to leave the
merged cloud as-is.

**Possible follow-ups, not yet done**
- Feed a MAVROS-derived relative pose in as `initial_guess_transform_path`
  for `fine_registration_node` once odometry is available, skipping coarse
  registration entirely (see `POINTCLOUD_MERGE_PLAN.md`)

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Service call fails / hangs | Check the node is running in the other terminal; `ros2 node list` and `ros2 service list` |
| `ros2 node list` warns about duplicate names | A stray node from an earlier run is still alive — `ps -ef \| grep my_point_reg` and kill it, then relaunch |
| Pipeline reports "parameter services of 'X' are not available" | That stage node isn't running; use `ros2 launch my_point_reg pipeline.launch.py` rather than starting nodes by hand |
| Pipeline reports "did not respond within stage_timeout_sec" | The stage is still crunching — raise `stage_timeout_sec`, or voxelize harder with a bigger `leaf_size` |
| Empty/short outputs | Check the input paths; try a smaller `leaf_size` |
| Merged cloud is rotated way off ("backward") | Either the coarse stage produced a garbage transform, or it never ran. With `coarse_method:=sac_ia` on a repetitive scene the answer is usually the 180° flip — switch to `yaw_sweep`. With `skip_coarse:=true` GICP started at identity and cannot recover a large yaw offset at all — set it false |
| Poor alignment | Check the coarse `overlap_score` first — if it is low, the initial guess is the problem and no GICP tuning will fix it. Then tune `fine_max_correspondence_distances` |
| Alignment scores well but looks wrong | A cloud that is mostly ground plane free-slides in XY/yaw and scores *better* for it. Tighten the last entry of `fine_max_correspondence_distances`, and judge on structure rather than fitness — see the warning under [Verify Results](#verify-results) |
| `yaw_sweep` fails with "No points left after the height band" | `sweep_min_z`/`sweep_max_z` are absolute heights in the clouds' frame and the scene's floor isn't near z = 0. Check the clouds' z range and move the band |
| `yaw_sweep` returns a low `overlap_score` | Genuine when the two scans cover mostly different areas — a ground scan and an aerial scan see opposite faces of the same objects, which caps overlap. Compare against the score of a pose you trust before assuming it failed |
| `yaw_sweep` is slow | Raise `sweep_vote_cell_size` (runtime scales with its square), or narrow the yaw range if the offset is roughly known |
| Fine registration fails with "GICP found no correspondences" | The initial guess never brought the clouds within the first gate. Check `initial_guess_transform_path` came from a coarse run with the **same** source/target roles — a transform solved in the opposite direction moves them apart. Failing that, widen the first entry of `max_correspondence_distances` |
| Fine transform is identical to the coarse one | Same cause as above, and now reported rather than silent: GICP returned the guess untouched because it had nothing to fit |
| Merged cloud is much bigger than either input | The transform was applied to a cloud it wasn't solved for — usually an already-aligned cloud passed to `merge_node`, which transforms the source itself |
| GICP not converging | Check the coarse transform is sane before it's used; try a different initial guess, or identity |
| Out of memory | Increase `leaf_size` to voxelize more aggressively |
| Merge fails (file not found) | Verify the transform path exists — the registration step before it may have failed |
| Merged cloud has holes/gaps | Expected where source and target don't overlap |
| Merged cloud has duplicates | Expected from `merge_node` alone — run `dedup_node` (step 8) on it, or leave `skip_dedup:=false` (the default) when using `pipeline_node` |
| Relative output paths land somewhere unexpected | They resolve against the directory you ran `ros2 run`/`ros2 launch` from, not the package |
