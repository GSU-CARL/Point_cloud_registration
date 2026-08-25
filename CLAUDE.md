# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`my_point_reg` is an `ament_cmake` ROS 2 (Jazzy) package implementing a
point-cloud registration/merge pipeline for the UAV LiDAR mapping project this
workspace belongs to (see `/home/fishman/ros2_ws/src/CLAUDE.md` for the
workspace-wide picture and hardware-safety rules — this package is pure
file-in/file-out `.pcd` processing, no topics, no actuation, so most of that
document doesn't apply here, but read it once for context).

The implementation plan lives in [`POINTCLOUD_MERGE_PLAN.md`](POINTCLOUD_MERGE_PLAN.md):
load/preprocess → normals+FPFH → coarse/global registration → GICP fine
registration → transform+merge → dedup. **All six plan steps are implemented**
(`voxel_node`, `feature_node`, `coarse_registration_node`,
`fine_registration_node`, `merge_node`, `dedup_node`), plus `pipeline_node`,
which drives all six in order. Step 6 (`dedup_node`, re-running `VoxelGrid`
over the merged cloud to collapse points both scans contributed in the
overlapping region) was deliberately deferred until the merge misalignment was
sorted out, since deduplicating a badly aligned merge just bakes in the error.
**That alignment problem is now solved** (see
[Alignment: solved](#alignment-solved-2026-07-31-yaw_sweep)), which is what
unblocked Step 6. Read that section before touching the registration path —
it records which metrics lie and why.

Earlier `transform_node`/`icp_node`/`pipeline_node` prototypes existed at one
point and were deliberately deleted (dead ends, not worth reconciling with) —
don't be surprised if old references to them turn up in git history or stray
notes. The current `src/pipeline_node.cpp` is a fresh orchestrator written
against the plan, unrelated to the deleted prototype of the same name; don't
resurrect the old one to "restore" anything.

[`MERGE_WORKFLOW.md`](MERGE_WORKFLOW.md) is the end-to-end runbook (both the
one-shot pipeline and the stage-by-stage manual sequence);
[`README.md`](README.md) documents each node and its parameters.

## Build & test

Build from the **workspace root**, not from inside this package:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_point_reg \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The `Release` flag is not optional in practice — `VoxelGrid`, `StatisticalOutlierRemoval`,
and FPFH estimation on the multi-million-point clouds in `PCD/` are unreasonably
slow in a debug build.

```bash
colcon test --packages-select my_point_reg
colcon test-result --verbose
```

`colcon test` runs `ament_lint_auto` (uncrustify, cpplint disabled, copyright
disabled, cppcheck, lint_cmake, xmllint, plus flake8/pep257 over
`launch/*.py`). The vendored files under
`code_reference/` are expected to fail `ament_uncrustify` — they're copied
tutorial reference material, not this package's code; don't "fix" their style.

## Architecture

Every **stage node** in this package follows the same shape — read
`src/voxel_node.cpp` first as the canonical example before adding another one.
`pipeline_node` is the one exception and is described separately below.

- **One node per pipeline stage**, named after the plan step it implements
  (`voxel_node` = Step 1, `feature_node` = Step 2, ...). A stage may offer more
  than one algorithm behind a `method` parameter rather than becoming two nodes
  — `coarse_registration_node` does this with `yaw_sweep` (default) and
  `sac_ia`. Where the methods need different inputs, validate and load only
  what the selected one uses.
- **File in, file out.** No cloud topics are published or subscribed. A node
  does nothing at startup beyond declaring parameters and advertising a
  service — all work happens inside a single `std_srvs/srv/Trigger` service
  callback, so a human decides exactly when it runs. Service names are
  **relative and unnamespaced**: `voxel_node` → `/voxelize`, `feature_node` →
  `/estimate_features`, `coarse_registration_node` → `/register_coarse`,
  `fine_registration_node` → `/register_fine`, `merge_node` → `/merge`,
  `dedup_node` → `/deduplicate`, `pipeline_node` → `/run_pipeline`. Point
  counts, radii/leaf sizes, fitness
  scores, and output paths are reported back in the Trigger response
  `message`
  (`std::ostringstream` with `std::fixed << std::setprecision(6)`), not logged.
- **Every parameter has a default**, declared up front in the constructor —
  no magic numbers in the callback body.
- **Point type convention: `pcl::PointXYZI`.** All nodes load with
  `pcl::io::loadPCDFile<pcl::PointXYZI>`. Any `normal_x/normal_y/normal_z/curvature`
  fields present in a source `.pcd` are silently dropped on load — if normals
  are needed downstream, `feature_node` re-estimates them from point positions
  rather than any node trying to preserve stored ones.
- **Fail-fast validation, not exceptions.** Empty/missing required params, a
  failed `loadPCDFile`, or an empty cloud all set `response->success = false`
  with an explanatory message and return early — they don't throw.
- **Default output paths.** Any `output_*_pcd_path` parameter left empty is
  resolved at call time to `output_test_file/<input_stem>_<suffix>.pcd`
  (directory auto-created via `std::filesystem::create_directories`,
  relative to wherever the node process was launched from). The
  `kDefaultOutputDir = "output_test_file"` constant is duplicated per node
  file rather than shared — keep that pattern if you add a node rather than
  introducing a shared header for one constant.
- **Source is aligned onto target**, consistently, in every stage that takes
  both: the registration nodes solve for the transform that brings `source_*`
  into the target's frame, and `merge_node` writes `target + transformed
  source`. Don't flip the roles in one node.
- **4×4 transforms are plain text**, space-separated, row-major, written by
  `coarse_registration_node`/`fine_registration_node` and read back by
  `fine_registration_node`/`merge_node` through a small local `readTransform`
  helper. That helper is duplicated per node file, same as `kDefaultOutputDir`
  — keep it that way.
- Adding a node means: new `src/<name>_node.cpp`, a matching `add_executable`
  block in `CMakeLists.txt` (mirror an existing one — include dirs, compile
  features, `ament_target_dependencies`, `target_link_libraries` against
  `${PCL_LIBRARIES}`), the target added to the single `install(TARGETS ...)`
  line, and any new PCL component appended to the one
  `find_package(PCL REQUIRED COMPONENTS ...)` line (don't add a second
  `find_package(PCL ...)` call). If it's a new pipeline stage, also wire it
  into `pipeline_node` and `launch/pipeline.launch.py`.

### `pipeline_node` — the orchestrator

`src/pipeline_node.cpp` is deliberately the odd one out: it includes no PCL
headers and touches no point data. It owns the *sequence* and every
intermediate filename, and for each stage it pushes that stage node's
parameters over the parameter service (`rclcpp::AsyncParametersClient`), calls
that node's Trigger service, waits, and appends the stage's own response
`message` to a running log returned by `/run_pipeline`. A stage failure stops
the run and returns the log so far plus `FAILED at <stage>: <message>` —
partial progress stays diagnosable from the response alone.

Things to preserve if you touch it:

- **Two callback groups, `MultiThreadedExecutor` with ≥2 threads.** The
  `/run_pipeline` service callback blocks on futures that only complete when
  another executor thread services the client responses. One callback group,
  or a single-threaded executor, deadlocks. `main()` constructs the executor
  explicitly for this reason.
- **The stage nodes must be singletons.** Reconfiguring a node by name and
  then calling its service is ambiguous if two processes share a node name —
  a leftover manually-launched `voxel_node` will silently compete. Launch the
  stack via `launch/pipeline.launch.py`, which starts exactly one of each.
- **Stage nodes are launched bare.** Any parameter set on them in the launch
  file is overwritten by `pipeline_node` before each call, so all tuning lives
  on `pipeline_node`'s own parameters. Where a name collides across stages
  it's prefixed (`coarse_max_correspondence_distance` vs
  `fine_max_correspondence_distance`).
- **It sets intermediate paths explicitly** rather than relying on each stage
  node's empty-parameter default, so a run stays self-consistent when
  `output_dir` isn't the default.

## Alignment: solved 2026-07-31 (`yaw_sweep`)

**The merge misalignment is fixed.** Confirmed visually in CloudCompare on
`PCD/ground.pcd` → `PCD/air.pcd`. This section records the diagnosis because
the wrong conclusion was live in this file for a while and cost real time.

### What was actually wrong

**The two clouds were 87° of yaw and 47 m apart, and nothing in the pipeline
was searching for that.** `skip_coarse` defaulted to true, so GICP started from
identity — and GICP refines, it cannot search. No correspondence-distance
schedule recovers an 87° error, so every run failed the same way.

The correct transform, ground → air, is a pure planar offset:

```
yaw = −86.75°   pitch ≈ 0°   roll ≈ 0°   t = (5.75, 46.75, 0)
```

Roll, pitch and z all came out at essentially zero — both scans are already
gravity-aligned with floors at z ≈ 0. **The residual offset is genuinely
3-DOF**, which is what makes an exhaustive (x, y, yaw) search both cheap and
immune to local minima. That observation is the whole basis of the fix.

### Two beliefs previously recorded here that were false

Both were written against the *older* `ground.pcd`/`air.pcd` (3.86M/2.55M
points). Those files were replaced on 2026-07-31 with a different pair
(928K/1.73M points, normals baked in) and the conclusions did not survive:

- ~~"The two clouds arrive already coarsely aligned in a shared world frame …
  a refinement problem, not a global-registration problem."~~ **False.** At
  yaw 0 the best available translation is still a 29 m shift. They were never
  coarsely aligned.
- ~~`skip_coarse` should default to true.~~ **Inverted.** It now defaults to
  false. Identity is only a safe initial guess when it is already nearly the
  answer.

**Lesson worth keeping: re-derive conclusions when the input data is
replaced.** The point counts in a note are the cheapest way to spot that the
data underneath it has changed.

### Why FPFH/SAC-IA could not fix it

Re-measured on the current pair with `feature_node`'s own parameters
(normal radius 0.5, FPFH radius 1.0, normals oriented up), three trials with
the floor kept and three with it removed: **6 of 6 wrong**, rotation errors
54–180°, translation errors 23–48 m. Removing the floor did not help.

The 176–180° errors are the tell. The scene is a warehouse of near-identical
rack rows, and FPFH describes roughly a 1 m neighbourhood — it cannot
distinguish one aisle from another, or a rack's front face from its back.
RANSAC then has thousands of equally self-consistent correspondence sets and
picks a wrong one. This is structural, not a tuning problem. Compounding it,
a ground scan and an aerial scan see opposite faces of the same objects, so
genuine correspondences are scarce even in principle.

`feature_node` and the `sac_ia` method are still in the tree and still
correct — they are simply the wrong tool for a gravity-aligned repetitive
scene. Note that `feature_node`'s output feeds **only** `coarse_registration_node`;
GICP computes its own covariances and never reads those normals.

### Scores lie here — three different ways

1. **GICP fitness ranks alignments backwards on floor-heavy clouds.** A cloud
   that is mostly ground plane free-slides in XY/yaw until it maximises
   floor-on-floor overlap, and scores *better* for it. Identity-seeded GICP
   reports fitness **0.514** for a pose that is 87° wrong, against **0.366**
   for the correct one.
2. **SAC-IA fitness did the same.** Its highest-scoring run (0.452) was 54°
   and 34 m off.
3. **`overlap_score` is asymmetric by construction.** It is the fraction of
   *source* points near a target point, so running a pair in both directions
   gives two different numbers for the same pose — the larger cloud scores
   lower because more of it lies outside the shared area. Not a disagreement.

**Judge alignment on structure**: crop the floor out (`0.6 < z < 4.0` works
here) and measure how much of the source lands near the target. On that
metric the correct answer scores 0.732 within 0.5 m (median 0.29 m) against
0.036 (median 2.27 m) at identity.

### What changed in the code

- **`coarse_registration_node` gained a `method` parameter**, default
  `yaw_sweep`; `sac_ia` is unchanged and still selectable. The sweep runs two
  passes: a **vote** pass that, for each yaw step, accumulates a histogram over
  the translations implied by every (source cell, target cell) pair and takes
  its peak; then a **refine** pass that re-scores the top
  `sweep_refine_candidates` poses at a finer cell size. Scoring is on occupied
  *cells*, not raw points, so a densely-sampled surface cannot outvote the rest
  of the scene. Gate parameter is `min_overlap_score` (higher is better) —
  distinct from `sac_ia`'s `max_fitness_score` (lower is better).
- **`pipeline_node` gained `coarse_method`; `skip_coarse` now defaults to
  false.** Step count adapts: 5 for `yaw_sweep` (feature stages drop out),
  7 for `sac_ia`, 4 for `skip_coarse` — plus one more for the dedup stage
  (`skip_dedup`, default false) added afterward.
- **`fine_registration_node` now counts and reports correspondences per pass,
  and fails when the final count is zero.** See the next section for why.
- `launch/pipeline.launch.py` exposes `coarse_method` and `skip_coarse`.

### A transform file does not record its own direction

Worth knowing before writing another stage. Running the coarse stage a second
time with source and target swapped is a genuinely good check — two
independent searches should return mutually inverse transforms, and on this
pair they did, agreeing to within the 0.25 m refine cell. But the two runs
were writing to the same `coarse_aligned.txt`, so the reverse run silently
overwrote the forward one, and the fine stage consumed the inverse.

The result: GICP found **0 correspondences at every gate** (against 24093 at
0.5 m for the correct guess), so it returned the initial guess untouched — and
reported `converged=true` with a plausible fitness score. The merged cloud came
out with a bounding box reaching x = 67 when neither input goes past 49.

PCL exposes nothing that distinguishes "GICP fitted the clouds" from "GICP had
nothing to fit". Hence the correspondence count in `fine_registration_node`,
and the direction-encoded filenames throughout `MERGE_WORKFLOW.md`
(`ground_to_air_coarse.txt`, not `coarse_aligned.txt`).

## Open problems (2026-07-31)

### Stage nodes die with SIGBUS after a successful write

`merge_node` and (in a separate run) `voxel_node` have both exited with
**exit code −7 (SIGBUS)**. The mechanism is **not understood**. What is known:

- The node returns `success` and writes a **complete, valid** output file
  first, then dies ~1 s later. Verified for the merge: 332,384 points =
  222,893 target + 109,491 source, all finite. Output is not corrupted, and the
  pipeline logs the stage as done before the process death is reported.
  (Those counts are from the *older* `ground.pcd`/`air.pcd` — see the note on
  the data swap above. The crash has not been reproduced since.)
- Both crashing nodes call PCL's `savePCDFileBinary`, which writes via `mmap` —
  the one path in this code that can raise SIGBUS rather than return an error.
- Not a resource exhaustion: disk had 249 G free, `/dev/shm` 7.4 G free, RAM
  8.8 G available. Swap was tight (266 Mi of 2 Gi free).
- Not a rebuild-under-a-running-process: the binaries predated the launch.
- No core dump was captured (`core_pattern` is apport; nothing landed in
  `/var/crash`). `/dev/shm` had ~81 stale `fastrtps_*` segments orphaned from
  crashed runs.
- **Two full stacks were running concurrently when the merge crash happened** —
  a leftover launch plus a fresh one, i.e. two of every node and two `/merge`
  services. That is the singleton violation described under `pipeline_node`
  above, it makes stage timings and "timed out setting parameters" errors from
  those runs untrustworthy, and it is a plausible but **unproven** route to two
  processes writing the same output path through `mmap` at once.

**Next step is a clean single-stack rerun** (`ros2 node list` first, kill any
duplicate before triggering). If SIGBUS still happens with exactly one of each
node, it is a real bug in the save path worth chasing; if it doesn't, it was
the duplicate-node collision.

Still unproven either way. Stray stage nodes from earlier manual runs do
survive across sessions — one was found alive during the `yaw_sweep` work — so
check `ros2 node list` and `pgrep -af my_point_reg/lib` before trusting any
timing or crash observation.

## Reference material (not part of the build)

- `code_reference/` (gitignored) — plain PCL API usage examples (voxelization,
  ICP, etc.) copied from an external tutorial repo
  (`/home/fishman/Documents/internship/icp/pcl_tutorial`). These are
  standalone demo `.cpp` files with their own `main()`/PCL-visualizer calls —
  useful only for the filter/estimator API calls (e.g. `setLeafSize`,
  `setMeanK`), never for ROS 2 node structure. They are not compiled by
  `CMakeLists.txt`.
- `PCD/` (gitignored) — real sample scans used for manual testing:
  `full_scan_high.pcd` (6.63M points), `notfull_scan_low.pcd` (5.02M),
  `ground.pcd` (928K) and `air.pcd` (1.73M), plus earlier captures
  `ground1`/`ground2`/`air1`/`air2`. The `ground`/`air` pair is what the merge
  workflow is currently being exercised on. Always voxelize before running
  anything expensive on these.

  **These files get replaced in place.** The current `ground.pcd`/`air.pcd`
  are not the ones the earlier notes were written against, and they already
  carry `normal_x/normal_y/normal_z/curvature` fields (dropped on load — see
  the point type convention above). Check the point counts against whatever
  a note claims before trusting a conclusion recorded about "the ground/air
  pair"; the swap on 2026-07-31 silently invalidated several.
- `output_test_file/` (gitignored) — default landing spot for node outputs
  when an output path param is left unset; not committed, safe to clear.
