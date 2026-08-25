# Plan: Merging Two Point Clouds of the Same Map (PCL, C++, offline)

**Status: Steps 1–6 implemented.** Each step below is marked. The
package is `my_point_reg` (this directory) — one node per step, plus
`pipeline_node` running them in sequence. See [`README.md`](README.md) for the
nodes and [`MERGE_WORKFLOW.md`](MERGE_WORKFLOW.md) for how to run them.

Scope assumptions (confirm/update if these change):
- Two static `.pcd` files, merged once (offline post-processing, not live SLAM).
- No prior relative pose between the two clouds — treat as **fully unknown**.
- Target implementation: PCL C++. This document was originally written in
  `/home/fishman/Documents/internship/icp/pcl_tutorial` as a **reference map**,
  so the `lecXX_*.cpp` links below point at examples in *that* repo (copies of
  which are in this package's gitignored `code_reference/`), not at anything
  built here.
- Once MAVROS odometry is available in the real project, the coarse/global
  registration step (Step 3) can usually be skipped — use the odometry-derived
  relative pose directly as the initial guess for Step 4 (ICP/GICP), and only
  fall back to global registration when odometry is missing or has drifted
  too far to trust.
- I have run `/graphify .` in `/home/fishman/Documents/internship/icp` do whatever with that information. (I haven't learned how to use it yet.)

---

## Background

**Global / coarse registration** — the step that estimates an *approximate*
relative transform between two clouds when there is no prior pose guess.
ICP/GICP are local optimizers: they only converge to the correct alignment if
they start close to it. With two clouds in arbitrary poses, ICP run directly
will typically converge to the wrong local minimum. Coarse registration finds
a rough transform good enough to seed ICP/GICP, via feature matching
(FPFH + RANSAC / SAC-IA), geometric congruent-set matching (4PCS / Super4PCS),
or learned matchers (e.g. KISS-Matcher).

**GICP (Generalized ICP, Segal et al. 2009)** — an extension of ICP that
models each point's local surface shape as a covariance (from its neighbors)
instead of treating points as exact locations. It generalizes point-to-point
and point-to-plane ICP into a single plane-to-plane formulation using both
clouds' local covariances, giving more robust convergence on structured,
plane-rich data (e.g. LiDAR). It is still a **local/fine** method — it needs
a reasonable initial guess, same as plain ICP.

---

## Pipeline

### Step 1. Load + preprocess each cloud — implemented (`voxel_node`)
- `pcl::io::loadPCDFile` for each input.
- Downsample: `pcl::VoxelGrid` — reference: [`lec05_voxelization.cpp`](lec05_voxelization.cpp).
- Remove outliers: `pcl::StatisticalOutlierRemoval` — reference: [`lec07_sor.cpp`](lec07_sor.cpp).

### Step 2. Estimate normals + FPFH features — implemented (`feature_node`)
- Normals: `pcl::NormalEstimation`/`NormalEstimationOMP` — reference:
  [`lec10_1_normal.cpp`](lec10_1_normal.cpp), [`lec10_2_normal_corner.cpp`](lec10_2_normal_corner.cpp).
- FPFH: `pcl::FPFHEstimationOMP` on top of the normals.
  **No reference in this repo** — not yet demonstrated here (see gap below).

### Step 3. Coarse / global registration — implemented (`coarse_registration_node`, SAC-IA)
Options:
- `pcl::SampleConsensusInitialAlignment` (SAC-IA): FPFH + RANSAC.
- `pcl::registration::FPCSInitialAlignment` (4PCS): correspondence-free,
  more robust to repetitive/symmetric geometry, slower.
- KISS-Matcher: this repo's chapter 14 demo (`web/`) uses it, but only via a
  **precomputed Python path** (`tools/gen_kiss_matcher_data.py`, using the
  `kiss-matcher` pip package) for the browser visualization — there is no
  C++ PCL example of it here.

> **Gap:** this repo currently has no C++ reference for Step 2 (FPFH) or
> Step 3 (SAC-IA / 4PCS / KISS-Matcher). This will need new code, either
> written directly against PCL's registration module docs, or by linking
> against the KISS-Matcher C++ library if going that route.

### Step 4. Fine registration — implemented (`fine_registration_node`, GICP)
- `pcl::GeneralizedIterativeClosestPoint` (GICP), seeded with the Step 3
  transform as its initial guess — reference: [`lec12_gicp.cpp`](lec12_gicp.cpp).
- Plain `pcl::IterativeClosestPoint` reference also available:
  [`lec11_icp.cpp`](lec11_icp.cpp) (fall back to this only if GICP
  underperforms on the actual data).
- Check `hasConverged()` and `getFitnessScore()` before trusting the result.

### Step 5. Transform + merge — implemented (`merge_node`)
- `pcl::transformPointCloud` with `gicp.getFinalTransformation()`.
- Concatenate: `merged = cloudA + cloudB_aligned`.

### Step 6. Deduplicate merged cloud — implemented (`dedup_node`)
- Re-run `pcl::VoxelGrid` (Step 1's filter) on the merged cloud to collapse
  overlapping points from both scans.

---

## Cross-checking a result in Python (Open3D)

`getFinalTransformation()` returns an `Eigen::Matrix4f` — a standard 4×4
homogeneous transform (`p' = T · p`), same convention Open3D uses.

- C++: dump the matrix to a text file (4 rows × 4 floats).
- Python:
  ```python
  import numpy as np, open3d as o3d
  T = np.loadtxt("transform.txt")
  pcd = o3d.io.read_point_cloud("cloudB.pcd")
  pcd.transform(T)
  ```
No conversion needed, as long as both clouds are already in the same axis/
unit convention (meters, right-handed) — no ROS↔camera-frame flips.
