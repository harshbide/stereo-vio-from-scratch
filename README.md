# Stereo Visual-Inertial Odometry (VIO) from First Principles

A keyframe-based, fixed-lag **stereo visual-inertial odometry** estimator built from scratch: IMU preintegration, stereo VO frontend, factor graph, manifold Levenberg-Marquardt optimizer, and a sliding-window smoother. Evaluated on the **EuRoC MAV** dataset (`V1_01_easy`).


---

## Overview

The system estimates the full 6-DoF pose, velocity, and IMU biases of a moving rigid body using only stereo images and inertial measurements. No GPS or motion capture is used in estimation (Vicon ground truth is used for evaluation only).

**State (15-D per keyframe):** orientation `R ∈ SO(3)`, velocity `v`, position `p`, gyroscope bias `b_g`, accelerometer bias `b_a`.

Since rotations live on a manifold, all updates use Lie group / Lie algebra operations and the manifold-aware `⊞` (boxplus) operator instead of naive vector addition.

## Key Features

- **IMU preintegration** (Forster et al.) with first-order bias correction, no re-integration on bias updates
- **Stereo VO frontend**: feature detection, epipolar stereo matching and triangulation (metric scale), KLT tracking, PnP + RANSAC relative pose
- **Static initialization**: variance-based static window search, gyro bias estimation, gravity alignment, accel bias along gravity
- **Factor graph** with IMU, visual-odometry, and zero-velocity (ZUPT) factors
- **Manifold Levenberg-Marquardt** with cost-based step acceptance and per-block step limits
- **Fixed-lag sliding-window smoother** for jointly optimizing the last N keyframes, letting bias errors be corrected after the fact
- **Evaluation tools**: Umeyama alignment, SE(3)-aligned ATE, and RPE

## Pipeline

```
for each new keyframe timestamp t_k:
    1. Preintegrate IMU samples since last keyframe
    2. Run stereo VO frontend on the new image pair  -> relative R, t
    3. Check VO / IMU consistency; build factor list
    4. Check stationarity; optionally add a ZUPT factor
    5. Insert new node into the sliding window
    6. Jointly optimize the window (manifold LM)
    7. If window exceeds size limit, freeze + archive the oldest node
```

## Methodology Summary

| Stage | Approach |
| --- | --- |
| IMU model | Gyro = true rate + bias + noise; accel = specific force (gravity-compensated, body frame) + bias + noise |
| Preintegration | Relative motion (ΔR, Δv, Δp) integrated once between keyframes, Jacobians accumulated for bias correction |
| Visual frontend | Shi-Tomasi/FAST corners, stereo triangulation, KLT tracking, PnP-RANSAC + LM refinement |
| Initialization | Static window scored on gyro variance, accel variance, and `‖a‖ ≈ 9.81 m/s²` |
| IMU factor | 15-D residual (rotation, velocity, position, bias random walk) |
| VO factor | Relative pose residual, weights inflated when PnP inlier count is low |
| ZUPT factor | Zero-velocity constraint when stationarity is detected; the only absolute constraint, it bounds long-horizon drift |
| Optimizer | Gauss-Newton normal equations + adaptive LM damping on the manifold |
| Smoother | Fixed-lag window, oldest node anchored, numerical Jacobian for the older node |

## Dataset

[EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets), sequence **V1_01_easy**: stereo camera pair (cam0, cam1), ~200 Hz IMU, Vicon ground truth (evaluation only).

## Results

SE(3)-aligned ATE and relative position error across the debugging and tuning progression:

| Configuration | SE(3)-Aligned RMSE | Relative Pos. RMSE | Notes |
| --- | --- | --- | --- |
| Original / unfixed | 643.70 m | 20.50 m | Divergent baseline |
| + static-window search & ZUPT (w=5, 60 s) | 0.156 m | 0.048 m | First healthy result |
| + static-window search & ZUPT (w=5, 143 s) | 0.680 m | 0.098 m | Full sequence |
| Sliding window, size 8 (60 s) | 0.064 m | 0.023 m | Window-reach ablation |
| Sliding window, size 8 (143 s, full) | **0.280 m** | **0.050 m** | **Best full-sequence result** |
| Sliding window, size 12 (120 s) | 0.156 m | 0.028 m | Not yet run full-length |

The ~1000x improvement came from two root-cause fixes (an over-permissive outlier gate in the VO frontend, and cost-based step acceptance in the optimizer), followed by giving the smoother more temporal reach. Larger windows cost roughly cubic compute per keyframe, so accuracy gains must be weighed against runtime.

## Development Methodology

- **Symptom characterization**: error-vs-time curve shapes used to classify failure modes before touching code
- **Synthetic tests**: each fix validated on hand-built scenarios reproducing the failure signature
- **Ablations**: window size and factor inclusion varied one at a time on fixed time slices
- **Short vs. long horizon validation**: 60 s and full 143 s runs, since several bugs only appeared over long integration

## Repository Structure

> Adjust to match your actual layout.

```
.
├── README.md
├── docs/
│   └── VIO_Methodology_and_Algorithms.docx
├── src/
│   ├── imu_preintegration/
│   ├── vo_frontend/
│   ├── initialization/
│   ├── factors/
│   ├── optimizer/
│   └── smoother/
├── eval/
│   └── ate_rpe.py
├── tests/
└── results/
```

## Getting Started

```bash
git clone https://github.com/<your-username>/<repo-name>.git
cd <repo-name>
pip install -r requirements.txt

# Download EuRoC V1_01_easy and set the dataset path
python run_vio.py --dataset /path/to/V1_01_easy --window 8
```

## Limitations and Future Work

- Sliding-window marginalization is simplified: the oldest node acts as a fixed anchor instead of a full marginalization prior
- The block-tridiagonal system is currently solved densely
- Loop closure and global bundle adjustment are not included (VIO only)
- Window size 12 has not yet been evaluated on the full sequence
- Only `V1_01_easy` has been evaluated so far

## References

- Forster et al., *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry*, IEEE T-RO, 2017
- Burri et al., *The EuRoC Micro Aerial Vehicle Datasets*, IJRR, 2016
- Umeyama, *Least-squares estimation of transformation parameters between two point patterns*, IEEE TPAMI, 1991

## Author

**Harsh**, Dept. of Electronics and Telecommunication Engineering, Vishwakarma Institute of Technology, Pune

## License

MIT (or your preferred license)
