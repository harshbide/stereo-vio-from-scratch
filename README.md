# GTSAM-free Visual-Inertial Odometry (EuRoC MAV)

A from-scratch Python implementation of the pipeline in your diagram:

```
Camera -> Feature tracking -> Visual odometry  \
                                                  >-- Jacobian blocks -> Block accumulation (J^T W J) -> Solve H*dx=b -> Update pose -> repeat
IMU data -> IMU factor computation (preintegration) /
```

No GTSAM anywhere — the factor-graph math (SO(3) Lie algebra, IMU
preintegration, analytic Jacobians, Gauss-Newton normal equations) is
implemented directly with numpy/scipy. Visual odometry uses OpenCV
(stereo triangulation + optical flow + PnP).

Architecture note: at every keyframe, only the **new** keyframe's state
is free (15-dim: rotation, velocity, position, gyro bias, accel bias);
the previous keyframe's state is treated as fixed. That's what keeps
the "block accumulation -> solve" step a genuinely small 15x15 linear
solve every time, instead of a growing bundle-adjustment problem — it's
an incremental/marginalized (filter-like) formulation, well suited to
the FPGA/ARM split your diagram describes.

## Project layout

```
vio_project/
  main.py                  CLI entry point
  requirements.txt
  src/
    lie.py                 SO(3) Exp/Log/right-Jacobian
    state.py                15-dim NavState + manifold boxplus update
    imu_preintegration.py   on-manifold IMU preintegration + bias Jacobians
    dataset.py               EuRoC mav0/ loader (cam0/cam1/imu0/groundtruth)
    vo_frontend.py            stereo feature tracking -> metric relative pose
    factors.py                IMU factor + VO factor residuals & Jacobians
    optimizer.py              Gauss-Newton: J^T W J accumulation + solve
    pipeline.py               orchestrates the full per-keyframe loop
```

## 1. Set up the environment (PyCharm)

1. Open this folder (`vio_project/`) as a new PyCharm project.
2. Create a virtualenv interpreter (PyCharm: *Settings -> Project ->
   Python Interpreter -> Add -> Virtualenv*), Python 3.10+.
3. In PyCharm's terminal:
   ```bash
   pip install -r requirements.txt
   ```

## 2. Get the EuRoC V1_01_easy sequence

Download the zip from the ASL EuRoC MAV page (the **ASL Dataset Format**,
not the ROS bag) and extract it so you end up with:

```
V1_01_easy/
  mav0/
    cam0/  {data.csv, data/*.png, sensor.yaml}
    cam1/  {data.csv, data/*.png, sensor.yaml}
    imu0/  {data.csv, sensor.yaml}
    state_groundtruth_estimate0/data.csv
    ...
```

`dataset.py` expects exactly this layout — point `--dataset` at the
folder that directly **contains** `mav0/` (i.e. the `V1_01_easy` folder
itself, however you named it after unzipping). No editing of the zip's
internal structure is needed; just unzip it as-is.

## 3. Run it

From the project root, as a PyCharm Run Configuration or from the
terminal:

```bash
# quick smoke test on the first 100 keyframes, with a live plot at the end
python main.py --dataset /path/to/V1_01_easy --stride 2 --max-keyframes 100 --plot --verbose

# full sequence
python main.py --dataset /path/to/V1_01_easy --stride 2 --out trajectory.csv --plot
```

- `--stride N` uses every Nth cam0 frame as a keyframe (larger N = fewer,
  more-spaced-out keyframes, faster but less accurate VO tracking).
- Output is a CSV with one row per keyframe: `t, px,py,pz, vx,vy,vz,
  qw,qx,qy,qz, bgx,bgy,bgz, bax,bay,baz`.
- `--plot` overlays the estimated trajectory against groundtruth
  (top-down x/y).

## 4. What's implemented vs. simplified (read this before trusting numbers)

This is a working scaffold that has been **unit- and integration-tested**
(see `tests/` — Jacobians checked against finite differences, IMU
preintegration checked against closed-form stationary/constant-rate
cases, stereo geometry checked against synthetic ground truth, and the
whole pipeline smoke-tested on a synthetic dataset). It is *not* a
drop-in replacement for a mature system like OKVIS/VINS-Mono/Kimera —
notably:

- **VO frontend** re-triangulates a fresh stereo point cloud at every
  keyframe and tracks it exactly one keyframe forward with KLT, so it
  discards long-track feature history. This keeps it simple but throws
  away information a real system would keep (longer tracks reduce
  drift). Good next step if you want to improve accuracy.
- **No loop closure, no global bundle adjustment.** This is a pure
  odometry front-to-back filter, so it *will* drift over a long
  sequence (as does any odometry-only VIO).
- **Noise parameters** in `imu_preintegration.py` (`gyro_noise_density`
  etc.) are EuRoC-typical MPU-9250 datasheet numbers — tune if your
  results look mis-weighted between VO and IMU.
- **No outlier rejection beyond PnP RANSAC** — no dedicated failure
  detector for tracking loss on strongly rotating/aggressive motion
  segments (e.g. V2_03_difficult); start with V1_01_easy as you planned.
- Initial state uses groundtruth pose if `state_groundtruth_estimate0`
  is present (fine for evaluating estimator accuracy in isolation); set
  it to identity/zero yourself if you want a "no groundtruth available"
  test.

## 5. Suggested next steps

1. Run on `V1_01_easy` with `--max-keyframes 100` first to sanity check
   before a full run.
2. Plot position error vs. groundtruth (ATE) — a small evaluation script
   would compare `trajectory.csv` against
   `mav0/state_groundtruth_estimate0/data.csv`.
3. Tune `keyframe_stride` and the VO/IMU factor weights
   (`rot_sigma`/`trans_sigma` in `factors.vo_factor`,
   `*_noise_density`/`*_bias_rw` in `ImuPreintegrator`) once you see
   real drift numbers.
4. If accuracy matters more than embedded-realism, the natural upgrade
   is a real sliding window (N keyframes free at once, marginalizing
   the oldest) instead of the 2-node scheme here — bigger H matrix, but
   still no GTSAM required.
