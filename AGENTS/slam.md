# SLAM — Agent Guide

Agent and contributor guide for adding pure-Python 2D LiDAR+IMU SLAM to the rover stack.

## What We Have (don't re-implement)

| Already exists | Location |
|---|---|
| JSON serial pipeline (`scan`, `imu`, `status`) | `rover_stack/py/controller_teleop.py` — `ControllerTeleopBridge` |
| Thread-safe `ScanSnapshot` (x_m, y_m, distance_m, intensity, yaw_deg, yaw_rate_dps) | same file |
| `LD06Viewer.update_pose(x, y, theta_rad)` + `get_latest_scan()` SLAM hooks | `py_scripts/sensor_viewers/ld06_viewer/viewer.py:96` |
| World-frame map accumulation driven by pose | `ld06_viewer/viewer.py:152` |
| `MadgwickFilter` (6-DoF gyro+accel → quaternion) | `py_scripts/sensor_viewers/vl53l5cx_viewer/imu_fusion.py` |

The bot's BNO085 already runs onboard sensor fusion and emits `yaw_deg` / `yaw_rate_dps` in the `imu` JSON packet — the controller bridge parses these directly. No host-side Madgwick needed for heading; that's done in firmware.

## Package Layout

SLAM lives at the repo root as a standalone package, imported by both the rover
stack and the simulator:

```
slam/                           ← standalone pure-Python SLAM package
├── __init__.py
├── icp.py                      ← scan preprocessing + point-to-point ICP
├── occupancy_grid.py           ← log-odds grid with Bresenham ray-casting
└── slam_thread.py              ← orchestration thread; duck-typed bridge interface

slam_sim/                       ← hardware-free simulator
├── __init__.py
└── sim.py                      ← synthetic world, scripted robot, viser display

rover_stack/py/
└── controller_teleop.py        ← adds sys.path + imports SlamThread when --slam passed
```

Both `slam_sim/sim.py` and `controller_teleop.py` add the repo root to `sys.path`
and then `from slam.slam_thread import SlamThread`. No installation needed.

## Algorithm: IMU-Assisted ICP SLAM

### Why this stack

- **No external dependencies beyond numpy/scipy** — fits the pure-Python requirement.
- The BNO085 gives accurate yaw (±1°), so the hardest ICP failure mode (large rotation between scans) is eliminated by using IMU heading as a prior.
- ICP handles translation estimation where the IMU gives nothing useful.

### Data flow

```
Bot firmware
  └─ BNO085 (yaw_deg, yaw_rate_dps)
  └─ LD06 (x_m[], y_m[])
        ↓ ESP-NOW
Controller firmware
        ↓ USB JSON lines
ControllerTeleopBridge.get_snapshot()
        ↓ ScanSnapshot (thread-safe copy)
SlamThread (background)
  1. IMU prediction  → delta_theta from yaw_deg diff
  2. ICP correction  → delta_x, delta_y
  3. Pose update     → (x, y, theta)
  4. viewer.update_pose(x, y, theta)   ← drives viser map display
  5. grid.insert(scan, pose)           ← occupancy grid update
```

### Step-by-step

#### Step 1 — IMU heading prediction (`slam_thread.py`)

```python
delta_theta = math.radians(new_yaw_deg - prev_yaw_deg)
# wrap to [-pi, pi]
delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi
predicted_theta = prev_theta + delta_theta
```

Use `yaw_deg` directly from `ScanSnapshot.yaw_deg` — no extra filtering needed on the host.

#### Step 2 — Scan pre-processing (`icp.py`)

```python
def scan_to_points(snapshot: ScanSnapshot, max_dist_m: float = 8.0) -> np.ndarray:
    """Return (N,2) array of valid scan points in robot frame."""
    mask = (snapshot.distance_m > 0.05) & (snapshot.distance_m < max_dist_m)
    return np.stack([snapshot.x_m[mask], snapshot.y_m[mask]], axis=1)
```

Downsample with a voxel grid (bucket by 0.05 m cells, keep one point per cell) to keep ICP fast:

```python
def voxel_downsample(pts: np.ndarray, cell_m: float = 0.05) -> np.ndarray:
    keys = (pts / cell_m).astype(np.int32)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]
```

#### Step 3 — Point-to-point ICP (`icp.py`)

Inputs: `source` (current scan in robot frame), `target` (previous scan transformed to world frame and rotated back), `init_R` (2×2 rotation from IMU delta_theta).

```python
def icp_2d(source, target, init_R, max_iter=20, tol=1e-4):
    """
    Returns (R, t) — 2×2 rotation and (2,) translation that align
    source → target.  init_R seeds the first correspondence search.
    Uses scipy.spatial.KDTree for nearest-neighbour queries.
    """
    from scipy.spatial import KDTree
    R, t = init_R.copy(), np.zeros(2)
    tree = KDTree(target)
    for _ in range(max_iter):
        src_rot = (R @ source.T).T + t
        dist, idx = tree.query(src_rot, workers=-1)
        # reject outliers beyond 3× median distance
        med = np.median(dist)
        mask = dist < 3.0 * med
        if mask.sum() < 10:
            break
        src_m = source[mask]
        tgt_m = target[idx[mask]]
        # SVD closed-form
        src_c = src_m - src_m.mean(0)
        tgt_c = tgt_m - tgt_m.mean(0)
        H = src_c.T @ tgt_c
        U, _, Vt = np.linalg.svd(H)
        R_new = Vt.T @ U.T
        if np.linalg.det(R_new) < 0:
            Vt[-1] *= -1
            R_new = Vt.T @ U.T
        t_new = tgt_m.mean(0) - (R_new @ src_m.mean(0))
        if np.linalg.norm(t_new - t) + abs(np.arctan2(R_new[1,0], R_new[0,0])) < tol:
            R, t = R_new, t_new
            break
        R, t = R_new, t_new
    return R, t
```

`scipy` is already in the py environment (pulled by viser). If it isn't: `pip install scipy`.

#### Step 4 — Pose integration (`slam_thread.py`)

```python
icp_theta = math.atan2(R[1, 0], R[0, 0])
icp_correction = icp_theta - delta_theta   # deviation from IMU seed

if not ok or abs(icp_correction) > MAX_ICP_CORRECTION:
    # ICP diverged — trust IMU heading, skip translation.
    self._theta = prev_theta + delta_theta
else:
    # R encodes total relative rotation (IMU seed + ICP refinement).
    self._theta = prev_theta + icp_theta
    # t is in PREVIOUS robot frame — rotate to world frame.
    c, s = math.cos(prev_theta), math.sin(prev_theta)
    self._x += c * t[0] - s * t[1]
    self._y += s * t[0] + c * t[1]
```

**Critical**: `t` from ICP is expressed in the previous scan's robot frame, not the
world frame. Without the cos/sin rotation above, translation is only correct when the
robot faces +X (theta≈0) and silently wrong otherwise.

**Also critical**: `icp_theta` already encodes the full relative rotation (IMU delta was
the seed). Do not add `delta_theta + icp_theta` — that double-counts the rotation.

#### Step 5 — Occupancy grid (`occupancy_grid.py`)

Log-odds binary grid, 0.05 m resolution. Use bresenham ray-casting to mark free cells along each beam and the endpoint cell as occupied.

```python
class OccupancyGrid:
    RESOLUTION_M = 0.05
    LOG_OCC = 0.85      # log-odds increment for occupied
    LOG_FREE = -0.40    # log-odds decrement for free
    CLAMP = (-5.0, 5.0)

    def insert(self, scan_pts_robot: np.ndarray, pose: tuple[float,float,float]):
        # transform scan to world frame, ray-cast each point
        ...

    def to_image(self) -> np.ndarray:
        # sigmoid(log_odds) → uint8 image, 0=free, 128=unknown, 255=occupied
        ...
```

Display the grid in viser as `server.scene.add_image("/map/grid", ...)` or export as PNG.

#### Step 6 — SLAM thread (`slam_thread.py`)

```python
class SlamThread:
    def __init__(self, bridge: ControllerTeleopBridge, viewer: TeleopWebViewer):
        self._bridge = bridge
        self._viewer = viewer      # viewer.robot_frame for pose, server for grid image
        self._grid = OccupancyGrid()
        self._thread: threading.Thread | None = None
        self._x = self._y = self._theta = 0.0
        self._prev_scan: np.ndarray | None = None
        self._prev_yaw_deg: float | None = None
        self._prev_scan_count = -1

    def start(self): ...
    def stop(self): ...

    def _loop(self):
        while self._running:
            snap = self._bridge.get_snapshot()
            if snap.scan_count == self._prev_scan_count:
                time.sleep(0.01)
                continue
            self._prev_scan_count = snap.scan_count
            self._update(snap)

    def _update(self, snap: ScanSnapshot):
        pts = voxel_downsample(scan_to_points(snap))
        if pts.shape[0] < 20:
            return
        if self._prev_scan is None or self._prev_yaw_deg is None:
            self._prev_scan = pts
            self._prev_yaw_deg = snap.yaw_deg
            return

        delta_theta = math.radians(snap.yaw_deg - self._prev_yaw_deg)
        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi
        init_R = np.array([[math.cos(delta_theta), -math.sin(delta_theta)],
                           [math.sin(delta_theta),  math.cos(delta_theta)]])

        R, t = icp_2d(pts, self._prev_scan, init_R)
        icp_theta = math.atan2(R[1, 0], R[0, 0])
        self._theta += delta_theta + icp_theta
        self._x += t[0]
        self._y += t[1]

        # push pose to viewer
        self._viewer.robot_frame.position = (self._x, self._y, 0.0)
        self._viewer.robot_frame.wxyz = _theta_to_wxyz(self._theta)

        self._grid.insert(pts, (self._x, self._y, self._theta))
        self._prev_scan = pts
        self._prev_yaw_deg = snap.yaw_deg
```

#### Step 7 — Wire into `controller_teleop.py`

Add `--slam` flag. When set, instantiate `SlamThread(bridge, viewer)` alongside the existing teleop loop — no structural changes to the teleop loop itself.

```python
if args.slam:
    slam = SlamThread(teleop, viewer)
    slam.start()
```

Clean up in the `finally` block.

## Known Limitations and Mitigations

| Issue | Mitigation |
|---|---|
| ICP diverges in featureless corridors | Skip ICP update if point correspondence ratio < 50%; rely on IMU heading only |
| Drift accumulates over time | No loop-closure in this plan; acceptable for short sessions. Add graph-SLAM later if needed. |
| ESP-NOW scan packet loss | `scan_count` monotonic counter already detects drops; ICP simply uses older reference scan |
| Yaw wrapping (0°/360° boundary) | Wrap delta to `[-π, π]` (shown above) before integrating |
| ICP is slow for dense scans | Voxel downsample to ~200–400 pts keeps ICP under 5 ms per frame |

## Dependencies

All pure-Python, no ROS, no C extensions beyond numpy:

```
numpy       ← already present
scipy       ← for KDTree; already pulled by viser indirectly; add to requirements.txt if absent
```

Add to `rover_stack/py/requirements.txt`:
```
scipy>=1.11
```

## Simulator

Test SLAM without hardware:

```bash
# from repo root
python slam_sim/sim.py [--web-port 8080] [--map random|office|warehouse|maze] [--seed 42]
```

**Map presets** (`--map`):

| Preset | Description |
|---|---|
| `random` | Random boxes + stub walls; `--seed N` for reproducibility |
| `office` | Two rooms joined by a doorway |
| `warehouse` | Grid of shelving units, repetitive aisles |
| `maze` | Alternating horizontal baffles, tests ICP in repetitive geometry |

**Viser scene**:
- Blue axes/trail (`/truth`) — ground truth pose
- Magenta trail (`/slam/trail`) — SLAM estimated path
- Green→blue point cloud (`/truth/scan`) — live LiDAR scan in robot frame
- Amber cloud (`/slam/grid`) — occupancy grid (Bresenham ray-cast)

**GUI controls** (live, no restart needed):
- Speed ×, LiDAR noise (m), IMU noise (°) sliders
- Pause / Resume button
- New Random Map button (regenerates world and restarts SLAM from scratch)

**Info panels**: Ground Truth (X, Y, θ), SLAM Estimate (X, Y, θ), Pos error (m), Hdg error (°), scan count, elapsed, loops completed.

`SimBridge` / `SimSnapshot` are duck-typed mocks — `SlamThread` runs unmodified.

## Recording and Replay

Record real rover telemetry to disk:

```bash
cd rover_stack/py
python record.py --port /dev/ttyACM0 [--output run1.jsonl]
```

Replay offline through SLAM (no hardware):

```bash
python replay.py --file run1.jsonl [--web-port 8080] [--speed 2.0]
# --speed 0 = as fast as possible
```

The replay pauses after the file ends so you can inspect the final map.

## Rules

- `slam/` is a standalone package at the repo root — not inside rover_stack or py_scripts
- `SlamThread` is duck-typed: any bridge with `get_snapshot()` works (real or sim)
- IMU heading is ground truth for rotation; ICP correction > ±15° vs IMU seed is treated as divergence
- SLAM is opt-in via `--slam` flag; the teleop runs identically without it
- `t` from `icp_2d` is in the previous robot frame — always rotate by `prev_theta` before adding to world position
- Update `rover_stack/README.md` when run instructions change
