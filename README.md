# ROS PHDS GeGI Driver

## What This Repository Does

This project provides:

- A TCP driver that talks to a PHDS GeGI detector over IP/port.
- A ROS node that publishes detector events and spectrum data.
- ROS services for detector control (start, stop, clear, run info, detector info, timed acquisitions, bias toggle).
- Optional Compton localization and live heatmap generation.
- Utility scripts for acquisition control, bag capture, and plotting.

In short: it is a full runtime stack for live GeGI data acquisition, status/control, and localization output.

## Main ROS Interfaces

### Published Topics

- `/compton_event`: Raw Compton event stream from the detector.
- `/spectrum_without_dose`: Energy spectrum updates.
- `/source_estimate`: Estimated source position output from localization.
- `/source_heatmap`: Heatmap representation of likely source locations.
- `/source_peaks`: Peak candidates extracted from the heatmap.

### Exposed Services

All detector control services are under `/detector/*`, including:

- `/detector/start_acquisition`
- `/detector/stop_acquisition`
- `/detector/clear_data`
- `/detector/get_run_info`
- `/detector/get_detector_info`
- `/detector/toggle_bias_mode`
- `/detector/start_5min_acquisition` through `/detector/start_25min_acquisition`

## Prerequisites

### Hardware / Detector

- Detector powered and cooled (according to PHDS operating guidance).
- Detector network reachable from the host running Docker.
- Detector acquisition application on the tablet in the correct recording state.

### Software

- Docker Desktop (Windows/Linux/macOS).
- For non-Docker workflows: ROS Melodic-compatible catkin environment.

## Quick Start (Docker)

### 1) Build the image

Run from the repository root:

```powershell
docker build -t phds_gegi_driver -f .\docker\dockerfiles\phds_gegi_amd64 .
```

What it does:

- Builds the ROS workspace and driver into an image named `phds_gegi_driver`.
- Uses the amd64 Dockerfile in `docker/dockerfiles/phds_gegi_amd64`.

Notes:

- The first build needs internet access because Docker must pull the base ROS image and install apt packages.
- After the image has been built once, source-only rebuilds can usually run offline as long as Docker cache and the base image are still present locally.
- If you change the Dockerfile itself or clear the Docker cache, the build may need internet again.

### 2) Run the live stack container

```powershell
docker run --rm --network host --name gegi_live phds_gegi_driver bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; roslaunch phds_gegi_driver gegi_with_localization.launch"
```

What it does:

- Starts a container named `gegi_live`.
- Uses host networking so ROS and detector traffic are directly reachable.
- Sources ROS + workspace setup files.
- Launches driver plus localization nodes via `gegi_with_localization.launch`.

If you only want the detector driver (without localization), use:

```powershell
docker run --rm --network host --name gegi_live phds_gegi_driver bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; roslaunch phds_gegi_driver gegi_driver.launch"
```

## Detector Control Commands (Detailed)

The script `acquisition_control.ps1` calls detector ROS services from the running container.

### Start / Stop / Clear

```powershell
.\acquisition_control.ps1 start
.\acquisition_control.ps1 stop
.\acquisition_control.ps1 clear
```

- `start`: Sends `/detector/start_acquisition` (continuous acquisition mode).
- `stop`: Sends `/detector/stop_acquisition`.
- `clear`: Sends `/detector/clear_data` (clears accumulated data on detector side).

### Timed presets

```powershell
.\acquisition_control.ps1 1
.\acquisition_control.ps1 2
.\acquisition_control.ps1 3
.\acquisition_control.ps1 4
.\acquisition_control.ps1 5
```

- `1` = 5 minutes (`/detector/start_5min_acquisition`)
- `2` = 10 minutes
- `3` = 15 minutes
- `4` = 20 minutes
- `5` = 25 minutes

### Detector status / run status

```powershell
.\acquisition_control.ps1 d
.\acquisition_control.ps1 i
```

- `d` (detector-info): returns serial, detector temperature, bias state, line power state, battery levels.
- `i` (info): returns run timing and count-rate metrics.

### Toggle bias mode

```powershell
.\acquisition_control.ps1 bias
```

- Calls `/detector/toggle_bias_mode`.
- Use with care during active acquisition, according to detector operating procedures.

## Monitoring and Debug Commands (Detailed)

The following commands are executed inside the running container and are useful for live validation.

### 1) Echo raw event stream

```powershell
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; rostopic echo /compton_event"
```

What it shows:

- Every Compton event message as it is published.
- Useful to verify detector data flow is alive.

How to interpret:

- Continuous messages indicate live detector traffic.
- No output usually means acquisition is not running, detector link is down, or driver is not connected.

### 2) Echo localization estimate

```powershell
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; rostopic echo /source_estimate"
```

What it shows:

- The current source localization estimate generated by the localization node.

How to interpret:

- Stable estimates over time indicate enough event statistics and stable geometry.
- Highly noisy or missing estimates usually indicate low counts, insufficient cones, or localization parameter mismatch.

### 3) Echo spectrum output

```powershell
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; rostopic echo /spectrum_without_dose"
```

What it shows:

- Spectrum histogram updates from the detector driver.

How to interpret:

- Confirms energy-channel data is being produced even if localization is sparse.

### 4) List available topics

```powershell
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; rostopic list"
```

What it does:

- Lists all active topics in the current ROS graph.
- Good first check when something appears missing.

### 5) List available services

```powershell
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; rosservice list"
```

What it does:

- Confirms detector control services are registered.
- If `/detector/*` services are absent, the driver node is not up correctly.

## Live Heatmap Script

You can launch a tuned live heatmap pipeline with:

```powershell
.\run_live_heatmap.ps1 -PeakCount 1 -WindowSeconds 180 -SnapshotPeriodSeconds 20 -SnapshotPrefix stable_single
```

What it does:

- Starts (or replaces) container `gemi_live`.
- Runs `gemi_with_localization.launch` with heatmap parameters.
- Saves PNG snapshots to `data/heatmaps` on the host.

## Live Source Estimate Viewer (Windows Native)

A native Windows PowerShell viewer displays live 3D source localization estimates in real-time across three 2D projections (XY, XZ, YZ): 

```powershell
.\live_source_estimate_viewer.ps1 -AxisLimit 2.0 -MaxPoints 500
```

### Basic Usage

Parameters:
- `-AxisLimit 2.0`: Sets plot axis range to ±2.0 m (default 1.5 m).
- `-MaxPoints 500`: Retain last 500 estimates before rolling buffer (default 300).
- `-ContainerName gegi_live`: Docker container name (default "gegi_live").
- `-Topic /source_estimate`: ROS topic to monitor (default "/source_estimate").

### Understanding the Plot

**Markers and colors:**
- **Orange squares**: Historical source estimates (rolling buffer).
- **Cyan diamond**: Latest single estimate (can jitter).
- **Green cross**: Mean of all retained estimates (statistical best guess).
- **Magenta star**: Hottest location (densest cluster of recent estimates).

**Status display (top-left):**
- `Latest: x/y/z` + range: Most recent estimate coordinates and 3D distance from origin.
- `Mean: x/y/z` + range: Mean estimate (usually more stable).
- `Face(...)`: Distance metrics relative to detector front face (see below).
- `Hottest: x/y/z`: Peak density location in 3D binned histogram of recent points.
- `Points retained`: Number of estimates currently in rolling buffer.

### Interpreting Face-Relative Distance

If you know the physical source location relative to the detector front face, configure:

```powershell
.\live_source_estimate_viewer.ps1 -FrontFaceAxis Y -FrontFacePositionM 0.0 -AxisLimit 2.0 -MaxPoints 500
```

Then read the `Face(...)` line:
- `-FrontFaceAxis Y`: Which axis points outward from detector face (`X`, `Y`, or `Z`).
- `-FrontFacePositionM 0.0`: Detector frame coordinate of the front face plane (usually 0 if frame origin is at face).
- `normal`: Distance perpendicular to the face plane.
- `lateral`: Offset parallel to the face (should be small for on-axis source).

Example: If source is 32 cm away from face, `mean normal` should converge near `0.32 m`.

### Tips for Convergence

- Point cloud should remain in same region and trend toward hotspot location.
- If estimates are scattered, increase localization parameters in launch file:
  - `min_cones_for_estimate`: Require more coincident Compton cones (default 3).
  - `cone_accumulation_timeout_s`: Accumulate cones over longer window (default 5 s).
  - `estimation_period_s`: Recompute less frequently for more events per update (default 2 s).
- Green mean is more reliable than cyan latest for geometric validation.

## Data Capture

If you are running inside a ROS environment with `rosbag` available:

```bash
python3 capture.py --duration 60 --label Cs137_test
python3 capture.py --label background_scan --manual
```

What it does:

- Records selected topics into `data/*.bag`.
- Writes metadata JSON sidecar files.
- Supports fixed-duration or Ctrl+C manual stop mode.

## Raw .ipb Grid Dataset Preparation

If you captured runs directly from GImagerPro as `.ipb` files in `data/rawData`, use:

```powershell
python .\parse_ipb.py --input-dir .\data\rawData --manifest .\data\rawData\ipb_manifest.csv --write-grid-template .\data\rawData\grid_labels_template.csv
```

What this gives you:

- `data/rawData/ipb_manifest.csv`: one row per `.ipb` with timestamp/index parsed from filename, gzip validation, payload sizes, checksums, and key marker flags.
- `data/rawData/grid_labels_template.csv`: template to fill with ground-truth source positions:
  - `grid_x_m, grid_y_m, grid_z_m, source_id, notes`

After filling `grid_labels_template.csv`, rerun with labels merged:

```powershell
python .\parse_ipb.py --input-dir .\data\rawData --manifest .\data\rawData\ipb_manifest_labeled.csv --grid-labels .\data\rawData\grid_labels_template.csv
```

Optional: export decompressed payloads for deeper reverse-engineering/parser development:

```powershell
python .\parse_ipb.py --input-dir .\data\rawData --export-bin-dir .\data\rawData\bin_payloads
```

Note:

- `.ipb` files are gzip-wrapped serialized payloads; event-level decoding depends on the internal GImagerPro object schema.
- The manifest/label workflow is the recommended first step to build a clean supervised reconstruction dataset.

## Accuracy-Focused Reconstruction Pipeline

To improve source localization accuracy from accumulated Compton events, use:

### 1) Build tuner-ready labeled events CSV

Export per-scan event CSV files first (for example from bag files using `analyse_bag.py --csv`).

Then join those event CSVs with the 127-pose label index:

```powershell
python .\build_tuner_events_csv.py --events-glob ".\data\rawData\*_events.csv" --index-csv .\data\rawData\reconstruction_index_pose127.csv --output-csv .\data\rawData\events_with_labels.csv --mapping-mode sequence
```

If your event file names contain `source_id` (for example `pose_042_events.csv`), you can use:

```powershell
python .\build_tuner_events_csv.py --events-glob ".\data\rawData\*_events.csv" --mapping-mode source_id
```

### 2) Offline parameter tuning

Prepare an event-level CSV (one row per gamma event) with:

- Geometry fields: `x1,y1,z1,x2,y2,z2,cone_angle_rad,cone_angle_uncertainty_rad`
- Energy fields: `energy_kev_1,energy_kev_2`
- Ground truth labels: `grid_x_m,grid_y_m,grid_z_m`
- Group column (recommended): `source_id`

Then tune weighting/uncertainty parameters:

```powershell
python .\tune_reconstruction_params.py --events-csv .\data\rawData\events_with_labels.csv --group-column source_id --output-csv .\data\rawData\tuning_results.csv --output-best-json .\data\rawData\best_reconstruction_params.json
```

Outputs:

- `data/rawData/tuning_results.csv`: parameter leaderboard with mean and p95 localization error.
- `data/rawData/best_reconstruction_params.json`: best parameter set for live use.

One-command version (build + tune):

```powershell
.\run_tuning_pipeline.ps1
```

Useful variants:

```powershell
# If event filenames include source_id text (e.g. pose_042_events.csv)
.\run_tuning_pipeline.ps1 -MappingMode source_id

# Restrict to full-energy window around Cs-137 peak and cap events per scan
.\run_tuning_pipeline.ps1 -MinTotalEnergyKeV 580 -MaxTotalEnergyKeV 750 -MaxEventsPerScan 4000
```

### 3) Live fused estimator (rolling accumulation)

Run the uncertainty-weighted 3D fused estimator:

```powershell
.\run_live_fused_estimator.ps1 -WindowSeconds 120 -UpdatePeriodSeconds 2 -MinEvents 200 -Resolution 0.02
```

What it does:

- Consumes `/compton_event`.
- Accumulates events over `WindowSeconds`.
- Solves a 3D grid maximum-likelihood estimate using cone uncertainty and energy weighting.
- Publishes fused output on `/source_estimate_fused` as `PoseWithCovarianceStamped`.

If detector pose changes during scan and a pose topic is available, include it:

```powershell
.\run_live_fused_estimator.ps1 -DetectorPoseTopic /detector_pose
```

This allows cross-position event fusion in a consistent world frame.

To apply tuned parameters from `best_reconstruction_params.json`, set:

- `-SigmaFloor` from `sigma_floor`
- `-UncertaintyScale` from `uncertainty_scale`
- `-EnergyPower` from `energy_power`

Example:

```powershell
.\run_live_fused_estimator.ps1 -SigmaFloor 0.03 -UncertaintyScale 1.0 -EnergyPower 0.7
```

## Troubleshooting

### Command returns no output

- Confirm container is running: `docker ps` and verify `gegi_live` exists.
- Confirm ROS graph is alive with `rostopic list`.
- Confirm services are available with `rosservice list`.

### Topic exists but no messages

- Check acquisition state (`.\acquisition_control.ps1 start`).
- Verify detector is physically connected and in recording mode.
- Check container logs for driver errors:

```powershell
docker logs gegi_live --tail 200
```

### Driver container name mismatch

`acquisition_control.ps1` auto-resolves container names in this order:

1. Exact `gegi_live`
2. Prefix `gegi_live*`
3. Prefix `gegi_*`

## Non-Docker Build (Linux catkin)

1. Create or use an existing catkin workspace.
2. Put this repository under `<catkin_ws>/src`.
3. Install dependencies (`setup/phds_gegi_driver/setup_dependencies.sh`).
4. Build with `catkin_make`.
5. Run with `roslaunch phds_gegi_driver gegi_driver.launch`.

## License

Please see [LICENSE.md](LICENSE.md).
