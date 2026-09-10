# ROS PHDS GeGI Driver

## What This Repository Does

This project provides:

- A TCP driver that talks to a PHDS GeGI detector over IP/port.
- A ROS node that publishes Compton events and per-interaction energy deposits.
- ROS services for detector control (start, stop, clear, run info, detector info, timed acquisitions, bias toggle).
- A spectrum accumulator node with energy calibration.
- A spherical heatmap node for directional source localization and isotope identification.
- An activity node that computes net peak areas and Bq per isotope.
- A data recorder that saves `.bag`, `.n42`, and `.csv` at the end of a timed acquisition.
- Utility scripts for topic monitoring and live plotting.

## Main ROS Interfaces

### Published Topics

**Driver node** (`phds_gegi_driver_node`):

- `/compton_event` (`radiation_detector_msgs/ComptonEvent`): Raw Compton event stream from the detector.
- `/energy_deposit` (`std_msgs/Float64`): Per-interaction energy in keV.

**Spectrum node** (`spectrum_node.py`):

- `/spectrum` (`radiation_detector_msgs/Spectrum`): Accumulated energy histogram, published at configurable rate.
  - Publishes **per-interval deltas**, not a running total. The data recorder integrates them.

**Spherical heatmap node** (`spherical_heatmap_node.py`):

- `/sphere_heatmap` (`sensor_msgs/PointCloud2`): Scored sphere grid for RViz visualization.
- `/source_direction` (`geometry_msgs/PoseStamped`): Peak source direction estimate.
- `/source_directions` (`geometry_msgs/PoseArray`): All detected direction peaks.
- `/source_isotopes` (`std_msgs/String`): Isotope identification result.

**Activity node** (`activity_node.py`):

- Computes net peak area and activity (Bq) per isotope from the accumulated spectrum, using the calibration and geometry in `config/isotopes.yaml`.

### Exposed Services

All detector control services are under `/detector/*`:

- `/detector/start_acquisition`
- `/detector/stop_acquisition`
- `/detector/clear_data`
- `/detector/get_run_info`
- `/detector/get_detector_info`
- `/detector/toggle_bias_mode`
- `/detector/start_timed_acquisition` (takes `duration_minutes`)

### Launch Files

| File | Description |
|------|-------------|
| `gegi_driver.launch` | Driver node only (TCP connection to detector) |
| `gegi_full_pipeline.launch` | Driver + spectrum + spherical heatmap + activity + data recorder + rosbridge websocket |
| `spectrum.launch` | Standalone spectrum accumulator node |
| `spherical_heatmap.launch` | Standalone spherical heatmap node |
| `activity.launch` | Standalone activity computation node |
| `data_recorder.launch` | Standalone data recorder node |

## Prerequisites

### Hardware / Detector

- Detector powered and cooled (according to PHDS operating guidance).
- Detector network reachable from the host running the driver (default `192.168.50.109:27015`).
- Detector acquisition application on the tablet in the correct recording state.
- The detector accepts **one TCP connection**, owned by the C++ driver node. Do not open a second connection.

### Software

- **ROS Melodic** (Python 2.7) on **Ubuntu 18.04** — Melodic is end-of-life and will not install on newer Ubuntu.
- On Windows this runs under **WSL2** (the current development setup) or Docker; the driver cannot run natively on Windows.
- Only the live plotting tools run natively on Windows — they connect over the rosbridge websocket on port 9090.

## Development Workflow (WSL2 — current setup)

The driver is developed and run natively inside a WSL2 distro named **`ros-melodic`**
(Ubuntu 18.04 + ROS Melodic). This replaced the old Docker "rebuild an image per
change" loop.

**The Windows repository is the single source of truth.** The catkin workspace at
`~/gegi_ws` inside the distro is a working copy, kept in sync by helper scripts.

> The `sync.sh` / `build.sh` / `run.sh` helper scripts live **outside the repo**, in
> `~/gegi_ws/` inside the distro (staged copies at `C:\wsl\*.sh`). They are a WSL
> convenience and are intentionally not versioned here — a fresh clone uses standard
> catkin instead (see [docs/PORTING.md](docs/PORTING.md)).

### What each helper does

| Script | Action | When to run |
|--------|--------|-------------|
| `sync.sh` | Re-copy source from the Windows repo, strip CRLF line endings, install the Python nodes into `devel/lib`. | After **Python** node changes |
| `build.sh` | `sync.sh` + `catkin_make`. | After **C++** changes |
| `run.sh` | `roslaunch phds_gegi_driver gegi_full_pipeline.launch`. | To start the pipeline |

Because `catkin_install_python` is install-only, the Python nodes are copied into
`~/gegi_ws/devel/lib/phds_gegi_driver/` by `sync.sh` (this mirrors the Dockerfile).

### Typical loop

Open the distro:

```powershell
wsl -d ros-melodic
```

Then inside the distro:

```bash
# after editing Python nodes on the Windows side:
cd ~/gegi_ws && ./sync.sh && ./run.sh

# after editing C++ (driver / socket_comms):
cd ~/gegi_ws && ./build.sh && ./run.sh
```

> **Cross-shell quoting caveat:** running `wsl bash -lc "..."` from a Windows shell
> eats `$variables`. Run script *files* inside the distro (or via
> `tr -d '\r' < /mnt/c/wsl/x.sh > /tmp/x.sh && bash /tmp/x.sh`) rather than passing
> inline command strings with shell variables.

### Landing recorded data in the Windows repo

Override `output_dir` to a `/mnt/c/...` path so the data recorder writes `.bag` /
`.n42` / `.csv` straight into a Windows folder:

```bash
roslaunch phds_gegi_driver gegi_full_pipeline.launch \
    output_dir:=/mnt/c/Users/<you>/gegi_data
```

(`data/` is gitignored — measurement data is archived separately, not committed.)

## Monitoring Topics

From inside the `ros-melodic` distro (ROS is sourced by the login shell via the
helper setup):

```bash
rostopic list
rosservice list

rostopic echo /compton_event
rostopic echo /spectrum
rostopic echo /source_direction

rostopic hz /compton_event /energy_deposit    # is the detector streaming?
```

The `watch_gegi_topics.ps1` helper (Windows side) can open separate windows that
stream topic output; it auto-detects the running environment.

## Plotting Tools (run on Windows)

Live plotting scripts connect to the driver via the rosbridge websocket on port
9090. They need `matplotlib` and a websocket client.

| Script | Description |
|--------|-------------|
| `tools/plot_live_spectrum.py` | Real-time energy spectrum display (`c` clears the display) |
| `tools/plot_live_heatmap.py` | Live spherical heatmap visualization |
| `tools/plot_live_2d_heatmap.py` | Live 2D heatmap projection |
| `tools/plot_spectrum.ps1` | Static spectrum plot from bag/data |
| `tools/plot_heatmap.ps1` | Static heatmap plot |
| `tools/plot_2d_heatmap.ps1` | Static 2D heatmap plot |

## Detector Control via ROS Services

From inside the distro:

```bash
# Start continuous acquisition (streams/plots only — does NOT save files)
rosservice call /detector/start_acquisition

# Stop acquisition
rosservice call /detector/stop_acquisition

# Clear accumulated data
rosservice call /detector/clear_data

# Start a TIMED, RECORDED run (e.g. 5 minutes). Call the RECORDER's service,
# not the detector's — this one forwards to the detector AND saves
# .bag/.n42/.csv at the end. The bare /detector/start_timed_acquisition does
# not save anything.
rosservice call /data_recorder/start_timed_recording "{duration_minutes: 5}"

# Stop a recorded run early and save what was collected
rosservice call /data_recorder/stop_recording

# Get detector info (serial, temperature, bias, battery)
rosservice call /detector/get_detector_info

# Get run info (timing, count-rate)
rosservice call /detector/get_run_info

# Toggle bias mode
rosservice call /detector/toggle_bias_mode
```

## Unit Tests

The tests need **no detector, no roscore, no hardware**:

```bash
bash test/run_tests.sh          # -v for verbose
```

The runner auto-detects the ROS distro, catkin workspace, and Python interpreter.
Rig-specific values (calibration factors, geometry) are isolated in
`test/commissioned.py`; the physics tests are parameter-driven. See
[docs/PORTING.md](docs/PORTING.md) for how the suites map to failures.

## Troubleshooting

### Topic exists but no messages

- Start acquisition via the `/detector/start_acquisition` service.
- Verify the detector is physically connected and in recording mode.
- Confirm the ROS graph is alive with `rostopic list` and `rosservice list`.

### Empty spectra / heatmaps during acquisition

The detector multiplexes commands and the event stream on one TCP link. Live
run-info dead-time polling is **disabled by default** in `gegi_full_pipeline.launch`
for this reason — polling `/detector/get_run_info` mid-run contends with the event
stream. Authoritative dead-time is fetched once, after the run, by the data recorder.

### Detector unreachable

- Confirm `192.168.50.109:27015` is reachable from the distro (default WSL2 NAT is
  sufficient — no mirrored networking needed).
- Only one TCP connection is allowed; make sure nothing else holds it.

## Docker (fallback)

Docker is retained as a fallback only; the WSL2 workflow above is the current
development path. The `phds_gegi_driver:latest` image and the amd64 Dockerfile in
`docker/dockerfiles/` still build the workspace.

```powershell
# Build
docker build -t phds_gegi_driver -f .\docker\dockerfiles\phds_gegi_amd64 .

# Run the full pipeline (rosbridge exposed on 9090)
docker run --rm -d -p 9090:9090 --name gegi_live phds_gegi_driver bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; roslaunch phds_gegi_driver gegi_full_pipeline.launch"
```

> **Note:** Do not use `--network host` on Docker Desktop for Windows — it does not
> expose container ports to the host. Use `-p` port mappings instead.

## Cloning to Another Machine

A fresh clone builds with **standard catkin** — it does not need the WSL helper
scripts. See **[docs/PORTING.md](docs/PORTING.md)**, and read §1 first: the
calibration in `config/isotopes.yaml` is **detector-specific** and must be re-derived
for a different GeGI unit or a changed rig.

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone <repo-url> phds_gegi_driver
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
roslaunch phds_gegi_driver gegi_full_pipeline.launch
```

## License

Please see [LICENSE.md](LICENSE.md).
