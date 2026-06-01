# ROS PHDS GeGI Driver

## What This Repository Does

This project provides:

- A TCP driver that talks to a PHDS GeGI detector over IP/port.
- A ROS node that publishes Compton events and per-interaction energy deposits.
- ROS services for detector control (start, stop, clear, run info, detector info, timed acquisitions, bias toggle).
- A spectrum accumulator node with energy calibration.
- A spherical heatmap node for directional source localization and isotope identification.
- Utility scripts for topic monitoring and live plotting.

## Main ROS Interfaces

### Published Topics

**Driver node** (`phds_gegi_driver_node`):

- `/compton_event` (`radiation_detector_msgs/ComptonEvent`): Raw Compton event stream from the detector.
- `/energy_deposit` (`std_msgs/Float64`): Per-interaction energy in keV.

**Spectrum node** (`spectrum_node.py`):

- `/spectrum` (`radiation_detector_msgs/Spectrum`): Accumulated energy histogram, published at configurable rate.

**Spherical heatmap node** (`spherical_heatmap_node.py`):

- `/sphere_heatmap` (`sensor_msgs/PointCloud2`): Scored sphere grid for RViz visualization.
- `/source_direction` (`geometry_msgs/PoseStamped`): Peak source direction estimate.
- `/source_directions` (`geometry_msgs/PoseArray`): All detected direction peaks.
- `/source_isotopes` (`std_msgs/String`): Isotope identification result.

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
| `gegi_full_pipeline.launch` | Driver + spectrum + spherical heatmap + rosbridge websocket |
| `spectrum.launch` | Standalone spectrum accumulator node |
| `spherical_heatmap.launch` | Standalone spherical heatmap node |

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

### 2) Run the full pipeline container

```powershell
docker run --rm -d -p 9090:9090 --name gegi_live phds_gegi_driver bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; roslaunch phds_gegi_driver gegi_full_pipeline.launch"
```

What it does:

- Starts a container named `gegi_live` in detached mode.
- Publishes the rosbridge websocket on port 9090 so Windows-side plotting tools can connect.
- Sources ROS + workspace setup files.
- Launches driver + spectrum + spherical heatmap + rosbridge via `gegi_full_pipeline.launch`.

> **Note:** Do not use `--network host` on Docker Desktop for Windows — it does not actually expose container ports to the host. Use `-p` port mappings instead.

If you only want the detector driver (without spectrum/heatmap), use:

```powershell
docker run --rm -d -p 9090:9090 --name gegi_live phds_gegi_driver bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; roslaunch phds_gegi_driver gegi_driver.launch"
```

## Monitoring Topics

### watch_gegi_topics.ps1

Opens separate PowerShell windows that stream topic output from the running container:

```powershell
.\watch_gegi_topics.ps1
```

Parameters:

- `-ContainerName <name>`: Specify container explicitly (auto-detects by default).
- `-Mode echo|hz`: Use `echo` for message content or `hz` for publish rate.
- `-SampleCount N`: Limit to N messages then stop (echo mode only).

### Manual topic inspection

```powershell
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; rostopic echo /compton_event"
```

```powershell
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; rostopic echo /spectrum"
```

```powershell
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; rostopic echo /source_direction"
```

### List all active topics / services

```powershell
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; rostopic list"
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; rosservice list"
```

## Plotting Tools

Live plotting scripts connect to the ROS container via rosbridge websocket:

| Script | Description |
|--------|-------------|
| `tools/plot_live_spectrum.py` | Real-time energy spectrum display |
| `tools/plot_live_heatmap.py` | Live spherical heatmap visualization |
| `tools/plot_live_2d_heatmap.py` | Live 2D heatmap projection |
| `tools/plot_spectrum.ps1` | Static spectrum plot from bag/data |
| `tools/plot_heatmap.ps1` | Static heatmap plot |
| `tools/plot_2d_heatmap.ps1` | Static 2D heatmap plot |

## Detector Control via ROS Services

Call services from inside the container:

```powershell
# Start continuous acquisition
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; rosservice call /detector/start_acquisition"

# Stop acquisition
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; rosservice call /detector/stop_acquisition"

# Clear accumulated data
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; rosservice call /detector/clear_data"

# Start timed acquisition (e.g. 5 minutes)
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; rosservice call /detector/start_timed_acquisition '{duration_minutes: 5}'"

# Get detector info (serial, temperature, bias, battery)
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; rosservice call /detector/get_detector_info"

# Get run info (timing, count-rate)
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; rosservice call /detector/get_run_info"

# Toggle bias mode
docker exec -it gegi_live bash -lc "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash; rosservice call /detector/toggle_bias_mode"
```

## Troubleshooting

### Command returns no output

- Confirm container is running: `docker ps` and verify `gegi_live` exists.
- Confirm ROS graph is alive with `rostopic list`.
- Confirm services are available with `rosservice list`.

### Topic exists but no messages

- Start acquisition via the `/detector/start_acquisition` service.
- Verify detector is physically connected and in recording mode.
- Check container logs for driver errors:

```powershell
docker logs gegi_live --tail 200
```

## Non-Docker Build (Linux catkin)

1. Create or use an existing catkin workspace.
2. Put this repository under `<catkin_ws>/src`.
3. Install dependencies (`setup/phds_gegi_driver/setup_dependencies.sh`).
4. Build with `catkin_make`.
5. Run with `roslaunch phds_gegi_driver gegi_driver.launch`.

## License

Please see [LICENSE.md](LICENSE.md).
