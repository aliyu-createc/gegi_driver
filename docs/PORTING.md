# Porting / cloning this driver to another machine

What a fresh clone needs in order to build, run, and — most importantly — produce
*correct* activity numbers. Read §1 before anything else: the driver will happily
run on a different detector and report wrong activities without any error.

---

## 1. ⚠️ The calibration is DETECTOR-SPECIFIC — read this first

`config/isotopes.yaml` contains `calibration_factor` values that were empirically
derived **for one specific GeGi unit** and then corrected against certificated
sources (the Co-60 factors were adjusted by −8.7% and −4.5% during commissioning).

**On a different GeGi detector these numbers are wrong**, and nothing will crash —
every reported activity will simply be wrong. The same applies to the geometry:
the 0.325 m bare standoff and the single permanent 5 mm steel plate describe one
particular frame.

**If the detector or the rig changes you MUST:**

1. Re-run the efficiency validation (Experiment B in `docs/DJR_assay_experiments.md`):
   assay a certificated source in the operational geometry and compare against the
   decay-corrected certificate.
   ```bash
   python tools/validate_efficiency.py --csv <run>_activity.csv \
       --nuclide Co60 --cert-activity 1.130 --cert-date 2026-06-01
   ```
2. Update `config/isotopes.yaml` — `calibration_factor` per line, and the
   `shielding` block (`base_standoff_m`, `plate_thickness_m`, `base_shield_plates`,
   `material`) plus `mu_shield_per_m` if the shielding material changed.
3. Update **`test/commissioned.py`** to the new rig. This is the *only* test file
   that encodes rig-specific values; the physics tests are parameter-driven and
   need no changes.
4. Re-run the tests — `TestCommissionedConfiguration` failing is the suite telling
   you the config and the rig disagree.

---

## 2. Environment

| | Requirement |
|---|---|
| ROS | **Melodic** (Python 2.7) |
| OS | **Ubuntu 18.04** |
| Windows | Driver cannot run natively — use **WSL2** or **Docker** |

**ROS Melodic is end-of-life and is not installable on Ubuntu 20.04/22.04/24.04.**
On a modern machine you have three options:

- **WSL2 / Docker with Ubuntu 18.04** (what the current rig uses; see `docker/`).
- **Port to ROS Noetic + Python 3.** The code already uses
  `from __future__ import print_function` and is mostly Python-3-clean, but this
  needs a real porting pass and re-testing. `package.xml` already carries
  `ROS_PYTHON_VERSION` conditions for the numpy/yaml dependencies.
- Keep an Ubuntu 18.04 machine for the driver.

**Windows note:** only the plotting tools (`tools/plot_live_spectrum.py`,
`plot_live_2d_heatmap.py`, `plot_live_heatmap.py`) run natively on Windows — they
talk to the driver over the **rosbridge websocket on port 9090**. They need
`matplotlib` and a websocket client.

---

## 3. Build (standard catkin — no bespoke scripts needed)

The `sync.sh` / `build.sh` / `run.sh` helpers on the current machine live *outside*
the repo and are a WSL convenience only. A clone does not need them:

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone <repo-url> phds_gegi_driver
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y   # pulls declared deps
catkin_make                                          # builds deps/radiation_detector_msgs too
source devel/setup.bash
```

`deps/radiation_detector_msgs` is vendored in-tree as a proper catkin package, so
the custom `ComptonEvent` / `Spectrum` messages build automatically.

---

## 4. Site-specific settings to change

| Setting | Where | Note |
|---|---|---|
| `gegi_ip` (default `192.168.50.109`) | `launch/gegi_driver.launch`, `launch/gegi_full_pipeline.launch` | your network |
| `gegi_port` (default `27015`) | same | rarely changes |
| `output_dir` (default `/opt/phds_gegi_driver/data`) | `launch/gegi_full_pipeline.launch`, `launch/data_recorder.launch` | must exist and be writable |
| Calibration + geometry | `config/isotopes.yaml`, `test/commissioned.py` | **see §1** |
| Energy calibration | `config/EnergyCal.csv` | detector-specific bin edges |

Override at launch rather than editing defaults:

```bash
roslaunch phds_gegi_driver gegi_full_pipeline.launch \
    gegi_ip:=<detector-ip> output_dir:=$HOME/gegi_data
```

The detector accepts **one TCP connection** — the C++ node owns it. Do not open a
second connection.

---

## 5. Verify the clone

Unit tests need **no detector, no roscore, no hardware** — run them first:

```bash
bash test/run_tests.sh          # 97 tests; -v for verbose
```

The runner auto-detects the ROS distro (`/opt/ros/*`), the catkin workspace
(`$GEGI_WS`, `~/gegi_ws`, `~/catkin_ws`, or the parent workspace) and the Python
interpreter. Override with `GEGI_WS=... PYTHON=... bash test/run_tests.sh`.

Interpreting failures:

| Failing suite | Meaning |
|---|---|
| `TestCommissionedConfiguration` | The yaml no longer matches the rig. Either a bad edit to `isotopes.yaml`, or a genuine rig change → update `test/commissioned.py`. |
| `TestStructuralInvariants` | `isotopes.yaml` is malformed or physically impossible (zeroed calibration factor, ROI that no longer brackets its photopeak, …). |
| Physics / tools suites | A real regression in the driver maths. These are rig-independent — they should never fail because of a config or hardware change. |

Then check the live system:

```bash
rostopic hz /compton_event /energy_deposit     # detector streaming?
rosservice call /detector/get_detector_info    # serial, temperature, bias
```

---

## 6. Known gotchas

- **Shell scripts must stay LF.** `.gitattributes` enforces `*.sh text eol=lf`;
  without it, CRLF line endings break `bash` on Linux.
- **Python 2 needs an encoding header.** Any source file containing non-ASCII
  characters (e.g. `—`, `µ`, `§`) needs `# -*- coding: utf-8 -*-` or it raises
  `SyntaxError` at import.
- **Executable bit** on the Python nodes must survive the clone for `rosrun`;
  `catkin_install_python` handles the installed copies.
- **`/spectrum` publishes per-interval DELTAS, not a cumulative histogram.** The
  data recorder integrates them. If you change this, every saved N42 will
  over-count. It is unit-tested (`test/test_spectrum.py::TestDeltaContract`).
- **Known quirk:** events above the calibrated energy range are counted in the top
  spectrum bin rather than discarded (documented in `test_spectrum.py`). Harmless
  for the Cs/Co ROIs.
