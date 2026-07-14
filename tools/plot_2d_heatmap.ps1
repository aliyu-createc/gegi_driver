param(
    [string]$RosHost = "localhost",
    [int]$RosPort = 9090
)
# Launch the per-isotope 2D heatmap plotter natively on Windows via rosbridge.
# Produces the Y-Z plane projection with isotope-labeled source markers.
# Requires:
#   - gegi_full_pipeline.launch running (WSL distro 'ros-melodic': ~/gegi_ws/run.sh),
#     which starts rosbridge_server on port 9090
#   - pip install roslibpy matplotlib numpy scipy   (in the Windows Python env)
#
# Usage:
#   .\plot_2d_heatmap.ps1
#   .\plot_2d_heatmap.ps1 -RosHost <wsl-ip>   # If localhost forwarding fails, use `wsl hostname -I`

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$pythonScript = Join-Path $scriptDir "plot_live_2d_heatmap.py"

& python $pythonScript --host $RosHost --port $RosPort
