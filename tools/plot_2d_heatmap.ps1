param()
# Launch the per-isotope 2D heatmap plotter natively on Windows via rosbridge.
# Produces the Y-Z plane projection with isotope-labeled source markers.
# Requires:
#   - Container 'gegi' running with full pipeline + rosbridge
#   - pip install roslibpy matplotlib numpy scipy
#
# Usage:
#   .\plot_2d_heatmap.ps1

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$pythonScript = Join-Path $scriptDir "plot_live_2d_heatmap.py"

& python $pythonScript --host localhost --port 9090
