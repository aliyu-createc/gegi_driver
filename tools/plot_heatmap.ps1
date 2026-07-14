param()
# Launch the live spherical heatmap plotter natively on Windows via rosbridge.
# Requires:
#   - Container 'gegi' running with gegi_full_pipeline.launch
#   - rosbridge_server running in container (port 9090)
#   - pip install roslibpy matplotlib numpy
#
# Usage:
#   .\plot_heatmap.ps1

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$pythonScript = Join-Path $scriptDir "plot_live_heatmap.py"

& python $pythonScript --host localhost --port 9090
