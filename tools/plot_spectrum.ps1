param(
    [switch]$Cumulative
)
# Launch the live spectrum plotter natively on Windows via rosbridge.
# Requires:
#   - Container 'gegi' running with gegi_full_pipeline.launch
#   - rosbridge_server running in container (port 9090)
#   - pip install roslibpy matplotlib numpy
#
# Usage:
#   .\plot_spectrum.ps1              # Show latest snapshot each update
#   .\plot_spectrum.ps1 -Cumulative  # Accumulate counts over time

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$pythonScript = Join-Path $scriptDir "plot_live_spectrum.py"

# Ensure rosbridge is running in the container
docker exec gegi bash -c "source /opt/ros/melodic/setup.bash && source /opt/phds_gegi_driver/devel/setup.bash && rosnode list" 2>$null | Out-Null

$args_ = @($pythonScript, "--host", "localhost", "--port", "9090")
if ($Cumulative) {
    $args_ += "--cumulative"
}

& python @args_
