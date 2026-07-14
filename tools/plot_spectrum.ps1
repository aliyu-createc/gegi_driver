param(
    [switch]$Cumulative,
    [string]$RosHost = "localhost",
    [int]$RosPort = 9090
)
# Launch the live spectrum plotter natively on Windows via rosbridge.
# Requires:
#   - gegi_full_pipeline.launch running (WSL distro 'ros-melodic': ~/gegi_ws/run.sh),
#     which starts rosbridge_server on port 9090
#   - pip install roslibpy matplotlib numpy   (in the Windows Python env)
#
# Usage:
#   .\plot_spectrum.ps1                       # Show latest snapshot each update
#   .\plot_spectrum.ps1 -Cumulative           # Accumulate counts over time
#   .\plot_spectrum.ps1 -RosHost <wsl-ip>     # If localhost forwarding fails, use `wsl hostname -I`

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$pythonScript = Join-Path $scriptDir "plot_live_spectrum.py"

$args_ = @($pythonScript, "--host", $RosHost, "--port", $RosPort)
if ($Cumulative) {
    $args_ += "--cumulative"
}

& python @args_
