param(
    [string]$ContainerName = "",
    [string]$ImageHint = "phds_gegi_driver",
    [ValidateSet("echo", "hz")]
    [string]$Mode = "echo",
    [int]$SampleCount = 0
)

$ErrorActionPreference = "Stop"

# Full GeGI pipeline topics (driver + localization + reconstruction)
$Topics = @(
    "/compton_event",
    "/spectrum_without_dose",
    "/source_estimate",
    "/source_heatmap",
    "/source_peaks",
    "/synthetic_plane_cloud",
    "/compton_point_cloud"
)

function Test-ContainerRunning {
    param([string]$Name)

    $runningId = docker ps -q --filter "name=^${Name}$"
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to query Docker for container '${Name}'."
    }

    return -not [string]::IsNullOrWhiteSpace(($runningId | Out-String).Trim())
}

function Find-ContainerByImageHint {
    param([string]$Hint)

    $rows = docker ps --format "{{.Names}}|{{.Image}}"
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to query running Docker containers."
    }

    foreach ($row in $rows) {
        if ([string]::IsNullOrWhiteSpace($row)) {
            continue
        }

        $parts = $row -split "\|", 2
        if ($parts.Count -ne 2) {
            continue
        }

        $name = $parts[0].Trim()
        $image = $parts[1].Trim()
        if ($image -like "${Hint}*" -or $name -like "gegi*" -or $name -like "gemi*") {
            return $name
        }
    }

    return ""
}

function New-RosTopicCommand {
    param(
        [string]$Topic,
        [string]$SelectedMode,
        [int]$N
    )

    $setup = "source /opt/ros/melodic/setup.bash; source /opt/phds_gegi_driver/devel/setup.bash"

    if ($SelectedMode -eq "hz") {
        return "docker exec -it $ContainerName bash -lc '$setup; rostopic hz $Topic'"
    }

    if ($N -gt 0) {
        return "docker exec -it $ContainerName bash -lc '$setup; rostopic echo -n $N $Topic'"
    }

    return "docker exec -it $ContainerName bash -lc '$setup; rostopic echo $Topic'"
}

if ([string]::IsNullOrWhiteSpace($ContainerName)) {
    $ContainerName = Find-ContainerByImageHint -Hint $ImageHint
}

if ([string]::IsNullOrWhiteSpace($ContainerName) -or -not (Test-ContainerRunning -Name $ContainerName)) {
    $runningNames = docker ps --format "{{.Names}}"
    $runningText = (($runningNames | Out-String).Trim())
    if ([string]::IsNullOrWhiteSpace($runningText)) {
        $runningText = "(none)"
    }

    throw "No matching GeGI container is running. Running containers: $runningText`nStart one first, e.g. .\start_gegi_reconstruction.ps1, then run .\watch_gegi_topics.ps1 -ContainerName <name>."
}

Write-Host "Opening watcher windows for container: $ContainerName"
Write-Host "Mode: $Mode"
if ($Mode -eq "echo" -and $SampleCount -gt 0) {
    Write-Host "SampleCount: $SampleCount (rostopic echo -n)"
}

foreach ($topic in $Topics) {
    $cmd = New-RosTopicCommand -Topic $topic -SelectedMode $Mode -N $SampleCount
    $startupCmd = "`$Host.UI.RawUI.WindowTitle = 'GeGI $Mode $topic'; $cmd"

    # Keep each window open for streaming output.
    Start-Process powershell -ArgumentList @(
        "-NoExit",
        "-Command",
        $startupCmd
    ) | Out-Null
}

Write-Host "Launched $($Topics.Count) watcher windows."
Write-Host "Tip: Use Ctrl+C in a watcher window to stop that topic monitor."
