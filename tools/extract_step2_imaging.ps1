param(
    [string]$DataDir = "./data",
    [string]$OutputFile = "./data/step2_imaging_extracted.csv",
    [ValidateSet("auto", "3d", "raster")]
    [string]$SourcePreference = "auto",
    [int]$TopPeaks = 3,
    [double]$MinPeakSeparationM = 0.10,
    [string]$GroundTruthCsv = "",
    # Intensity-weighted centroid radius (m) for sub-pixel peak refinement.
    # The brightest cell is found first, then the prediction is refined to the
    # intensity-weighted centre of mass of all cells within this radius. This
    # removes the heatmap-grid quantisation floor (~0.57 deg / 5 mm cell) and the
    # systematic on-axis offset (no grid cell sits at the origin). Set to 0 to
    # disable and use the raw argmax cell. Keep < 0.5 * MinPeakSeparationM so
    # neighbouring sources are not pulled into each other's centroid.
    [double]$CentroidRadiusM = 0.03,
    # Aggregated per-configuration angular-resolution summary. Empty = derive
    # "<OutputFile>_summary.csv" next to the main output.
    [string]$SummaryFile = ""
)

$ErrorActionPreference = "Stop"

function To-Double {
    param([string]$Value)
    if ([string]::IsNullOrWhiteSpace($Value)) { return 0.0 }
    return [double]::Parse($Value, [System.Globalization.CultureInfo]::InvariantCulture)
}

function Get-PeakCentroid {
    # Intensity-weighted centre of mass of all cells within RadiusM (in the Y-Z
    # plane) of the argmax cell (cy, cz). Returns the refined (y, z); falls back
    # to the argmax if no positive-weight cells are in range.
    param(
        [array]$Rows,
        [string]$ValueCol,
        [double]$cy,
        [double]$cz,
        [double]$RadiusM
    )

    $r2 = $RadiusM * $RadiusM
    $sumW = 0.0
    $sumY = 0.0
    $sumZ = 0.0

    foreach ($row in $Rows) {
        $v = To-Double $row.$ValueCol
        if ($v -le 0) { continue }
        $y = To-Double $row.y
        $z = To-Double $row.z
        $dy = $y - $cy
        $dz = $z - $cz
        if (($dy * $dy + $dz * $dz) -le $r2) {
            $sumW += $v
            $sumY += $v * $y
            $sumZ += $v * $z
        }
    }

    if ($sumW -le 0) {
        return @{ y = $cy; z = $cz }
    }
    return @{ y = ($sumY / $sumW); z = ($sumZ / $sumW) }
}

function Get-RunPrefixes {
    param([string]$Root)

    $files = Get-ChildItem -Path $Root -File -ErrorAction Stop |
        Where-Object { $_.Name -match "_heatmap_(3d|raster)\.csv$" }

    $prefixes = @{}
    foreach ($f in $files) {
        $prefix = $f.BaseName -replace "_heatmap_(3d|raster)$", ""
        $prefixes[$prefix] = $true
    }
    return $prefixes.Keys | Sort-Object
}

function Get-HeatmapPath {
    param(
        [string]$Root,
        [string]$Prefix,
        [string]$Preference
    )

    $path3d = Join-Path $Root ("{0}_heatmap_3d.csv" -f $Prefix)
    $pathRaster = Join-Path $Root ("{0}_heatmap_raster.csv" -f $Prefix)

    $has3d = (Test-Path $path3d)
    $hasRaster = (Test-Path $pathRaster)

    if ($Preference -eq "3d") {
        if ($has3d) { return @{ Path = $path3d; Kind = "3d" } }
        return $null
    }
    if ($Preference -eq "raster") {
        if ($hasRaster) { return @{ Path = $pathRaster; Kind = "raster" } }
        return $null
    }

    # auto: prefer 3d when non-empty, fallback to raster
    if ($has3d) {
        $rows3d = Import-Csv -Path $path3d
        if ($rows3d.Count -gt 0) {
            return @{ Path = $path3d; Kind = "3d" }
        }
    }
    if ($hasRaster) {
        return @{ Path = $pathRaster; Kind = "raster" }
    }

    return $null
}

function Select-Peaks {
    param(
        [array]$Rows,
        [string]$Kind,
        [int]$MaxPeaks,
        [double]$MinSepM
    )

    if (-not $Rows -or $Rows.Count -eq 0) { return @() }

    $valueCol = if ($Kind -eq "3d") { "dose_rate_uSv_h" } else { "intensity" }

    $ordered = $Rows |
        Where-Object { $_.$valueCol -ne $null -and $_.x -ne $null -and $_.y -ne $null -and $_.z -ne $null } |
        Sort-Object @{ Expression = { To-Double $_.$valueCol }; Descending = $true }

    $selected = @()
    foreach ($row in $ordered) {
        if ($selected.Count -ge $MaxPeaks) { break }

        $x = To-Double $row.x
        $y = To-Double $row.y
        $z = To-Double $row.z
        $v = To-Double $row.$valueCol

        $tooClose = $false
        foreach ($p in $selected) {
            $dy = $y - $p.y
            $dz = $z - $p.z
            $d = [math]::Sqrt($dy * $dy + $dz * $dz)
            if ($d -lt $MinSepM) {
                $tooClose = $true
                break
            }
        }

        if (-not $tooClose) {
            $selected += [pscustomobject]@{
                x = $x
                y = $y
                z = $z
                value = $v
                metric = $valueCol
            }
        }
    }

    return $selected
}

function Get-ActivitySummary {
    param(
        [string]$Root,
        [string]$Prefix
    )

    $path = Join-Path $Root ("{0}_activity.csv" -f $Prefix)
    if (-not (Test-Path $path)) {
        return [pscustomobject]@{
            activity_rows = 0
            window_count = 0
            dead_time_mean = 0.0
            live_time_mean_s = 0.0
            total_activity_mean_mbq = 0.0
        }
    }

    $rows = Import-Csv -Path $path
    if (-not $rows -or $rows.Count -eq 0) {
        return [pscustomobject]@{
            activity_rows = 0
            window_count = 0
            dead_time_mean = 0.0
            live_time_mean_s = 0.0
            total_activity_mean_mbq = 0.0
        }
    }

    # Dead time now comes from the detector hardware run-info (percent -> fraction).
    $dtPctMean = ($rows | Measure-Object -Property detector_run_dead_time_percent -Average).Average
    $dtMean = if ($null -ne $dtPctMean) { $dtPctMean / 100.0 } else { 0.0 }
    $ltMean = ($rows | Measure-Object -Property live_time_s -Average).Average

    $windows = $rows | Group-Object -Property timestamp_s
    $windowTotals = @()
    foreach ($w in $windows) {
        $sumMbq = ($w.Group | Measure-Object -Property activity_MBq -Sum).Sum
        $windowTotals += [double]$sumMbq
    }

    $totalMean = 0.0
    if ($windowTotals.Count -gt 0) {
        $totalMean = ($windowTotals | Measure-Object -Average).Average
    }

    return [pscustomobject]@{
        activity_rows = $rows.Count
        window_count = $windows.Count
        dead_time_mean = [double]$dtMean
        live_time_mean_s = [double]$ltMean
        total_activity_mean_mbq = [double]$totalMean
    }
}

function Build-GroundTruthLookup {
    param([string]$Path)

    if ([string]::IsNullOrWhiteSpace($Path) -or -not (Test-Path $Path)) {
        return @{}
    }

    $rows = Import-Csv -Path $Path
    $lookup = @{}

    foreach ($r in $rows) {
        if (-not $r.run_prefix) { continue }
        if (-not $lookup.ContainsKey($r.run_prefix)) {
            $lookup[$r.run_prefix] = @()
        }
        $lookup[$r.run_prefix] += $r
    }

    return $lookup
}

function Get-TruthForPeak {
    param(
        [hashtable]$Lookup,
        [string]$Prefix,
        [int]$PeakRank
    )

    if (-not $Lookup.ContainsKey($Prefix)) { return $null }

    $rows = $Lookup[$Prefix]
    if ($rows.Count -eq 1) { return $rows[0] }

    # If source_idx exists in ground truth, try rank-based match.
    foreach ($r in $rows) {
        if ($r.PSObject.Properties.Name -contains "source_idx") {
            if ([int]$r.source_idx -eq $PeakRank) {
                return $r
            }
        }
    }

    return $rows[0]
}

function Compute-AngularErrorDeg {
    param(
        [double]$xt,
        [double]$yt,
        [double]$zt,
        [double]$xp,
        [double]$yp,
        [double]$zp
    )

    $rt = [math]::Sqrt($xt*$xt + $yt*$yt + $zt*$zt)
    $rp = [math]::Sqrt($xp*$xp + $yp*$yp + $zp*$zp)
    if ($rt -le 0 -or $rp -le 0) { return 0.0 }

    $cosT = ($xt*$xp + $yt*$yp + $zt*$zp) / ($rt*$rp)
    if ($cosT -gt 1.0) { $cosT = 1.0 }
    if ($cosT -lt -1.0) { $cosT = -1.0 }

    return [math]::Acos($cosT) * 180.0 / [math]::PI
}

if (-not (Test-Path $DataDir)) {
    throw "Data directory not found: $DataDir"
}

$prefixes = Get-RunPrefixes -Root $DataDir
if (-not $prefixes -or $prefixes.Count -eq 0) {
    throw "No heatmap files found in $DataDir"
}

$truthLookup = Build-GroundTruthLookup -Path $GroundTruthCsv

$outRows = @()

foreach ($prefix in $prefixes) {
    $heatmap = Get-HeatmapPath -Root $DataDir -Prefix $prefix -Preference $SourcePreference
    if ($null -eq $heatmap) { continue }

    $rows = Import-Csv -Path $heatmap.Path
    $peaks = Select-Peaks -Rows $rows -Kind $heatmap.Kind -MaxPeaks $TopPeaks -MinSepM $MinPeakSeparationM
    if ($peaks.Count -eq 0) { continue }

    # Sub-pixel refinement: replace each argmax cell with the intensity-weighted
    # centroid of its neighbourhood, removing the 5 mm grid quantisation floor.
    if ($CentroidRadiusM -gt 0) {
        $valueCol = if ($heatmap.Kind -eq "3d") { "dose_rate_uSv_h" } else { "intensity" }
        foreach ($p in $peaks) {
            $c = Get-PeakCentroid -Rows $rows -ValueCol $valueCol -cy $p.y -cz $p.z -RadiusM $CentroidRadiusM
            $p.y = $c.y
            $p.z = $c.z
        }
    }

    $summary = Get-ActivitySummary -Root $DataDir -Prefix $prefix

    $rank = 0
    foreach ($p in $peaks) {
        $rank += 1

        $truth = Get-TruthForPeak -Lookup $truthLookup -Prefix $prefix -PeakRank $rank

        $xt = $null
        $yt = $null
        $zt = $null
        $eYZ = $null
        $theta = $null

        $isotopePattern = ""
        $trueSourceCount = ""
        $distanceM = ""
        $countTimeS = ""
        $separationCm = ""
        $sourceIdx = ""

        if ($null -ne $truth) {
            if ($truth.PSObject.Properties.Name -contains "isotope_pattern") { $isotopePattern = $truth.isotope_pattern }
            if ($truth.PSObject.Properties.Name -contains "true_source_count") { $trueSourceCount = $truth.true_source_count }
            if ($truth.PSObject.Properties.Name -contains "distance_m") { $distanceM = $truth.distance_m }
            if ($truth.PSObject.Properties.Name -contains "count_time_s") { $countTimeS = $truth.count_time_s }
            if ($truth.PSObject.Properties.Name -contains "separation_cm") { $separationCm = $truth.separation_cm }
            if ($truth.PSObject.Properties.Name -contains "source_idx") { $sourceIdx = $truth.source_idx }

            if (($truth.PSObject.Properties.Name -contains "x_true_m") -and
                ($truth.PSObject.Properties.Name -contains "y_true_m") -and
                ($truth.PSObject.Properties.Name -contains "z_true_m")) {

                $xt = To-Double $truth.x_true_m
                $yt = To-Double $truth.y_true_m
                $zt = To-Double $truth.z_true_m

                $dy = $p.y - $yt
                $dz = $p.z - $zt
                $eYZ = [math]::Sqrt($dy*$dy + $dz*$dz)
                $theta = Compute-AngularErrorDeg -xt $xt -yt $yt -zt $zt -xp $p.x -yp $p.y -zp $p.z
            }
        }

        $outRows += [pscustomobject]@{
            run_prefix = $prefix
            heatmap_source = $heatmap.Kind
            peak_rank = $rank
            peak_metric = $p.metric
            peak_value = [math]::Round($p.value, 8)
            x_pred_m = [math]::Round($p.x, 6)
            y_pred_m = [math]::Round($p.y, 6)
            z_pred_m = [math]::Round($p.z, 6)
            isotope_pattern = $isotopePattern
            true_source_count = $trueSourceCount
            distance_m = $distanceM
            count_time_s = $countTimeS
            separation_cm = $separationCm
            source_idx = $sourceIdx
            x_true_m = $xt
            y_true_m = $yt
            z_true_m = $zt
            e_yz_m = $eYZ
            theta_deg = $theta
            activity_rows = $summary.activity_rows
            window_count = $summary.window_count
            dead_time_mean = [math]::Round($summary.dead_time_mean, 8)
            live_time_mean_s = [math]::Round($summary.live_time_mean_s, 4)
            total_activity_mean_mbq = [math]::Round($summary.total_activity_mean_mbq, 6)
        }
    }
}

$outDir = Split-Path -Parent $OutputFile
if (-not [string]::IsNullOrWhiteSpace($outDir) -and -not (Test-Path $outDir)) {
    New-Item -Path $outDir -ItemType Directory | Out-Null
}

$outRows |
    Sort-Object run_prefix, peak_rank |
    Export-Csv -Path $OutputFile -NoTypeInformation

Write-Host "Step 2 extraction complete."
Write-Host ("  Runs processed: {0}" -f (($outRows | Select-Object -ExpandProperty run_prefix -Unique).Count))
Write-Host ("  Rows written:   {0}" -f $outRows.Count)
Write-Host ("  Output:         {0}" -f $OutputFile)
if ($CentroidRadiusM -gt 0) {
    Write-Host ("  Peak mode:      sub-pixel centroid (radius {0} m)" -f $CentroidRadiusM)
} else {
    Write-Host "  Peak mode:      raw argmax cell"
}

# ── Aggregated angular-resolution summary ───────────────────────────────────
# Group repeats by configuration (isotope + true source position) and report
# the angular-error statistics. theta_rms_deg is the standard single-number
# angular-resolution figure; theta_std_deg is the run-to-run repeatability.
$withTruth = @($outRows | Where-Object { $null -ne $_.theta_deg })
if ($withTruth.Count -gt 0) {
    if ([string]::IsNullOrWhiteSpace($SummaryFile)) {
        $dir = Split-Path -Parent $OutputFile
        $base = [System.IO.Path]::GetFileNameWithoutExtension($OutputFile)
        $ext = [System.IO.Path]::GetExtension($OutputFile)
        if ([string]::IsNullOrWhiteSpace($ext)) { $ext = ".csv" }
        $SummaryFile = Join-Path $dir ("{0}_summary{1}" -f $base, $ext)
    }

    $summaryRows = @()
    $groups = $withTruth | Group-Object {
        "{0}|{1}|{2}|{3}|{4}" -f $_.isotope_pattern, $_.x_true_m, $_.y_true_m, $_.z_true_m, $_.peak_rank
    }

    foreach ($g in $groups) {
        $thetas = @($g.Group | ForEach-Object { [double]$_.theta_deg })
        $eyzs = @($g.Group | ForEach-Object { [double]$_.e_yz_m })
        $n = $thetas.Count

        $thetaMean = ($thetas | Measure-Object -Average).Average
        $thetaRms = [math]::Sqrt((@($thetas | ForEach-Object { $_ * $_ }) | Measure-Object -Average).Average)
        $eyzMean = ($eyzs | Measure-Object -Average).Average
        $eyzRms = [math]::Sqrt((@($eyzs | ForEach-Object { $_ * $_ }) | Measure-Object -Average).Average)

        $thetaStd = 0.0
        $eyzStd = 0.0
        if ($n -gt 1) {
            $tVar = (@($thetas | ForEach-Object { ($_ - $thetaMean) * ($_ - $thetaMean) }) | Measure-Object -Sum).Sum / ($n - 1)
            $thetaStd = [math]::Sqrt($tVar)
            $eVar = (@($eyzs | ForEach-Object { ($_ - $eyzMean) * ($_ - $eyzMean) }) | Measure-Object -Sum).Sum / ($n - 1)
            $eyzStd = [math]::Sqrt($eVar)
        }

        $first = $g.Group[0]
        $summaryRows += [pscustomobject]@{
            isotope_pattern = $first.isotope_pattern
            x_true_m = $first.x_true_m
            y_true_m = $first.y_true_m
            z_true_m = $first.z_true_m
            peak_rank = $first.peak_rank
            n_runs = $n
            theta_mean_deg = [math]::Round($thetaMean, 4)
            theta_rms_deg = [math]::Round($thetaRms, 4)
            theta_std_deg = [math]::Round($thetaStd, 4)
            e_yz_mean_m = [math]::Round($eyzMean, 5)
            e_yz_rms_m = [math]::Round($eyzRms, 5)
            e_yz_std_m = [math]::Round($eyzStd, 5)
            run_prefixes = (@($g.Group | ForEach-Object { $_.run_prefix }) | Sort-Object -Unique) -join ";"
        }
    }

    $summaryRows |
        Sort-Object isotope_pattern, y_true_m, z_true_m, peak_rank |
        Export-Csv -Path $SummaryFile -NoTypeInformation

    Write-Host ("  Summary:        {0} ({1} configuration(s))" -f $SummaryFile, $summaryRows.Count)
}

if ([string]::IsNullOrWhiteSpace($GroundTruthCsv)) {
    Write-Host "Tip: pass -GroundTruthCsv <path> to auto-compute e_yz_m and theta_deg."
}
