param(
    [string]$ExePath = ".\\build\\windows-mingw-release\\PathTracer.exe",
    [string]$Scene = "example-scenes-cg25\\living-room",
    [int]$Spp = 1,
    [int]$Repeats = 1,
    [string]$CompilerBin = "C:\\Compilers\\msys64\\mingw64\\bin",
    [int[]]$Candidates = @(),
    [string]$OutCsv = ".\\tmp\\omp_tuning_results.csv"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

if (-not (Test-Path $ExePath)) {
    throw "Executable not found: $ExePath"
}

$logical = [Environment]::ProcessorCount
if ($Candidates.Count -eq 0) {
    $defaults = @(1,2,4,6,8,10,12,14,16,18,20,24,28,32,$logical)
    $Candidates = $defaults | Where-Object { $_ -ge 1 -and $_ -le $logical } | Sort-Object -Unique
}

if ($Candidates.Count -eq 0) {
    throw "No valid thread candidates."
}

if ($CompilerBin -and (Test-Path $CompilerBin)) {
    $env:PATH = "$CompilerBin;$env:PATH"
}

function Invoke-Render([int]$Threads) {
    $env:OMP_NUM_THREADS = "$Threads"
    $env:OMP_DYNAMIC = "false"
    $env:OMP_WAIT_POLICY = "ACTIVE"

    Remove-Item Env:PT_DEBUG_PIXEL,Env:PT_DEBUG_MAX_SAMPLES,Env:PT_DEBUG_MAX_BOUNCES,Env:PT_WRITE_LINEAR -ErrorAction SilentlyContinue

    $cmdLine = '"' + $ExePath + '" "' + $Scene + '" ' + $Spp + ' 2>&1'
    $log = (& cmd /c $cmdLine) | Out-String
    $clean = $log.Replace("`r", " ").Replace("`n", " ")
    $m = [regex]::Match($clean, "Done in ([0-9]+(?:[\\.,][0-9]+)?)s")
    if (-not $m.Success) {
        throw "Cannot parse render time for threads=$Threads. Output:`n$log"
    }
    $secText = $m.Groups[1].Value.Replace(',', '.')
    return [double]$secText
}

$results = @()
Write-Host "Logical CPU threads: $logical"
Write-Host "Testing candidates: $($Candidates -join ', ')"
Write-Host "Scene: $Scene, spp=$Spp, repeats=$Repeats"

foreach ($t in $Candidates) {
    $times = @()
    for ($i = 1; $i -le $Repeats; $i++) {
        $sec = Invoke-Render -Threads $t
        $times += $sec
        Write-Host ("threads={0,2} run={1} time={2:N3}s" -f $t, $i, $sec)
    }

    $avg = ($times | Measure-Object -Average).Average
    $min = ($times | Measure-Object -Minimum).Minimum
    $max = ($times | Measure-Object -Maximum).Maximum

    $results += [pscustomobject]@{
        Threads = $t
        AvgSec  = [Math]::Round($avg, 3)
        MinSec  = [Math]::Round($min, 3)
        MaxSec  = [Math]::Round($max, 3)
    }
}

$sorted = $results | Sort-Object AvgSec, MinSec

$outDir = Split-Path -Parent $OutCsv
if ($outDir -and -not (Test-Path $outDir)) {
    New-Item -ItemType Directory -Path $outDir | Out-Null
}
$sorted | Export-Csv -NoTypeInformation -Encoding UTF8 -Path $OutCsv

Write-Host ""
Write-Host "=== OMP tuning result (lower is better) ==="
$sorted | Format-Table -AutoSize

$best = $sorted[0]
Write-Host ""
Write-Host ("Best OMP_NUM_THREADS = {0}, AvgSec = {1:N3}s" -f $best.Threads, $best.AvgSec)
Write-Host "Recommended environment before rendering:"
Write-Host ('$env:OMP_NUM_THREADS = "' + $best.Threads + '"')
Write-Host '$env:OMP_DYNAMIC   = "false"'
Write-Host ('$env:PATH = "' + $CompilerBin + ';$env:PATH"')
Write-Host ('Results saved to: ' + $OutCsv)
