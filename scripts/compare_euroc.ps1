# Compare kalibr_cpp results with the official EuRoC calibration (reference\cam*_sensor.yaml).
# usage: powershell -ExecutionPolicy Bypass -File scripts\compare_euroc.ps1 -CamChain results\cam_april\cam_april-camchain.yaml -ImuCam results\imu_april\imu_april-camchain-imucam.yaml -Report results\report.md
param([string]$CamChain = "",
      [string]$ImuCam = "",
      [string]$Report = "results\report.md")
. "$PSScriptRoot\common.ps1"

$a = @("--ref", (Join-Path $Root "reference\cam0_sensor.yaml"), (Join-Path $Root "reference\cam1_sensor.yaml"))
if ($CamChain -ne "") { $a += @("--camchain", (Resolve-RepoPath $CamChain)) }
if ($ImuCam   -ne "") { $a += @("--imucam",   (Resolve-RepoPath $ImuCam)) }
$a += @("--report", (Resolve-RepoPath $Report))
& "$Bin\kalibr_compare_euroc.exe" @a
