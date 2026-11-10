# Camera calibration (intrinsics + stereo extrinsics) on an EuRoC/ASL sequence with kalibr_cpp.
# usage: powershell -ExecutionPolicy Bypass -File scripts\run_cam_calib.ps1 [-Seq cam_april] [-Name cam_april] [-Freq 4]
param([double]$Freq = 0,
      [string]$Name = "cam_april",
      [string]$Seq = "cam_april",
      [string]$Target = "april_6x6.yaml",
      [string]$Models = "pinhole-radtan",
      [string]$DatasetRoot = "dataset\calibration_datasets")
. "$PSScriptRoot\common.ps1"

$ds  = Join-Path (Resolve-RepoPath $DatasetRoot) $Seq
$out = Join-Path $Root "results\$Name"
New-Item -ItemType Directory -Force $out | Out-Null
$a = @("--target", "$ds\$Target", "--dataset", "$ds\mav0", "--topics", "cam0", "cam1",
       "--models", $Models, $Models, "--output", "$out\$Name")
if ($Freq -gt 0) { $a += @("--bag-freq", "$Freq") }
$t0 = Get-Date
& "$Bin\kalibr_calibrate_cameras.exe" @a *> "$out\log.txt"
"EXIT=$LASTEXITCODE  runtime_s=$(((Get-Date) - $t0).TotalSeconds)" | Add-Content "$out\log.txt"
Get-Content "$out\log.txt" -Tail 1
