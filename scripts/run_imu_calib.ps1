# Camera-IMU (spatial + temporal) calibration on an EuRoC/ASL sequence with kalibr_cpp.
# usage: powershell -ExecutionPolicy Bypass -File scripts\run_imu_calib.ps1 [-CamChain results\cam_april\cam_april-camchain.yaml] [-Name imu_april] [-Extra "--bag-freq 10"]
param([string]$CamChain = "results\cam_april\cam_april-camchain.yaml",
      [string]$Name = "imu_april",
      [string]$Seq = "imu_april",
      [string]$Target = "april_6x6.yaml",
      [string]$DatasetRoot = "dataset\calibration_datasets",
      [string]$Extra = "")
. "$PSScriptRoot\common.ps1"

$ds  = Join-Path (Resolve-RepoPath $DatasetRoot) $Seq
$out = Join-Path $Root "results\$Name"
New-Item -ItemType Directory -Force $out | Out-Null
$a = @("--dataset", "$ds\mav0", "--cams", (Resolve-RepoPath $CamChain), "--imu", "$ds\mav0\imu0\sensor.yaml",
       "--target", "$ds\$Target", "--output", "$out\$Name")
if ($Extra -ne "") { $a += $Extra.Split(" ") }
$t0 = Get-Date
& "$Bin\kalibr_calibrate_imu_camera.exe" @a *> "$out\log.txt"
"EXIT=$LASTEXITCODE  runtime_s=$(((Get-Date) - $t0).TotalSeconds)" | Add-Content "$out\log.txt"
Get-Content "$out\log.txt" -Tail 1
