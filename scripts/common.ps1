# Shared helpers for the run_*.ps1 scripts (dot-sourced).
$Root = Split-Path $PSScriptRoot -Parent
$Bin  = Join-Path $Root "kalibr_cpp\build"

# The build bundles all runtime DLLs next to the executables, so nothing has to
# be added to PATH. Only for builds made with -DKALIBR_BUNDLE_DLLS=OFF: set
# $env:MSYS2_ROOT to the MSYS2 installation the tools were built with.
if ($env:MSYS2_ROOT) {
  $ucrt = [System.IO.Path]::Combine($env:MSYS2_ROOT, "ucrt64", "bin")
  if (Test-Path -LiteralPath $ucrt) { $env:PATH = "$ucrt;$env:PATH" }
  else { Write-Warning "MSYS2_ROOT is set but '$ucrt' does not exist - ignored." }
}

if (-not (Test-Path -LiteralPath (Join-Path $Bin "kalibr_calibrate_cameras.exe"))) {
  Write-Warning "Executables not found in $Bin - build first (scripts/build.sh) or use a release zip."
}

function Resolve-RepoPath([string]$p) {
  if ([System.IO.Path]::IsPathRooted($p)) { return $p }
  return (Join-Path $Root $p)
}
