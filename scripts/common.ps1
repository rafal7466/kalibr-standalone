# Shared helpers for the run_*.ps1 scripts (dot-sourced).
$Root = Split-Path $PSScriptRoot -Parent
$Bin  = Join-Path $Root "kalibr_cpp\build"

# Locate the MSYS2 UCRT64 runtime DLLs: $env:MSYS2_ROOT, then the private
# toolchain in tools\msys64, then the default system install C:\msys64.
$candidates = @()
if ($env:MSYS2_ROOT) { $candidates += $env:MSYS2_ROOT }
$candidates += (Join-Path $Root "tools\msys64"), "C:\msys64"
foreach ($c in $candidates) {
  $ucrt = Join-Path $c "ucrt64\bin"
  if (Test-Path $ucrt) { $env:PATH = "$ucrt;$env:PATH"; break }
}

function Resolve-RepoPath([string]$p) {
  if ([System.IO.Path]::IsPathRooted($p)) { return $p }
  return (Join-Path $Root $p)
}
